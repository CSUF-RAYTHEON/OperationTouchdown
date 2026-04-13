import numpy as np
import depthai as dai
from multiprocessing import shared_memory

def camera_broadcaster(frame_shared_memory_mutex):
    W, H = 640, 400
    FPS = 30.0

    # 1. Connect to the three shared memory blocks for RGB, gray, and depth frames
    shm_rgb = shared_memory.SharedMemory(name="oak_rgb")
    shm_gray = shared_memory.SharedMemory(name="oak_gray")
    shm_depth = shared_memory.SharedMemory(name="oak_depth")
    shared_rgb = np.ndarray((H, W, 3), dtype=np.uint8, buffer=shm_rgb.buf)
    shared_gray = np.ndarray((H, W), dtype=np.uint8, buffer=shm_gray.buf)
    shared_depth = np.ndarray((H, W), dtype=np.uint16, buffer=shm_depth.buf)
    print("Broadcaster shared memory connected. Booting camera...")

    with dai.Device() as device:
        with dai.Pipeline(device) as pipeline:
            # 2. Create the three camera nodes for RGB, left gray, and right gray, and the stereo depth node.
            #    As well as the sync node to synchronize the frames from all three cameras together. Then create
            #    the output queues for the synchronized frames using the camera nodes and the stereo depth node 
            #    as the sources. And lastly start the camera frame pipeline.
            cam_rgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
            cam_left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
            cam_right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
            
            stereo = pipeline.create(dai.node.StereoDepth)
            sync = pipeline.create(dai.node.Sync)

            stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.FAST_DENSITY)
            stereo.setLeftRightCheck(True)
            stereo.setSubpixel(True)
            stereo.setDepthAlign(dai.StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_LEFT)

            rgb_out = cam_rgb.requestOutput(size=(W, H), fps=FPS, enableUndistortion=True)
            left_out = cam_left.requestOutput(size=(W, H), fps=FPS)
            right_out = cam_right.requestOutput(size=(W, H), fps=FPS)

            left_out.link(stereo.left)
            right_out.link(stereo.right)
            
            rgb_out.link(sync.inputs["rgb"])
            stereo.rectifiedLeft.link(sync.inputs["left"])
            stereo.depth.link(sync.inputs["depth"])

            sync_q = sync.out.createOutputQueue(maxSize=4, blocking=False)
            pipeline.start()
            print("Broadcaster camera running. Writing frames...")

            while pipeline.isRunning():
                msg_group = sync_q.get()
                if msg_group is None:
                    continue
                
                # 3. Get the RGB, gray, and depth frames from the camera and store them in the the local frame variables.
                rgb_frame = msg_group["rgb"].getCvFrame()
                gray_frame = msg_group["left"].getCvFrame()
                depth_frame = msg_group["depth"].getFrame()

                # 4. Force RGB to 3 channels just in case it brings an alpha channel (BGRA)
                if len(rgb_frame.shape) == 3 and rgb_frame.shape[2] == 4:
                    rgb_frame = rgb_frame[:, :, :3]

                try:
                    # 5. This section is a critical section as we must aquire the mutex/lock before 
                    #    copying from the shared memory, and release it immediately after.
                    with frame_shared_memory_mutex:
                        np.copyto(shared_rgb, rgb_frame)
                        np.copyto(shared_gray, gray_frame)
                        np.copyto(shared_depth, depth_frame)
                except Exception as e:
                    print("\nBroadcaster error, memory write failed")
                    print(f"Error Message: {e}")
                    print(f"Camera sent RGB shape: {rgb_frame.shape} | Expected: {shared_rgb.shape}")
                    print(f"Camera sent Gray shape: {gray_frame.shape} | Expected: {shared_gray.shape}")
                    print(f"Camera sent Depth shape: {depth_frame.shape} | Expected: {shared_depth.shape}")
                    print("Shutting down broadcaster...\n")
                    break