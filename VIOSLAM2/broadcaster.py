import numpy as np
import depthai as dai
from multiprocessing import shared_memory
from controls.connect import connect_UART2
from controls.attitude import request_attitude_messages
from controls.attitude import get_attitude

def broadcaster(camera_frame_mutex, camera_calibration_mutex, attitude_mutex):
    W, H = 640, 400
    FPS = 30.0
    # 1. Connect to the shared memory for RGB, gray, and depth
    shm_rgb = shared_memory.SharedMemory(name="oak_rgb")
    shm_gray = shared_memory.SharedMemory(name="oak_gray")
    shm_depth = shared_memory.SharedMemory(name="oak_depth")
    shm_calib = shared_memory.SharedMemory(name="oak_calib")
    # 2. Connect to shared memory for attitude
    shm_attitude = shared_memory.SharedMemory(name="attitude")
    # 3. Create numpy arrays that use the shared memory buffers for RGB, gray, depth, and camera calibration matrix
    shared_rgb = np.ndarray((H, W, 3), dtype=np.uint8, buffer=shm_rgb.buf)
    shared_gray = np.ndarray((H, W), dtype=np.uint8, buffer=shm_gray.buf)
    shared_depth = np.ndarray((H, W), dtype=np.uint16, buffer=shm_depth.buf)
    shared_calib = np.ndarray((3, 3), dtype=np.float64, buffer=shm_calib.buf)
    # 4. Create numpy array for the shared memory buffer for attitude
    shared_attitude = np.ndarray((3,), dtype=np.float64, buffer=shm_attitude.buf)
    print("Broadcaster shared memory connected")
    # 5. Connect to the Pixhawk via UART2 and request the ATTITUDE message stream at 50ms intervals
    master_uart2 = connect_UART2()
    request_attitude_messages(master_uart2, 50)

    with dai.Device() as device:
        with dai.Pipeline(device) as pipeline:
            print("Starting up camera")
            # 6. Create the three camera nodes for RGB, left gray, and right gray, and the stereo depth node.
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
            print("Camera running")
            # 7. Get the camera calibration data and write it to the shared memory
            calib = device.getCalibration()
            K = np.array(calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_B, W, H), dtype=np.float64)
            with camera_calibration_mutex:
                np.copyto(shared_calib, K)
            print("Camera calibration data written to shared memory.")
            print("Broadcaster entering main loop to get camera frames and attitude data")
            
            while pipeline.isRunning():
                msg_group = sync_q.get()
                # 8. Get the attitude data from the Pixhawk and store it in the local attitude variable. 
                attitude = get_attitude(master_uart2)
                if msg_group is None:
                    continue
                # 9. Get the RGB, gray, and depth frames from the camera and store them in the the local frame variables.
                rgb_frame = msg_group["rgb"].getCvFrame()
                gray_frame = msg_group["left"].getCvFrame()
                depth_frame = msg_group["depth"].getFrame()
                # 10. Force RGB to 3 channels just in case it brings an alpha channel (BGRA)
                if len(rgb_frame.shape) == 3 and rgb_frame.shape[2] == 4:
                    rgb_frame = rgb_frame[:, :, :3]
                # 11. This section is a critical section as we must aquire the mutex/lock before 
                #     writing to the shared memory, and release it immediately after.
                with attitude_mutex:
                    if attitude is not None:
                        shared_attitude[:] = attitude
                try:
                    # 12. This section is a critical section as we must aquire the mutex/lock before 
                    #    copying from the shared memory, and release it immediately after.
                    with camera_frame_mutex:
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