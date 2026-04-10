import numpy as np
import depthai as dai
from multiprocessing import shared_memory

def camera_broadcaster(lock):
    """
    Connects to the OAK-D, pulls frames, and writes them to Shared Memory.
    Runs in its own dedicated CPU process.
    """
    W, H = 640, 400

    # Connect to the Shared Memory blocks (created by the main script)
    shm_rgb = shared_memory.SharedMemory(name="oak_rgb")
    shm_gray = shared_memory.SharedMemory(name="oak_gray")
    shm_depth = shared_memory.SharedMemory(name="oak_depth")

    # Map numpy arrays directly to the shared RAM
    shared_rgb = np.ndarray((H, W, 3), dtype=np.uint8, buffer=shm_rgb.buf)
    shared_gray = np.ndarray((H, W), dtype=np.uint8, buffer=shm_gray.buf)
    shared_depth = np.ndarray((H, W), dtype=np.uint16, buffer=shm_depth.buf)

    print("[Broadcaster] Shared memory connected. Booting Camera Pipeline...")

    FPS = 30.0
    with dai.Device() as device:
        with dai.Pipeline(device) as pipeline:
            
            # Create Nodes
            cam_rgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
            cam_left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
            cam_right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
            
            stereo = pipeline.create(dai.node.StereoDepth)
            sync = pipeline.create(dai.node.Sync)

            # Stereo Settings
            stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.FAST_DENSITY)
            stereo.setLeftRightCheck(True)
            stereo.setSubpixel(True)
            stereo.setDepthAlign(dai.StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_LEFT)

            # Request Outputs
            rgb_out = cam_rgb.requestOutput(size=(W, H), fps=FPS)
            left_out = cam_left.requestOutput(size=(W, H), fps=FPS)
            right_out = cam_right.requestOutput(size=(W, H), fps=FPS)

            # Link Nodes
            left_out.link(stereo.left)
            right_out.link(stereo.right)
            
            rgb_out.link(sync.inputs["rgb"])
            stereo.rectifiedLeft.link(sync.inputs["left"])
            stereo.depth.link(sync.inputs["depth"])

            sync_q = sync.out.createOutputQueue(maxSize=4, blocking=False)
            pipeline.start()
            
            print("[Broadcaster] Camera running at 30 FPS. Broadcasting...")

            while pipeline.isRunning():
                msg_group = sync_q.get()
                if msg_group is None:
                    continue

                rgb_frame = msg_group["rgb"].getCvFrame()
                gray_frame = msg_group["left"].getCvFrame()
                depth_frame = msg_group["depth"].getFrame()

                # --- CRITICAL SECTION: Write to Shared Memory ---
                with lock:
                    np.copyto(shared_rgb, rgb_frame)
                    np.copyto(shared_gray, gray_frame)
                    np.copyto(shared_depth, depth_frame)
                # ------------------------------------------------