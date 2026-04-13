import multiprocessing as mp
from multiprocessing import shared_memory
from broadcaster import camera_broadcaster
from calculatevioslam_updateposition1 import calculatevioslam_updateposition

if __name__ == "__main__":
    # 1. Set the multiprocessing start method to 'spawn' this is so that when a new process is started it
    #    doesn't inherit the memory of the parent process, and instead creates its own memory space and also
    #    its own python interpreter. Effectively isolating the processes from each other except for the shared memory.
    mp.set_start_method('spawn', force=True)
    
    W, H = 640, 400
    RGB_BYTES = W * H * 3
    GRAY_BYTES = W * H
    DEPTH_BYTES = W * H * 2  # 16-bit depth uses 2 bytes per pixel
    CALIB_BYTES = 3 * 3 * 8

    # 2. Create the shared memory variables for RGB, gray, depth frames, and camera calibration matrix
    print("Broadcaster tester allocating shared memory...")
    shm_rgb = shared_memory.SharedMemory(create=True, size=RGB_BYTES, name="oak_rgb")
    shm_gray = shared_memory.SharedMemory(create=True, size=GRAY_BYTES, name="oak_gray")
    shm_depth = shared_memory.SharedMemory(create=True, size=DEPTH_BYTES, name="oak_depth")
    shm_calib = shared_memory.SharedMemory(create=True, size=CALIB_BYTES, name="oak_calib")
    
    # 2. Create the mutex lock for the shared memory
    camera_frame_mutex = mp.Lock()
    camera_calibration_mutex = mp.Lock()
    uart_tx_port_mutex = mp.Lock()

    # 3. Define the independent processes
    broadcaster_process = mp.Process(target=camera_broadcaster, args=(camera_frame_mutex, camera_calibration_mutex))
    calculatevioslam_updateposition_process = mp.Process(target=calculatevioslam_updateposition, args=(camera_frame_mutex, uart_tx_port_mutex, camera_calibration_mutex))

    try:
        # 4. Start the processes
        broadcaster_process.start()
        calculatevioslam_updateposition_process.start()

        # 5. Wait until the viewer window is closed
        calculatevioslam_updateposition_process.join()
        
        # 6. kill the broadcaster
        broadcaster_process.terminate()
        broadcaster_process.join()
    except KeyboardInterrupt:
        print("\nBroadcaster tester caught keyboard interrupt. Shutting down...")
    finally:
        # 7. Cleanup all three blocks
        print("Broadcaster tester cleaning up shared memory...")
        shm_rgb.close()
        shm_rgb.unlink()
        shm_gray.close()
        shm_gray.unlink()
        shm_depth.close()
        shm_depth.unlink()
        print("Broadcaster tester processes terminated safely.")