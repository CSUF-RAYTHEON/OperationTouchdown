import multiprocessing as mp
from multiprocessing import shared_memory
from vioslam.broadcaster import broadcaster
from vioslam.vio import test_latency_vio
from vioslam.slam import slam
import time
from landing.precision_landing_patrol import main
if __name__ == "__main__":
    mp.set_start_method('spawn', force=True)
    
    W, H = 640, 400
    RGB_BYTES = W * H * 3
    GRAY_BYTES = W * H
    DEPTH_BYTES = W * H * 2 
    CALIB_BYTES = 3 * 3 * 8 
    ATTITUDE_BYTES = 3 * 8 
    POSITION_BYTES = 3 * 8 
    LOCAL_POSITION_NED_BYTES = 3 * 8
    BOOL_BYTES = 2
    TARGET_BYTES = 4 * 8

    print("MISSION 1 ALLOCATING SHARED MEMORY...")
    shm_rgb = shared_memory.SharedMemory(create=True, size=RGB_BYTES, name="oak_rgb")
    shm_gray = shared_memory.SharedMemory(create=True, size=GRAY_BYTES, name="oak_gray")
    shm_depth = shared_memory.SharedMemory(create=True, size=DEPTH_BYTES, name="oak_depth")
    shm_calib = shared_memory.SharedMemory(create=True, size=CALIB_BYTES, name="oak_calib")
    shm_attitude = shared_memory.SharedMemory(create=True, size=ATTITUDE_BYTES, name="attitude")
    shm_position = shared_memory.SharedMemory(create=True, size=POSITION_BYTES, name="position")
    shm_local_position_ned = shared_memory.SharedMemory(create=True, size=LOCAL_POSITION_NED_BYTES, name="local_position_ned")
    shm_slam_enabled = shared_memory.SharedMemory(create=True, size=BOOL_BYTES, name="slam_enabled")
    shm_slam_target = shared_memory.SharedMemory(create=True, size=TARGET_BYTES, name="slam_target")
    shm_slam_trigger = shared_memory.SharedMemory(create=True, size=BOOL_BYTES, name="slam_trigger")
    print("MISSION 1 FINISHED ALLOCATING SHARED MEMORY...")

    rgb_frame_mutex = mp.Lock()
    gray_frame_mutex = mp.Lock()
    depth_frame_mutex = mp.Lock()
    attitude_mutex = mp.Lock()
    position_mutex = mp.Lock()
    local_position_ned_mutex = mp.Lock()
    slam_trigger_mutex = mp.Lock()
    slam_enabled_mutex = mp.Lock()

    from vioslam.slam import slam
    broadcaster_process = mp.Process(target=broadcaster, args=(rgb_frame_mutex, gray_frame_mutex, depth_frame_mutex, attitude_mutex, local_position_ned_mutex,))
    vio_process = mp.Process(target=test_latency_vio, args=(gray_frame_mutex, depth_frame_mutex, attitude_mutex, position_mutex, slam_trigger_mutex,))
    slam_process = mp.Process(target=slam, args=(rgb_frame_mutex, depth_frame_mutex, attitude_mutex, position_mutex, slam_enabled_mutex, slam_trigger_mutex,))
    main_process = mp.Process(target=main, args=(position_mutex,))

    try:
        broadcaster_process.start()
        time.sleep(3)
        vio_process.start()
        time.sleep(3)
        slam_process.start()
        time.sleep(3)
        main_process.start()
        time.sleep(3)
        main_process.join()
        
    except KeyboardInterrupt:
        print("MISSION 1 caught keyboard interrupt. Shutting down...")
    finally:
        broadcaster_process.terminate()
        vio_process.terminate()
        slam_process.terminate()
        main_process.terminate()

        broadcaster_process.join()
        vio_process.join()
        slam_process.join()
        main_process.join()

        print("MISSION 1 cleaning up shared memory...")
        shm_rgb.close()
        shm_rgb.unlink()
        shm_gray.close()
        shm_gray.unlink()
        shm_depth.close()
        shm_depth.unlink()
        shm_calib.close()
        shm_calib.unlink()
        shm_attitude.close()
        shm_attitude.unlink()
        shm_position.close()
        shm_position.unlink()
        shm_local_position_ned.close()
        shm_local_position_ned.unlink()
        shm_slam_enabled.close()
        shm_slam_enabled.unlink()
        shm_slam_target.close()
        shm_slam_target.unlink()
        shm_slam_trigger.close()
        shm_slam_trigger.unlink()
        print("MISSION 1 processes terminated safely.")