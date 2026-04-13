import cv2
import numpy as np
import time
from multiprocessing import shared_memory

def run_viewer(lock):
    W, H = 640, 400
    time.sleep(2)

    # 1. Connect to the shared memory blocks for RGB and gray frames as we don't care about depth for simply viewing the camera feed
    shm_rgb = shared_memory.SharedMemory(name="oak_rgb")
    shm_gray = shared_memory.SharedMemory(name="oak_gray")

    shared_rgb = np.ndarray((H, W, 3), dtype=np.uint8, buffer=shm_rgb.buf)
    shared_gray = np.ndarray((H, W), dtype=np.uint8, buffer=shm_gray.buf)

    local_rgb = np.zeros((H, W, 3), dtype=np.uint8)
    local_gray = np.zeros((H, W), dtype=np.uint8)
    print("Viewer connected to Shared Memory. Starting display...")

    while True:
        # 2. This section is a critical section as we must aquire the mutex/lock before 
        #    copying from the shared memory, and release it immediately after.
        with lock:
            np.copyto(local_rgb, shared_rgb)
            np.copyto(local_gray, shared_gray)

        # 3. Make the 1-channel gray into 3-channels so we can get them in the same image and display them together
        gray_bgr = cv2.cvtColor(local_gray, cv2.COLOR_GRAY2BGR)
        combined = np.hstack((local_rgb, gray_bgr))

        cv2.imshow("RGB and Gray", combined)
        if cv2.waitKey(60) & 0xFF == ord('q'):
            break
    
    # 4. Close the shared memory connections and destroy the viewer window
    print("Viewer exiting...")
    cv2.destroyAllWindows()
    shm_rgb.close()
    shm_gray.close()