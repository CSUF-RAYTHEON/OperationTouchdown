import cv2
import numpy as np
import time
from multiprocessing import shared_memory

def calculatevioslam_updateposition(frame_shared_memory_mutex, uart_tx_port_mutex):
    W, H = 640, 400
    time.sleep(2)

    # We only connect to the two blocks we actually care about drawing
    shm_rgb = shared_memory.SharedMemory(name="oak_rgb")
    shm_gray = shared_memory.SharedMemory(name="oak_gray")
    shm_depth = shared_memory.SharedMemory(name="oak_depth")

    shared_rgb = np.ndarray((H, W, 3), dtype=np.uint8, buffer=shm_rgb.buf)
    shared_gray = np.ndarray((H, W), dtype=np.uint8, buffer=shm_gray.buf)
    shared_depth = np.ndarray((H, W), dtype=np.uint16, buffer=shm_depth.buf)

    local_rgb = np.zeros((H, W, 3), dtype=np.uint8)
    local_gray = np.zeros((H, W), dtype=np.uint8)
    local_depth = np.zeros((H, W), dtype=np.uint16)

    print("VIO calculator and position updater connected to shared Memory. Starting calculations...")

    while True:
        # This section is a critical section as we must aquire the mutex/lock before 
        # copying from the shared memory, and release it immediately after.
        with frame_shared_memory_mutex:
            np.copyto(local_rgb, shared_rgb)
            np.copyto(local_gray, shared_gray)
            np.copyto(local_depth, shared_depth)