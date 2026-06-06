import os

def set_core_and_priority(core_index: int, nice_value: int = None):
    """Pins the current process to a specific CPU core and sets its priority."""
    pid = os.getpid()
    try:
        # 1. Pin to specific CPU Core
        os.sched_setaffinity(0, {core_index})
        
        # 2. Set Niceness (Requires sudo for negative values)
        if nice_value is not None:
            os.setpriority(os.PRIO_PROCESS, 0, nice_value)
            
        status = f"nice={nice_value}" if nice_value is not None else "default nice"
        print(f"[{pid}] SUCCESS: Pinned to CPU {core_index} with {status}")
        
    except PermissionError:
        print(f"[{pid}] WARNING: Cannot set nice={nice_value} without sudo! Run with 'sudo python3'.")
    except Exception as e:
        print(f"[{pid}] WARNING: Process optimization failed: {e}")