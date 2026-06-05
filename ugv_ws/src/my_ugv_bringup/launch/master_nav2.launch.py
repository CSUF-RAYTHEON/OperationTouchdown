"""Nav2 autonomy stack (no Akida perception).

Full driving + SLAM + Nav2 navigation. Equivalent to running
master.launch2.py with enable_nav2:=true, provided as a dedicated entry point.
"""
from my_ugv_bringup.master_builder import build_master


def generate_launch_description():
    return build_master(akida=False, nav2=True)
