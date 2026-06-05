"""Nav2 autonomy stack + Akida AKD1000 perception.

Full driving + SLAM + Nav2 navigation, plus BrainChip object detection with
the Foxglove text summary on /akida/detected_objects.
"""
from my_ugv_bringup.master_builder import build_master


def generate_launch_description():
    return build_master(akida=True, nav2=True)
