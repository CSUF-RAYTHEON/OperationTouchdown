"""Controller stack + Akida AKD1000 perception (no Nav2).

Drive with the Xbox controller (RB deadman + left stick) and run BrainChip
object detection. Detections appear in Foxglove as:
  /akida/image_annotated  (boxes + labels drawn on the frame)
  /akida/detected_objects  (text summary, e.g. "3x traffic_cones, 1x Box")
"""
from my_ugv_bringup.master_builder import build_master


def generate_launch_description():
    return build_master(akida=True, nav2=False)
