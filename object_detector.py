import json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO


class ObjectDetector(Node):
    def __init__(self):
        super().__init__("object_detector")

        self.declare_parameter("image_topic", "/oak/rgb/image_raw")
        self.declare_parameter("model_path", "best.pt")
        self.declare_parameter("conf_threshold", 0.4)

        self.image_topic = self.get_parameter("image_topic").value
        self.model_path = self.get_parameter("model_path").value
        self.conf_threshold = self.get_parameter("conf_threshold").value

        self.bridge = CvBridge()
        self.model = YOLO(self.model_path)

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        self.annotated_pub = self.create_publisher(
            Image,
            "/detections/image_annotated",
            10
        )

        self.json_pub = self.create_publisher(
            String,
            "/detections/json",
            10
        )

        self.get_logger().info(f"Listening on: {self.image_topic}")
        self.get_logger().info(f"Using model: {self.model_path}")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"CV bridge error: {e}")
            return

        results = self.model.predict(
            source=frame,
            conf=self.conf_threshold,
            verbose=False
        )

        if not results:
            return

        result = results[0]
        annotated = result.plot()

        detections = []
        names = result.names

        if result.boxes is not None:
            for box in result.boxes:
                cls_id = int(box.cls[0].item())
                conf = float(box.conf[0].item())
                x1, y1, x2, y2 = box.xyxy[0].tolist()

                detections.append({
                    "class_id": cls_id,
                    "label": names.get(cls_id, str(cls_id)),
                    "confidence": conf,
                    "bbox_xyxy": [x1, y1, x2, y2]
                })

        out_msg = String()
        out_msg.data = json.dumps(detections)
        self.json_pub.publish(out_msg)

        try:
            img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            img_msg.header = msg.header
            self.annotated_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f"Image publish error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()