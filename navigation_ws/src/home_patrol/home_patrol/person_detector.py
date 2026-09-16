import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO


class PersonDetector(Node):

    def __init__(self):
        super().__init__('person_detector')

        # YOLO pretrained model
        self.model = YOLO('yolo11n.pt')

        # TurtleBot3 camera
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.image_callback,
            10
        )

        self.get_logger().info('Person detector started')

    def image_callback(self, msg):

        # ROS CompressedImage -> OpenCV image
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            return

        # YOLO inference
        results = self.model(
            frame,
            classes=[0],      # COCO class 0 = person
            verbose=False
        )

        person_detected = False

        for result in results:
            if len(result.boxes) > 0:
                person_detected = True

            # Bounding boxes 표시
            annotated_frame = result.plot()

        if person_detected:
            self.get_logger().info('Person detected')

        cv2.imshow('Home Patrol - Person Detection', annotated_frame)
        cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)

    node = PersonDetector()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        cv2.destroyAllWindows()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
