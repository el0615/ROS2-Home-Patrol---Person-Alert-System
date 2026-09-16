import time

import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from ultralytics import YOLO


class PersonDetector(Node):

    def __init__(self):
        super().__init__('person_detector')

        # YOLO pretrained model
        self.model = YOLO('yolo11n.pt')

        # 사람 탐지 상태 발행
        self.person_publisher = self.create_publisher(
            Bool,
            '/person_detected',
            10
        )

        # TurtleBot3 camera
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.image_callback,
            10
        )

        # 사람 상태
        self.person_state = False
        self.last_person_time = 0.0

        # 사람이 사라진 뒤 이 시간 동안 기다렸다가 False 처리
        self.clear_delay = 3.0

        self.get_logger().info(
            'Person detector started'
        )

    def image_callback(self, msg):

        # CompressedImage -> OpenCV
        np_arr = np.frombuffer(
            msg.data,
            np.uint8
        )

        frame = cv2.imdecode(
            np_arr,
            cv2.IMREAD_COLOR
        )

        if frame is None:
            return

        # YOLO - person class만 탐지
        results = self.model(
            frame,
            classes=[0],
            conf=0.5,
            verbose=False
        )

        person_now = False

        for result in results:
            if len(result.boxes) > 0:
                person_now = True

        # Bounding Box 표시
        annotated_frame = results[0].plot()

        current_time = time.monotonic()

        # 사람 탐지
        if person_now:
            self.last_person_time = current_time

            if not self.person_state:
                self.person_state = True

                self.get_logger().warning(
                    'PERSON DETECTED'
                )

        # 사람을 마지막으로 본 뒤 3초 이상 경과
        else:
            if (
                self.person_state
                and current_time - self.last_person_time
                >= self.clear_delay
            ):
                self.person_state = False

                self.get_logger().info(
                    'Person cleared'
                )

        # 현재 상태를 계속 발행
        state_msg = Bool()
        state_msg.data = self.person_state

        self.person_publisher.publish(
            state_msg
        )

        # 화면 표시
        if self.person_state:
            cv2.putText(
                annotated_frame,
                'WARNING: PERSON DETECTED',
                (20, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 0, 255),
                2
            )

        cv2.imshow(
            'Home Patrol - Person Detection',
            annotated_frame
        )

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