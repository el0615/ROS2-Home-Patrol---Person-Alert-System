import os
import shutil
import subprocess

import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from action_msgs.msg import GoalStatus
from std_msgs.msg import Bool


class WaypointPatrol(Node):

    def __init__(self):
        super().__init__('waypoint_patrol')

        # Nav2 FollowWaypoints Action Client
        self._client = ActionClient(
            self,
            FollowWaypoints,
            '/follow_waypoints'
        )

        # YOLO 사람 탐지 결과
        self.person_subscription = self.create_subscription(
            Bool,
            '/person_detected',
            self.person_callback,
            10
        )

        self.person_detected = False
        self.cancel_due_to_person = False

        self.goal_handle = None

        # 현재 이동 중인 waypoint
        self.current_waypoint = 0
        self.active_start_index = 0

        # WP1 ~ WP9 + HOME
        self.waypoints = [
            (-5.9730,  3.7337,  0.0097, 1.0000),  # WP1
            (-1.1814,  3.6805, -0.7089, 0.7053),  # WP2
            ( 6.4592,  0.7496,  0.7053, 0.7089),  # WP3
            ( 6.4476,  4.2559,  0.9999, 0.0131),  # WP4
            ( 0.6034,  4.1683, -0.7089, 0.7053),  # WP5
            ( 1.4159,  3.0539,  0.7053, 0.7089),  # WP6
            ( 2.0374,  4.6286, -0.0025, 1.0000),  # WP7
            ( 3.4496,  0.7355, -0.0025, 1.0000),  # WP8
            ( 5.5047, -3.1609, -0.7089, 0.7053),  # WP9

            (-6.4422, -3.0367,  0.7135, 0.7007),  # HOME
        ]

        self.waypoint_names = [
            'WP1',
            'WP2',
            'WP3',
            'WP4',
            'WP5',
            'WP6',
            'WP7',
            'WP8',
            'WP9',
            'HOME',
        ]

    def create_pose(self, x, y, z, w):

        pose = PoseStamped()

        pose.header.frame_id = 'map'
        pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w

        return pose

    def sound_alarm(self):

        # Ubuntu 기본 경고음 사용 가능하면 재생
        sound_file = (
            '/usr/share/sounds/freedesktop/'
            'stereo/dialog-warning.oga'
        )

        if (
            shutil.which('paplay')
            and os.path.exists(sound_file)
        ):
            subprocess.Popen(
                ['paplay', sound_file],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )

        else:
            # 시스템 사운드가 없으면 터미널 Bell
            print('\a', end='', flush=True)

    def person_callback(self, msg):

        # 사람 처음 발견
        if msg.data and not self.person_detected:

            self.person_detected = True
            self.cancel_due_to_person = True

            self.get_logger().warning(
                '================================'
            )
            self.get_logger().warning(
                'PERSON DETECTED - PATROL STOPPED'
            )
            self.get_logger().warning(
                '================================'
            )

            self.sound_alarm()

            # 현재 Nav2 waypoint goal 취소
            if self.goal_handle is not None:

                self.get_logger().warning(
                    'Cancelling current navigation goal...'
                )

                self.goal_handle.cancel_goal_async()

        # 사람이 사라짐
        elif (
            not msg.data
            and self.person_detected
        ):
            self.person_detected = False

            self.get_logger().info(
                'Person no longer detected'
            )

    def feedback_callback(self, feedback_msg):

        feedback = feedback_msg.feedback

        # FollowWaypoints에서 현재 목표 waypoint 번호
        relative_index = feedback.current_waypoint

        self.current_waypoint = (
            self.active_start_index
            + relative_index
        )

    def wait_until_person_clear(self):

        self.get_logger().warning(
            'Waiting until area is clear...'
        )

        while (
            rclpy.ok()
            and self.person_detected
        ):
            rclpy.spin_once(
                self,
                timeout_sec=0.1
            )

        self.get_logger().info(
            'Area clear - patrol will resume'
        )

    def patrol_from(self, start_index):

        # 시작하기 전부터 사람이 보이면 대기
        if self.person_detected:
            self.wait_until_person_clear()

        self._client.wait_for_server()

        self.active_start_index = start_index
        self.current_waypoint = start_index

        poses = []

        for x, y, z, w in self.waypoints[start_index:]:

            poses.append(
                self.create_pose(
                    x,
                    y,
                    z,
                    w
                )
            )

        self.get_logger().info(
            'Starting patrol from '
            f'{self.waypoint_names[start_index]}'
        )

        goal = FollowWaypoints.Goal()
        goal.poses = poses

        send_future = self._client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )

        while (
            rclpy.ok()
            and not send_future.done()
        ):
            rclpy.spin_once(
                self,
                timeout_sec=0.1
            )

        goal_handle = send_future.result()

        if not goal_handle.accepted:

            self.get_logger().error(
                'Waypoint patrol goal rejected'
            )

            return 'failed', start_index

        self.goal_handle = goal_handle

        self.get_logger().info(
            'Waypoint patrol goal accepted'
        )

        # goal이 받아지는 순간 이미 사람이 감지된 경우
        if self.person_detected:

            self.cancel_due_to_person = True

            self.goal_handle.cancel_goal_async()

        result_future = (
            self.goal_handle.get_result_async()
        )

        while (
            rclpy.ok()
            and not result_future.done()
        ):
            rclpy.spin_once(
                self,
                timeout_sec=0.1
            )

        result = result_future.result()

        self.goal_handle = None

        # 정상적으로 HOME까지 완료
        if (
            result.status
            == GoalStatus.STATUS_SUCCEEDED
        ):

            self.get_logger().info(
                'Patrol cycle completed'
            )

            return 'completed', 0

        # 사람 때문에 중지된 경우
        if (
            result.status
            == GoalStatus.STATUS_CANCELED
            and self.cancel_due_to_person
        ):

            resume_index = min(
                self.current_waypoint,
                len(self.waypoints) - 1
            )

            self.get_logger().warning(
                'Patrol paused at '
                f'{self.waypoint_names[resume_index]}'
            )

            self.wait_until_person_clear()

            self.cancel_due_to_person = False

            return 'resume', resume_index

        self.get_logger().error(
            'Patrol failed with status: '
            f'{result.status}'
        )

        return 'failed', start_index


def main(args=None):

    rclpy.init(args=args)

    node = WaypointPatrol()

    cycle = 1
    start_index = 0

    try:

        while rclpy.ok():

            if start_index == 0:

                node.get_logger().info(
                    f'===== Patrol Cycle {cycle} ====='
                )

            status, next_index = (
                node.patrol_from(start_index)
            )

            if status == 'completed':

                cycle += 1
                start_index = 0

            elif status == 'resume':

                start_index = next_index

                node.get_logger().info(
                    'Resuming patrol from '
                    f'{node.waypoint_names[start_index]}'
                )

            else:

                node.get_logger().error(
                    'Stopping patrol'
                )

                break

    except KeyboardInterrupt:

        node.get_logger().info(
            'Patrol stopped by user'
        )

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()