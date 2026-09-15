import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from action_msgs.msg import GoalStatus


class WaypointPatrol(Node):

    def __init__(self):
        super().__init__('waypoint_patrol')

        self._client = ActionClient(
            self,
            FollowWaypoints,
            '/follow_waypoints'
        )

        # DN-002에서 검증한 Patrol Waypoints
        # x, y, orientation.z, orientation.w
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
            (-6.4422, -3.0367,  0.7135, 0.7007),  # Return HOME
        ]

    def create_pose(self, x, y, z, w):

        pose = PoseStamped()

        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w

        return pose

    def patrol_once(self):

        self.get_logger().info(
            'Waiting for Nav2 FollowWaypoints server...'
        )

        self._client.wait_for_server()

        poses = []

        for x, y, z, w in self.waypoints:
            poses.append(
                self.create_pose(x, y, z, w)
            )

        goal = FollowWaypoints.Goal()
        goal.poses = poses

        self.get_logger().info(
            f'Starting patrol through {len(poses)} waypoints'
        )

        send_future = self._client.send_goal_async(goal)

        rclpy.spin_until_future_complete(
            self,
            send_future
        )

        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error(
                'Waypoint patrol goal rejected'
            )
            return False

        self.get_logger().info(
            'Waypoint patrol goal accepted'
        )

        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(
            self,
            result_future
        )

        result = result_future.result()

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                'Patrol cycle completed'
            )
            return True

        self.get_logger().error(
            f'Patrol failed with status: {result.status}'
        )

        return False


def main(args=None):

    rclpy.init(args=args)

    node = WaypointPatrol()

    try:

        cycle = 1

        while rclpy.ok():

            node.get_logger().info(
                f'===== Patrol Cycle {cycle} ====='
            )

            success = node.patrol_once()

            if not success:
                node.get_logger().error(
                    'Stopping patrol because navigation failed'
                )
                break

            cycle += 1

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
