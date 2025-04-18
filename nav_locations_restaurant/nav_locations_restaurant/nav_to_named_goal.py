import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler

named_locations = {
    "Table 1": [-1.31, -2.78, 0.0],
    "Kitchen": [-8.23, -5.81, 0.0],
}

class NavToNamedGoal(Node):
    def __init__(self):
        super().__init__('nav_to_named_goal')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, location_name):
        if location_name not in named_locations:
            self.get_logger().error(f"Unknown location: {location_name}")
            return

        x, y, yaw = named_locations[location_name]
        q = quaternion_from_euler(0, 0, yaw)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Navigation result: {result}")
        rclpy.shutdown()

def main():
    rclpy.init()
    navigator = NavToNamedGoal()
    # Replace with your destination name:
    navigator.send_goal('table_1')
    rclpy.spin(navigator)

if __name__ == '__main__':
    main()
