import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class Robot:
    def __init__(self, name: str, node: Node) -> None:
        self.node = node
        self.odom_sub = self.node.create_subscription(
            Twist, f"{name}/odom", self.odom_sub_callback, 10
        )
        self.name = name
        self.publisher = self.node.create_publisher(Twist, f"{self.name}/cmd_vel", 10)
        self._logger = self.node.get_logger()

    def publish_cmd_vel(self, msg: Twist):
        self.publisher.publish(msg)

    def odom_sub_callback(self, msg: Odometry):
        self._logger.info(
            f"{self.name}: pose x:{msg.pose.pose.position.x}, y:{msg.pose.pose.position.y}"
        )


class GazeboCmdVel(Node):
    def __init__(self):
        super().__init__("gazebo_cmd_vel")

        self.subscriber = self.create_subscription(
            Twist, "/cmd_vel", self.subscriber_callback, 10
        )

        self.robots = []
        self.number_of_robots = 4

        for i in range(self.number_of_robots):
            robot_name = f"r{i}"
            self.robots.append(Robot(robot_name, self))

    def subscriber_callback(self, msg: Twist):
        for robot in self.robots:
            robot.publish_cmd_vel(msg)


def main(args=None):
    rclpy.init(args=args)
    gazebo_cmd_vel = GazeboCmdVel()
    rclpy.spin(gazebo_cmd_vel)
    gazebo_cmd_vel.destroy_node()
    rclpy.shutdown()
