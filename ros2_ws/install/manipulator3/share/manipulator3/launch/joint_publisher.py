import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class JointPublisher(Node):
    def __init__(self):
        super().__init__('joint_publisher')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)
        self.joint_names = ['joint1', 'joint2', 'joint3']
        self.angle = 0.0

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [math.sin(self.angle), math.cos(self.angle), math.sin(self.angle / 2)]
        self.publisher.publish(msg)
        self.angle += 0.1  # Incremento do ângulo para simular movimento

def main(args=None):
    rclpy.init(args=args)
    node = JointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
