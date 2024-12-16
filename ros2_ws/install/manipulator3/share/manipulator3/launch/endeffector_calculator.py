import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

class EndEffectorCalculator(Node):
    def __init__(self):
        super().__init__('endeffector_calculator')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        joint_positions = msg.position
        endeffector_position = self.calculate_endeffector_position(joint_positions)
        self.get_logger().info(f'End-Effector Position: {endeffector_position}')

    def calculate_endeffector_position(self, joint_positions):
        # Exemplo: Manipulador planar de 3 DOFs
        l1, l2, l3 = 1.0, 1.0, 1.0  # Comprimentos dos elos
        theta1, theta2, theta3 = joint_positions

        x = l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2) + l3 * np.cos(theta1 + theta2 + theta3)
        y = l1 * np.sin(theta1) + l2 * np.sin(theta1 + theta2) + l3 * np.sin(theta1 + theta2 + theta3)
        return (x, y)

def main(args=None):
    rclpy.init(args=args)
    node = EndEffectorCalculator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
