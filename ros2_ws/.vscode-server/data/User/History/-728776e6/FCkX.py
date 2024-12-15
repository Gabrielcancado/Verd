from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Diretório do modelo URDF/Xacro
    urdf_file = os.path.join(
        get_package_share_directory('manipulator3'),
        'urdf',
        'manipulator3.urdf.xacro'
    )

    # Converte o modelo Xacro para URDF
    with open(urdf_file, 'r') as file:
        robot_description_content = file.read()

    # Nó para o Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Publica o modelo no parâmetro 'robot_description'
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # Publica o estado dos joints
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        joint_state_publisher_gui,
    ])
