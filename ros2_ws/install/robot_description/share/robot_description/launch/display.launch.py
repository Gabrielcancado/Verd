import os
import launch
from launch.substitutions import LaunchConfiguration
import launch_ros.actions

def generate_launch_description():
    # Find the package path for 'robot_description'
    pkgPath = launch_ros.substitutions.FindPackageShare(package='robot_description').find('robot_description')

    # Construct the path to the URDF (or xacro) file
    urdfModelPath = os.path.join(pkgPath, 'urdf', 'robot_description.xacro')  # Ensure the file extension is correct

    # Construct the path to the RViz2 configuration file
    rvizConfigPath = os.path.join(pkgPath, 'rviz2', 'display.rviz')

    # Print the URDF model path for debugging purposes
    print(urdfModelPath)

    # Read the URDF model content
    with open(urdfModelPath, 'r') as infp:
        robot_desc = infp.read()

    # Prepare parameters for the robot description
    params = {'robot_description': robot_desc}

    # Define the robot_state_publisher node
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',        # The package that contains the node
        executable='robot_state_publisher',     # The executable to run
        output='screen',                        # Output logs to the terminal
        parameters=[params]                     # Pass the robot description as parameters
    )

    # Define the joint_state_publisher node (publishes joint states)
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',        # The package that contains the node
        executable='joint_state_publisher',     # The executable to run
        name='joint_state_publisher',           # Name of the node
        parameters=[params]                     # Pass the robot description as parameters
    )

    # Define the joint_state_publisher_gui node (optional GUI for joint control)
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',    # The package that contains the GUI
        executable='joint_state_publisher_gui', # The executable to run
        name='joint_state_publisher_gui',       # Name of the node
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui')) # Only launch if 'gui' is True
    )

    # Define the RViz2 node (visualization tool)
    rviz_node = launch_ros.actions.Node(
        package='rviz2',                        # The package that contains RViz2
        executable='rviz2',                     # The executable to run
        name='rviz2',                           # Name of the node
        output='screen',                        # Output logs to the terminal
        arguments=['-d', rvizConfigPath]        # Use the specified RViz2 configuration file
    )

    # Return the launch description including all the defined nodes
    return launch.LaunchDescription([
        # Declare a launch argument to enable or disable the joint_state_publisher_gui
        launch.actions.DeclareLaunchArgument(
            name='gui', default_value='True',   # Default value is True
            description='Flag to enable joint_state_publisher_gui'
        ),
        robot_state_publisher_node,            # Add the robot_state_publisher node
        joint_state_publisher_node,            # Add the joint_state_publisher node
        joint_state_publisher_gui_node,        # Add the joint_state_publisher_gui node (conditional)
        rviz_node                              # Add the RViz2 node
    ])
