#!/bin/bash

# Specifies the shell interpreter to execute the script (in this case, Bash).

set -e
# Configures the script to immediately exit if any command fails.

source /opt/ros/humble/setup.bash
# Sets up the ROS 2 Humble environment by loading essential environment variables and tools.

# Checks if the script is running as the root user (user ID 0).
if [ "$(id -u)" = "0" ]; then
    echo "Running as root, adjusting permissions..."
    # If running as root, attempts to change ownership of the /home/ros2_ws directory to the 'ros2_ws' user and group.
    # Uses the '-R' option to apply changes recursively to all files and subdirectories.
    chown -R ros2_ws:ros2_ws /home/ros2_ws || echo "Failed to change ownership, continuing..."
    # If the 'chown' command fails, displays an error message but continues execution.
else
    echo "Not running as root, skipping ownership adjustment."
    # If not running as root, skips the ownership change and prints a message.
fi

exec "$@"
# Replaces the current script process with the command passed as arguments to this script.
# Useful for handing control over to another program or command specified at runtime.
