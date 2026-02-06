#!/bin/bash

echo -e "\033[33mExecute in this file in /VulcanAI/\033[0m"

pip install lark        # 'em' module for templating
pip install empy       # parsing ROS package.xml files
pip install catkin-pkg  # IDL parsing
colcon build --packages-select eprosima_bot_interfaces
source install/setup.bash

echo -e "\033[32mROS2 Interface EProsimaBotConfig:\033[0m"
ros2 interface show eprosima_bot_interfaces/msg/EProsimaBotConfig

