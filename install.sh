sudo mkdir -p $ROS_PACKAGE_PATH/turtlebot_gazebo/launch
sudo cp launch/gmapping_demo_custom.launch $ROS_PACKAGE_PATH/turtlebot_gazebo/launch

sudo mkdir -p /usr/share/gazebo/models
sudo cp models/* /usr/share/gazebo/models
