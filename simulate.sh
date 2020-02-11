export TURTLEBOT_GAZEBO_WORLD_FILE=/workspaces/catkin_ws/src/contest1/worlds/test.world
export GAZEBO_MODEL_PATH=/workspaces/catkin_ws/src/contest1/worlds:$GAZEBO_MODEL_PATH

roslaunch turtlebot_gazebo turtlebot_world.launch gui:=false
