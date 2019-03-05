sudo cp ./world/test_world_1.world /opt/ros/kinetic/share/turtlebot_gazebo/worlds/
export TURTLEBOT_GAZEBO_WORLD_FILE="/opt/ros/kinetic/share/turtlebot_gazebo/worlds/test_world_1.world"
export ROBOT_INITIAL_POSE="-x 0.5 -y 0.5 -Y 1.57"

#sudo cp ./world/test_world_2.world /opt/ros/kinetic/share/turtlebot_gazebo/worlds/
#export TURTLEBOT_GAZEBO_WORLD_FILE="/opt/ros/kinetic/share/turtlebot_gazebo/worlds/test_world_2.world"
#export ROBOT_INITIAL_POSE="-x 0.5 -y 0.5 -Y 1.57"

#sudo cp ./world/world_2018.world /opt/ros/kinetic/share/turtlebot_gazebo/worlds/
#export TURTLEBOT_GAZEBO_WORLD_FILE="/opt/ros/kinetic/share/turtlebot_gazebo/worlds/world_2018.world"
#export ROBOT_INITIAL_POSE="-x 1.5 -y 4 -Y -1.57"

source ./devel/setup.sh

roslaunch navbot main.launch 
