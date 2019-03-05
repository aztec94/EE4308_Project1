# EE4308_Project1

Instructions for use

1. Navigate to top of workspace and run "catkin_make".
2. Open a terminal window, run "./start.sh". 
3. Open a new terminal window, run "source ./devel/setup.bash" and then "roslaunch navbot main".

File Description

start.sh: There are 3 test worlds provided in the world/ directory. To switch worlds, uncomment and comment the relevant code sections in start.sh.

src/navbot/src/map.cpp: Map node responsible for constructing map, running path planning algorithms and giving commands to Navigator node.

src/navbot/src/navigator.cpp: Navigator node responsible for actuating movements on bot and sending feedback on current pose to Map node.

src/navbot/src/main.cpp: Main function integrates and runs Map and Navigator nodes.
