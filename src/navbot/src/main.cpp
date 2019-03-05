#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <queue> 
#include <deque> 
#include "navigator.h"
#include "map.h"

int main(int argc, char** argv)
{
  
  std::deque<int> command;
  std::deque<int> feedback;

  ros::init(argc, argv, "map");
  ros::init(argc, argv, "navigator");

  ros::NodeHandle mp;
  ros::NodeHandle nv;

  //command.push(10);
  //feedback.push(10);

  Map m(mp, &command, &feedback);
  BotController bc(nv, &command, &feedback);

  ros::spin();
  
  return 0;
}
