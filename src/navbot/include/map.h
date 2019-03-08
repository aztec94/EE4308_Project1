#ifndef __MAP_H_INCLUDED__
#define __MAP_H_INCLUDED__

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/video.hpp>

#include <iostream>
#include <string>
#include <math.h>
#include <tr1/tuple> // if gcc > 4.7 need to use #include <tuple> // std::tuple, std::make_tuple, std::tie

#include <deque> 

#define	MAP_SIZE 9
#define DEPTH_MIN 0.8
#define GOALCELL_X 4
#define GOALCELL_Y 4

enum cellStatus {EMPTY, UNEXPLORED};
enum cellSides {OPEN, CLOSE, UNDEFINED};
enum Position {N, S, E, W}; //0, 1 , 2 , 3
struct cell {
	cellSides north, south, east, west;
	int edges_explored;
	cellStatus status;
};

namespace enc = sensor_msgs::image_encodings;

class Map {
  private:
      ros::NodeHandle nh_;
      ros::Subscriber depth_sub;
      std::deque<int>* cmd_queue;
      std::deque<int>* fb_queue;
      cell map[MAP_SIZE][MAP_SIZE];
      Position curPos; // Current bot direction
      bool hasReachedCell;
      bool hasUpdatedCell;
      bool hasReachedGoal;
      bool isPathSet;
      int curX, curY; // Current cell coordinates
      int targetcellX, targetcellY; // Nearest unexplored cell coordinates
      int R[4] = {0, -1, 0, 1}; //y
      int C[4] = {-1, 0, 1, 0}; //x
  public:
    Map(ros::NodeHandle &nh, std::deque<int>* command, std::deque<int>* feedback);
    void mapcallback(const sensor_msgs::ImageConstPtr& depthmsg);
    void init();
    // Update bot current position
    void updateCurPos();
    // Update cell information
    void updateCell(float depth);
    // Movement 
    void insertCommandTurn(int command);
    void insertCommandMoveForward(int command);
    // Check cell openings and validity
    bool isValid(int i, int j);
    bool isOpenNorth(int i, int j);
    bool isOpenSouth(int i, int j);
    bool isOpenEast(int i, int j);
    bool isOpenWest(int i, int j);
    // Get nearest unexplored cell
    void getNearestUnexploredCell(int x, int y);
    // Get shortest path
    bool getShortestPath(int src, int dest, int pred[], int dist[]);
    // Give commands for bot to follow shortest path
    void setPath(int src, int dest);
    // Check if bot has reached nearest unexplored cell
    void hasReachedTargetCell();
    // Check if bot has reached goal
    void hasReachedGoalCell();
    // Unit test: Code driver
    void testdeque();
};


#endif
