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

#include "map.h"

namespace enc = sensor_msgs::image_encodings;

Map::Map(ros::NodeHandle &nh, std::deque<int>* command, std::deque<int>* feedback) {
  depth_sub = nh.subscribe("/camera/depth/image_raw",1, &Map::mapcallback, this);
  cmd_queue = command;
  fb_queue = feedback;
  init();
  //testdeque(); // Unit test code
}

void Map::mapcallback(const sensor_msgs::ImageConstPtr& depthmsg) {

  cv_bridge::CvImageConstPtr cv_ptr;
  float depth_val;
  int src, dest;

  try{
      cv_ptr = cv_bridge::toCvShare(depthmsg, enc::TYPE_32FC1);
  }catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  depth_val = cv_ptr->image.at<float>(320,240);

  updateCurPos();

  if (!hasReachedGoal) {  

	  if(hasReachedCell && !hasUpdatedCell) {
	    updateCell(depth_val);
	    isPathSet = false;
	  }
	  
	  if(hasUpdatedCell && !isPathSet) {
	    getNearestUnexploredCell(curX, curY);
	    src = curX + curY*MAP_SIZE;
	    dest = targetcellX + targetcellY*MAP_SIZE;
	    std::cout << "current X: " << curX << " current Y: " << curY << "\n";
	    std::cout << "target X: " << targetcellX << " target Y: " << targetcellY << "\n";    
	    setPath(src, dest);
	    isPathSet = true;
	  }

	  if(isPathSet) {
	    hasReachedTargetCell();
	  }
	  
	  hasReachedGoalCell();

  }
}

// To check the validity of commands inserted
void Map::insertCommandTurn(int command) {
  int last_cmd=0;
  if(!cmd_queue->empty()) {
    
    // Get last command (apart from forward / down)
    for(int i=(cmd_queue->size() - 1); i >= 0; i--) {
      if ((cmd_queue->at(i)) < 5) {
        last_cmd = cmd_queue->at(i);
        break;
      }
    }

    // All the previous commands are move forward / down
    if (last_cmd == 0) {
      cmd_queue->push_back(command);
      std::cout << "Inserted command " << command << "\n";
      return;
    }
    
    if (last_cmd == 4) {
      if (command != 1) {
        return; //not a valid command
      }
      else {
        cmd_queue->push_back(command);
        std::cout << "Inserted command " << command << "\n";
      }
    } 
    else {
      if ((last_cmd + 1) != command) {
        return;
      }
      else {
        cmd_queue->push_back(command);
        std::cout << "Inserted command " << command << "\n";
      }
    }
  }
  // If command list is empty
  else {
    cmd_queue->push_back(command);
    std::cout << "Inserted command " << command << "\n";
  }
}

void Map::insertCommandMoveForward(int command) {
  cmd_queue->push_back(command);
  std::cout << "Inserted command " << command << "\n";
}

void Map::getNearestUnexploredCell(int x, int y) {
    bool visited[MAP_SIZE*MAP_SIZE];
    int adjx, adjy;
    int index, currIndex, visY, visX;
    std::deque<int> q;

    for(int i = 0; i < MAP_SIZE*MAP_SIZE; i++) {
        visited[i] = false;
    }

    index = x + y*MAP_SIZE;
    visited[index] = true;
    q.push_back(index);

    while(!q.empty()) {
        currIndex = q.front();
        visX = currIndex % MAP_SIZE;
        visY = currIndex/MAP_SIZE;
        std::cout << "Visited X: " << visX << " ";
        std::cout << "Visited Y: " << visY << "\n";

        // Return if the first unexplored cell has been found
        if (map[visY][visX].status == UNEXPLORED) {
          targetcellX = visX;
          targetcellY = visY;
          return;
        }

        q.pop_front();

        adjx = visX + C[3];
        adjy = visY + R[3]; 
        if(isValid(adjx, adjy) && isOpenEast(visX, visY)) {
          index = adjx + adjy*MAP_SIZE;
          if(!visited[index]) {
            visited[index] = true;
            q.push_back(index);
          }
        }

        adjx = visX + C[2];
        adjy = visY + R[2]; 
        if(isValid(adjx, adjy) && isOpenNorth(visX, visY)) {
          index = adjx + adjy*MAP_SIZE;
          if(!visited[index]) {
            visited[index] = true;
            q.push_back(index);
          }
        }

        adjx = visX + C[0];
        adjy = visY + R[0]; 
        if(isValid(adjx, adjy) && isOpenSouth(visX, visY)) {
          index = adjx + adjy*MAP_SIZE;
          if(!visited[index]) {
            visited[index] = true;
            q.push_back(index);
          }
        }

        adjx = visX + C[1];
        adjy = visY + R[1]; 
        if(isValid(adjx, adjy) && isOpenWest(visX, visY)) {
          index = adjx + adjy*MAP_SIZE;
          if(!visited[index]) {
            visited[index] = true;
            q.push_back(index);
          }
        }
    }
}

bool Map::getShortestPath(int src, int dest, int pred[], int dist[]) { 
    int v = MAP_SIZE*MAP_SIZE;
    int adjx, adjy, visX, visY;
    std::list<int> queue; 
    bool visited[v]; 
    int index;

    for (int i = 0; i < v; i++) { 
        visited[i] = false; 
        dist[i] = INT_MAX; 
        pred[i] = -1; 
    } 

    visited[src] = true; 
    dist[src] = 0; 
    queue.push_back(src); 

    // standard BFS algorithm 
    while (!queue.empty()) { 
        int u = queue.front(); 
        visX = u % MAP_SIZE;
        visY = u/MAP_SIZE;          
        queue.pop_front(); 

        adjx = visX + C[3];
        adjy = visY + R[3]; 
        if(isValid(adjx, adjy) && isOpenEast(visX, visY)) {
            index = adjx + adjy*MAP_SIZE;
            if(!visited[index]) {
                visited[index] = true;
                dist[index] = dist[u] + 1;
                pred[index] = u;                  
                queue.push_back(index);
            }
        }

        if (index == dest) {
            return true; 
        }

        adjx = visX + C[2];
        adjy = visY + R[2]; 
        if(isValid(adjx, adjy) && isOpenNorth(visX, visY)) {
            index = adjx + adjy*MAP_SIZE;
            if(!visited[index]) {
                visited[index] = true;
                dist[index] = dist[u] + 1;
                pred[index] = u;
                queue.push_back(index);
            }
        }

        if (index == dest) {
            return true; 
        }

        adjx = visX + C[0];
        adjy = visY + R[0]; 
        if(isValid(adjx, adjy) && isOpenSouth(visX, visY)) {
            index = adjx + adjy*MAP_SIZE;
            if(!visited[index]) {
                visited[index] = true;
                dist[index] = dist[u] + 1;
                pred[index] = u;
                queue.push_back(index);
            }
        }

        if (index == dest) {
            return true; 
        }

        adjx = visX + C[1];
        adjy = visY + R[1]; 
        if(isValid(adjx, adjy) && isOpenWest(visX, visY)) {
            index = adjx + adjy*MAP_SIZE;
            if(!visited[index]) {
                visited[index] = true;
                dist[index] = dist[u] + 1;
                pred[index] = u;                    
                queue.push_back(index);
            }
        }

        if (index == dest) {
            return true; 
        }
    } 
    return false; 
} 

bool Map::isValid(int i, int j) {
  return (i >= 0 && i < MAP_SIZE) && (j >= 0 && j < MAP_SIZE);
}
bool Map::isOpenNorth(int i, int j) {
  if (map[j][i].north == OPEN) {
    return true;
  }
  return false;
}
bool Map::isOpenSouth(int i, int j) {
  if (map[j][i].south == OPEN) {
    return true;
  }
  return false;
}
bool Map::isOpenEast(int i, int j) {
  if (map[j][i].east == OPEN) {
    return true;
  }
  return false;
}
bool Map::isOpenWest(int i, int j) {
  if (map[j][i].west == OPEN) {
    return true;
  }
  return false;
}

void Map::updateCell(float depth) {
  //cell curCell = map[curY][curX];
  //std::cout << "Number of edges explored " << map[curY][curX].edges_explored << "\n"; // DEBUG
  std::cout << "current position " << curPos << "\n";
  if(map[curY][curX].edges_explored < 4) {

    if(map[curY][curX].north == UNDEFINED) {
      //std::cout << "North undefined\n";
      if(curPos == N) {
        if(depth < DEPTH_MIN) {
          map[curY][curX].north = CLOSE;
          map[curY][curX].edges_explored = map[curY][curX].edges_explored + 1;
          if (isValid(curX+1, curY)) {
            map[curY][curX+1].south = CLOSE;
            map[curY][curX+1].edges_explored = map[curY][curX+1].edges_explored + 1;
          }
        } else {
          map[curY][curX].north = OPEN;
          map[curY][curX].edges_explored = map[curY][curX].edges_explored + 1;
          if (isValid(curX+1, curY)) {
            map[curY][curX+1].south = OPEN;
            map[curY][curX+1].edges_explored = map[curY][curX+1].edges_explored + 1;
          }
        }
      }
      if(curPos == W) {
        insertCommandTurn(2);
        insertCommandTurn(3);
        insertCommandTurn(4);
      }
      if(curPos == S) {
        insertCommandTurn(3);
        insertCommandTurn(4);
      }
      if(curPos == E) {
        insertCommandTurn(4);
      }
    }
    if(map[curY][curX].south == UNDEFINED) {
      //std::cout << "South undefined\n";
      if(curPos == S) {
        if(depth < DEPTH_MIN) {
          map[curY][curX].south = CLOSE;
          map[curY][curX].edges_explored = map[curY][curX].edges_explored + 1;
          if (isValid(curX-1, curY)) {
            map[curY][curX-1].north = CLOSE;
            map[curY][curX-1].edges_explored = map[curY][curX-1].edges_explored + 1;
          }
        } else {
          map[curY][curX].south = OPEN;
          map[curY][curX].edges_explored = map[curY][curX].edges_explored + 1;
          if (isValid(curX-1, curY)) {
            map[curY][curX-1].north = OPEN;
            map[curY][curX-1].edges_explored = map[curY][curX-1].edges_explored + 1;
          }
        }
      }
      if(curPos == E) {
        insertCommandTurn(4);
        insertCommandTurn(1);
        insertCommandTurn(2);
      }
      if(curPos == N) {
        insertCommandTurn(1);
        insertCommandTurn(2);
      }
      if(curPos == W) {
        insertCommandTurn(2);
      }
    }
    if(map[curY][curX].east == UNDEFINED) {
      //std::cout << "East undefined\n";
      if(curPos == E) {
        if(depth < DEPTH_MIN) {
          map[curY][curX].east = CLOSE;
          map[curY][curX].edges_explored = map[curY][curX].edges_explored + 1;
          if (isValid(curX, curY+1)) {
            map[curY+1][curX].west = CLOSE;
            map[curY+1][curX].edges_explored = map[curY+1][curX].edges_explored + 1;
          }
        } else {
          map[curY][curX].east = OPEN;
          map[curY][curX].edges_explored = map[curY][curX].edges_explored + 1;
          if (isValid(curX, curY+1)) {
            map[curY+1][curX].west = OPEN;
            map[curY+1][curX].edges_explored = map[curY+1][curX].edges_explored + 1;
          }
        }
      }
      if(curPos == N) {
        insertCommandTurn(1);
        insertCommandTurn(2);
        insertCommandTurn(3);
      }
      if(curPos == W) {
        insertCommandTurn(2);
        insertCommandTurn(3);
      }
      if(curPos == S) {
        insertCommandTurn(3);
      }
    }
    if(map[curY][curX].west == UNDEFINED) {
      //std::cout << "West undefined\n";
      if(curPos == W) {
        if(depth < DEPTH_MIN) {
          map[curY][curX].west = CLOSE;
          map[curY][curX].edges_explored = map[curY][curX].edges_explored + 1;
          if (isValid(curX, curY-1)) {
            map[curY-1][curX].east = CLOSE;
            map[curY-1][curX].edges_explored = map[curY-1][curX].edges_explored + 1;
          }
        } else {
          map[curY][curX].west = OPEN;
          map[curY][curX].edges_explored = map[curY][curX].edges_explored + 1;
          if (isValid(curX, curY-1)) {
            map[curY-1][curX].east = OPEN;
            map[curY-1][curX].edges_explored = map[curY-1][curX].edges_explored + 1;
          }
        }
      }
      if(curPos == S) {
        insertCommandTurn(3);
        insertCommandTurn(4);
        insertCommandTurn(1);
      }
      if(curPos == E) {
        insertCommandTurn(4);
        insertCommandTurn(1);
      }
      if(curPos == N) {
        insertCommandTurn(1);
      }
    }    
  } else {
    map[curY][curX].status = EMPTY;
    hasUpdatedCell = true;    
  }
}

void Map::hasReachedTargetCell() {
  if ((curX == targetcellX) && (curY == targetcellY)) {
    hasReachedCell = true;
    hasUpdatedCell = false;
  }
}

void Map::hasReachedGoalCell() {
  if ((curX == GOALCELL_X) && (curY == GOALCELL_Y)) {
    hasReachedGoal = true;
    std::cout << "Goal reached\n";
  }
}

void Map::setPath(int src, int dest) { 
    int v = MAP_SIZE*MAP_SIZE;
    int pred[v], dist[v]; 
    int adjx, adjy;
  
    if (getShortestPath(src, dest, pred, dist) == false) { 
        std::cout << "Given source and destination" << " are not connected\n"; 
        return; 
    } 
  
    // vector path stores the shortest path 
    std::vector<int> path; 
    int crawl = dest; 
    path.push_back(crawl); 
    while (pred[crawl] != -1) { 
        path.push_back(pred[crawl]); 
        crawl = pred[crawl]; 
    } 
  
    // distance from source is in distance array 
    std::cout << "Shortest path length is : " << dist[dest] << "\n";
  
    // printing path from source to destination 
    std::cout << "Path is::\n"; 
    for (int i = path.size() - 1; i >= 0; i--) {
        std::cout << path[i] << " "; 
    }
    std::cout << "\n";

    for (int i = path.size() - 1; i >= 0; i--) {
        adjx = path[i] % MAP_SIZE;
        adjy = path[i] / MAP_SIZE;
        // Move in y direction
        if (adjx == curX) {
          if (adjy > curY) {
              switch(curPos) {
                case W:
                  insertCommandTurn(2);
                  insertCommandTurn(3);
                  break;   
                case S:
                  insertCommandTurn(3);
                  break;   
                case N:
                  insertCommandTurn(1);
                  insertCommandTurn(2);
                  insertCommandTurn(3);
                  break;        
              }
              insertCommandMoveForward(6);
          } 
          else {
            if (adjy < curY) {
                switch(curPos) {
                  case E:
                    insertCommandTurn(4);
                    insertCommandTurn(1);
                    break;   
                  case N:
                    insertCommandTurn(1);
                    break;   
                  case S:
                    insertCommandTurn(3);
                    insertCommandTurn(4);
                    insertCommandTurn(1);
                    break;        
                }
                insertCommandMoveForward(8);
            }
            else {
              std::cout << "adjy and curY are same\n";
            }
          }
        } else { // Move in x direction
            if (adjx > curX) {
              switch(curPos) {
                case W:
                  insertCommandTurn(2);
                  insertCommandTurn(3);
                  insertCommandTurn(4);
                  break;   
                case S:
                  insertCommandTurn(3);
                  insertCommandTurn(4);
                  break;   
                case E:
                  insertCommandTurn(4);
                  break;        
              }
              insertCommandMoveForward(5);
            } 
            else {
              if (adjx < curX) {
                  switch(curPos) {
                    case N:
                      insertCommandTurn(1);
                      insertCommandTurn(2);
                      break;   
                    case W:
                      insertCommandTurn(2);
                      break;   
                    case E:
                      insertCommandTurn(4);
                      insertCommandTurn(1);
                      insertCommandTurn(2);
                      break;        
                  }
                  insertCommandMoveForward(7);
              }
              else {
                std::cout << "adjx and curX are same\n";
              }
            }
        }
    }
}

void Map::init() {
  // Define all cells to be unexplored and all sides to be undefined
  for(int i=0; i < MAP_SIZE; i++) {
    for(int j=0; j < MAP_SIZE; j++) {
      map[i][j].status = UNEXPLORED;
      map[i][j].north = UNDEFINED;
      map[i][j].south = UNDEFINED;
      map[i][j].east = UNDEFINED;
      map[i][j].west = UNDEFINED;
      map[i][j].edges_explored = 0;    
    }
  }
  // Define sides of map edges to be closed
  for(int i=0; i < MAP_SIZE; i++) {
    map[0][i].west = CLOSE;
    map[0][i].edges_explored = map[0][i].edges_explored + 1; 
  }

  for(int i=0; i < MAP_SIZE; i++) {
    map[i][0].south = CLOSE;
    map[i][0].edges_explored = map[i][0].edges_explored + 1; 
  }

  for(int i=0; i < MAP_SIZE; i++) {
    map[MAP_SIZE-1][i].east = CLOSE;
    map[MAP_SIZE-1][i].edges_explored = map[MAP_SIZE-1][i].edges_explored + 1; 
  }

  for(int i=0; i < MAP_SIZE; i++) {
    map[i][MAP_SIZE-1].north = CLOSE;
    map[i][MAP_SIZE-1].edges_explored = map[i][MAP_SIZE-1].edges_explored + 1;  
  }

  hasReachedCell = true;
  hasReachedGoal = false;
  hasUpdatedCell = false;
  isPathSet = false;
  curX = 0;
  curY = 0;
  curPos = N;

  // Define starting cell (0,0) to be EMPTY
  map[curY][curX].status = EMPTY;

  std::cout << "Initialized\n";
}

void Map::updateCurPos() {
  int fb=0;
  // Get feedback commands from bot driver
  if (!fb_queue->empty()) {
      fb = fb_queue->front();
      fb_queue->pop_front();
  }
  
  // Update curPos based on feedback
  switch(fb) {
      case 1:
          curPos = W;
          break;
      case 2:
          curPos = S;
          break;
      case 3:
          curPos = E;
          break;
      case 4:
          curPos = N;
          break;
      case 5:
          curX = curX + 1;
          break;
      case 6:
          curY = curY + 1;
          break;
      case 7:
          curX = curX - 1;
          break;
      case 8:
          curY = curY - 1;
          break;
      case 0:
          curPos = curPos;
          break;
  }
}

// Unit test code
void Map::testdeque() {
  cmd_queue->push_back(5);
  cmd_queue->push_back(5);
  cmd_queue->push_back(5);
  cmd_queue->push_back(5);
  cmd_queue->push_back(5);
  cmd_queue->push_back(5);
  /*
  cmd_queue->push_back(1);
  cmd_queue->push_back(2);
  cmd_queue->push_back(3);
  
  cmd_queue->push_back(6);

  cmd_queue->push_back(4);
  cmd_queue->push_back(1);
  cmd_queue->push_back(2);
  
  cmd_queue->push_back(7);

  cmd_queue->push_back(3);
  cmd_queue->push_back(4);
  cmd_queue->push_back(1);
  cmd_queue->push_back(2);

  cmd_queue->push_back(7);

  cmd_queue->push_back(3);
  cmd_queue->push_back(4);
  cmd_queue->push_back(1);

  cmd_queue->push_back(8);

  cmd_queue->push_back(2);
  cmd_queue->push_back(3);
  cmd_queue->push_back(4);
  cmd_queue->push_back(1);
  cmd_queue->push_back(2);

  cmd_queue->push_back(7);
  */
}


/* 
// Code driver
int main(int argc, char** argv)
{

  ros::init(argc, argv, "map");
  ros::NodeHandle nh;
  Map m(nh);

  ros::spin();
  return 0;
}
*/
