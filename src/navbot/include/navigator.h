#ifndef __NAVIGATOR_H_INCLUDED__
#define __NAVIGATOR_H_INCLUDED__
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <iostream>
#include <cmath>
#include <queue> 
#include <deque> 

#define PI_ 3.1415
#define ANGULAR_VEL 0.3
#define LINEAR_VEL 0.5
#define ANGULAR_ADJUST_VEL  0.01
#define ANGULAR_OFFSET	0.03

class BotController{
    private:
        ros::NodeHandle nh;
        // Publishers and Subscribers
        ros::Subscriber pos_sub;
        ros::Publisher vel_pub;

        std::deque<int>* cmd_queue;
        std::deque<int>* fb_queue;

        // Flags
        bool initflag;

        // Current position bot is facing (North, South, East, West)
        enum Position {N, S, E, W};
        Position curPos;

        // Position variables
        double pos_x, pos_y, init_x, init_y, target_x, target_y; 

        // Angular variables
        double quatx, quaty, quatz, quatw;
        double yaw, pitch, roll; 
        double cur_ang_z, init_ang_z, target_ang_z; 

        // Velocity variables
        double linvel_x, ang_vel; 

    public:
        BotController(ros::NodeHandle &nh, std::deque<int>* command, std::deque<int>* feedback);
        void init(const nav_msgs::OdometryConstPtr& poseMsg);

	    // Movement functions
        void movePosX();
        void movePosY();
        void moveNegX();
        void moveNegY();
        void stop();
        
	    // Turning functions
        void turnNtoW();
        void turnWtoS();
        void turnStoE();
        void turnEtoN();
	    // Adjust pose of bot
        void adjustPose();
	    // Get current pose
        void getcurrentPose(const nav_msgs::OdometryConstPtr& poseMsg);

        void callback(const nav_msgs::OdometryConstPtr& poseMsg);
};

#endif
