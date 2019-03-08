#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <iostream>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>
#include <cmath>

#include <queue>
#include <deque>
#include "navigator.h"

/*
BotController::BotController(ros::NodeHandle &nh, std::queue<int>& command, std::queue<int>& feedback){
    initflag = false;
    pos_sub = nh.subscribe("/odom",1,&BotController::callback, this);
    vel_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);
    cmd_queue = command;
    fb_queue = feedback;
}
*/

BotController::BotController(ros::NodeHandle &nh, std::deque<int>* command, std::deque<int>* feedback) {
        initflag = false;
        pos_sub = nh.subscribe("/odom",1,&BotController::callback, this);
        vel_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1); 
        cmd_queue = command;
        fb_queue = feedback;
}

void BotController::init(const nav_msgs::OdometryConstPtr& poseMsg) {
    if(!initflag) {
        
        getcurrentPose(poseMsg);

        // Update initial pose 
        init_x = pos_x;
        init_y = pos_y;
        init_ang_z = yaw;

        // Set velocity var to 0
        linvel_x = 0;
        ang_vel = 0;
        
        // Assume bot is currently facing S when gazebo initialized
        curPos = N;

        // Fixed distance and angular increments to move bot
        target_ang_z = PI_/2;
        target_x = 1.0;
        target_y = 1.0;

        // Only initialize once when node is started
        initflag = true;
    }
}

void BotController::stop() {
    //aimForward(0.0);
    linvel_x = 0.0;
    ang_vel = 0.0; 
}

void BotController::movePosX () {
    if( (abs(pos_x - init_x)) < target_x){ 
        linvel_x = LINEAR_VEL; // bot linear velocity
    } else {
        init_x = pos_x;
        linvel_x = 0;
        std::cout << "Moved forward\n";
        cmd_queue->pop_front();
        fb_queue->push_back(5);
        cmd_queue->push_front(9);
    }
}

void BotController::moveNegX () {
    if( (abs(pos_x - init_x)) < target_x){ 
        linvel_x = LINEAR_VEL; // bot linear velocity
    } else {
        init_x = pos_x;
        linvel_x = 0;
        std::cout << "Moved Backward\n";
        cmd_queue->pop_front();
        fb_queue->push_back(7);
        cmd_queue->push_front(9);
    }
}

void BotController::movePosY () {
    if( (abs(pos_y - init_y)) < target_y){ 
        linvel_x = LINEAR_VEL; // bot linear velocity
    } else {
        init_y = pos_y;
        linvel_x = 0;
        std::cout << "Moved pos side\n";
        cmd_queue->pop_front();
        fb_queue->push_back(6);
        cmd_queue->push_front(9);
    }
}

void BotController::moveNegY () {
    if( (abs(pos_y - init_y)) < target_y){ 
        linvel_x = LINEAR_VEL; // bot linear velocity
    } else {
        init_y = pos_y;
        linvel_x = 0;
        std::cout << "Moved neg side\n";
        cmd_queue->pop_front();
        fb_queue->push_back(8);
        cmd_queue->push_front(9);
    }
}

void BotController::turnNtoW() {
    if (curPos == N) {
        if ( (cur_ang_z - init_ang_z) < (target_ang_z)){
            ang_vel = ANGULAR_VEL; // bot angular velocity
        } else {
            init_ang_z = cur_ang_z;
            ang_vel = 0;
            curPos = W;
            std::cout << "Facing W\n";
            cmd_queue->pop_front();
            fb_queue->push_back(1);
            cmd_queue->push_front(9);
        }
    }
}

void BotController::turnWtoS() {
    if (curPos == W) {
        bool pop=false;
        if ( (cur_ang_z - init_ang_z) < (target_ang_z)){
            ang_vel = ANGULAR_VEL; // bot angular velocity
        }  
        else {
            //init_ang_z = cur_ang_z;
	    init_ang_z = PI_;
            ang_vel = 0;
            curPos = S;
            std::cout << "Facing S\n";
            cmd_queue->pop_front();
            fb_queue->push_back(2);
            cmd_queue->push_front(9);
            pop = true;
        }
        if (cur_ang_z < 0) {
            //init_ang_z = cur_ang_z;
	    init_ang_z = PI_;
            ang_vel = 0;                    
            curPos = S;
            std::cout << "Facing S\n";
            if (!pop) {
                cmd_queue->pop_front();
                fb_queue->push_back(2);
                cmd_queue->push_front(9);
            }
        }
    }
}

void BotController::turnStoE() {
    if (curPos == S) {
	if (cur_ang_z > 0) {
		if ( (cur_ang_z - init_ang_z) < target_ang_z){
		    ang_vel = ANGULAR_VEL; // bot angular velocity
		}
        } 
	else {
		if (cur_ang_z < 0) {
			if ( (cur_ang_z + init_ang_z) < target_ang_z){
			    ang_vel = ANGULAR_VEL; // bot angular velocity
			} 
			else {
			    init_ang_z = cur_ang_z;
			    ang_vel = 0;
			    curPos = E;
			    std::cout << "Facing E\n";
			    cmd_queue->pop_front();
			    fb_queue->push_back(3);
			    cmd_queue->push_front(9);
			}
		} 
	}
    }
}

void BotController::turnEtoN() {
    if (curPos == E) {
        bool pop=false;
        if ( (cur_ang_z - init_ang_z) < (target_ang_z)){
            ang_vel = ANGULAR_VEL; // bot angular velocity
        }  
        else {
            init_ang_z = cur_ang_z;
            ang_vel = 0;
            curPos = N;
            std::cout << "Facing N\n";
            cmd_queue->pop_front();
            fb_queue->push_back(4);
            cmd_queue->push_front(9);
            pop = true;
        }
        if (cur_ang_z > 0) {
            init_ang_z = cur_ang_z;
            ang_vel = 0;                    
            curPos = N;
            std::cout << "Facing N\n";
            if (!pop) {
                cmd_queue->pop_front();
                fb_queue->push_back(4);
                cmd_queue->push_front(9);
            }
        }
    }
}

void BotController::getcurrentPose(const nav_msgs::OdometryConstPtr& poseMsg) {
    // Get current position
    pos_x = poseMsg->pose.pose.position.x;
    pos_y = poseMsg->pose.pose.position.y;

    // Get current orientation (yaw) from quarternion
    quatx= poseMsg->pose.pose.orientation.x;
    quaty= poseMsg->pose.pose.orientation.y;
    quatz= poseMsg->pose.pose.orientation.z;
    quatw= poseMsg->pose.pose.orientation.w;
    tf::Quaternion q(quatx, quaty, quatz, quatw);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    cur_ang_z = yaw;
    //std::cout << "yaw: " << cur_ang_z << "\n"; //DEBUG
}

void BotController::adjustPose() {
    if (curPos == N) {
        if ((cur_ang_z - 0) > ANGULAR_OFFSET) {
            // Turn bot slightly right
            ang_vel = -ANGULAR_ADJUST_VEL;
	    std::cout << "Adjusting north right\n"; //DEBUG
        }
        else if ((cur_ang_z - 0) < -ANGULAR_OFFSET) {
            // Turn bot slightly left
            ang_vel = ANGULAR_ADJUST_VEL;
	    std::cout << "Adjusting north left\n"; //DEBUG
        }
        else {
            cmd_queue->pop_front();
	    std::cout << "Adjusted north\n"; //DEBUG
        }
    }
    if (curPos == S) {
	if (cur_ang_z < 0) {
		if ((cur_ang_z + PI_) > ANGULAR_OFFSET) {
		    // Turn bot slightly right
		    ang_vel = -ANGULAR_ADJUST_VEL;
		    std::cout << "Adjusting south right\n"; //DEBUG
		}
		else {
		    cmd_queue->pop_front();
		    std::cout << "Adjusted south\n"; //DEBUG
		}
	}
	else {
	       	if ((cur_ang_z - PI_) < -ANGULAR_OFFSET) {
		    // Turn bot slightly left
		    ang_vel = ANGULAR_ADJUST_VEL;
		    std::cout << "Adjusting south left\n"; //DEBUG
		}
		else {
		    cmd_queue->pop_front();
		    std::cout << "Adjusted south\n"; //DEBUG
		}
	}
    }
    if (curPos == W) {
        if ((cur_ang_z - PI_/2) > ANGULAR_OFFSET) {
            // Turn bot slightly right
            ang_vel = -ANGULAR_ADJUST_VEL;
	    std::cout << "Adjusting west right\n"; //DEBUG
        }
        else if ((cur_ang_z - PI_/2) < -ANGULAR_OFFSET) {
            // Turn bot slightly left
            ang_vel = ANGULAR_ADJUST_VEL;
	    std::cout << "Adjusting west left\n"; //DEBUG
        }
        else {
            cmd_queue->pop_front();
	    std::cout << "Adjusted west\n"; //DEBUG
        }
    }
    if (curPos == E) {
        if ((cur_ang_z - (-PI_/2)) > ANGULAR_OFFSET) {
            // Turn bot slightly right
            ang_vel = -ANGULAR_ADJUST_VEL;
	    std::cout << "Adjusting east right\n"; //DEBUG
        }
        else if ((cur_ang_z - (-PI_/2)) < -ANGULAR_OFFSET) {
            // Turn bot slightly left
            ang_vel = ANGULAR_ADJUST_VEL;
	    std::cout << "Adjusting east left\n"; //DEBUG
        }
        else {
            cmd_queue->pop_front();
	    std::cout << "Adjusted east\n"; //DEBUG
        }
    }
}

void BotController::callback(const nav_msgs::OdometryConstPtr& poseMsg){
    geometry_msgs::Twist base_cmd;
    int cmd=0;

    init(poseMsg);
    getcurrentPose(poseMsg);
    
    // Bot driver: Get commands from queue
    if (!cmd_queue->empty()) {
        cmd = cmd_queue->front();
    }
    
    // Call movement functions based on commands
    switch(cmd) {
        case 1:
            turnNtoW();
            break;
        case 2:
            turnWtoS();
            break;
        case 3:
            turnStoE();
            break;
        case 4:
            turnEtoN();
            break;
        case 5:
            movePosX();
            break;
        case 6:
            movePosY();
            break;
        case 7:
            moveNegX();
            break;
        case 8:
            moveNegY();
            break;
        case 0:
            stop();
            break;
        case 9:
            adjustPose();
            break;
    }

    // Publish linear and angular velocities
    base_cmd.linear.x = linvel_x;
    base_cmd.angular.z = ang_vel;
    vel_pub.publish(base_cmd);

}

/*
// Unit Test: Code driver
int main(int argc, char** argv){
    ros::init(argc, argv, "navigator");
    ros::NodeHandle nh;
    BotController bc(nh);
    
    // Test drive in world 1
    bc.command.push(5);
    bc.command.push(1);
    bc.command.push(2);
    bc.command.push(3);
    bc.command.push(6);
    bc.command.push(4);
    bc.command.push(1);
    bc.command.push(6);
    bc.command.push(2);
    bc.command.push(5);
    
    ros::spin();
    return 0;
}
*/
