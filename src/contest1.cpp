#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

using namespace std;
#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad) * 180. / M_PI)
#define DEG2RAD(deg) ((deg) * M_PI / 180.)

float posX=0.0, posY=0.0, yaw=0.0;
uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
int32_t nLasers=0, desiredNLasers = 0, desiredAngle=5;
std::vector<float> lasers; 
float minLaserDist = std::numeric_limits<float>::infinity();
enum STATE_ENUM {reset, left_wall_follow, left_wall_turn_right};
STATE_ENUM state = reset;



#include <stdio.h>
#include <cmath>

#include <chrono>


void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
	//fill with your code

       //std::cout<< "State=" << msg->state << endl;
       //std::cout<< "Bumper=" << msg->bumper << endl;
    bumper[msg->bumper] = msg->state;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	//fill with your code
    minLaserDist = std::numeric_limits<float>::infinity();
    nLasers = (msg->angle_max - msg->angle_min)/msg->angle_increment;
    
    desiredNLasers = desiredAngle*M_PI / (180*msg->angle_increment);
    if (desiredAngle * M_PI / 180 < msg->angle_max && -desiredAngle * M_PI / 180 > msg->angle_min) {
        for (uint32_t laser_idx = nLasers / 2 - desiredNLasers; laser_idx < nLasers / 2 + desiredNLasers; ++laser_idx){
            minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
        }
    }
    else {
        for (uint32_t laser_idx = 0; laser_idx < nLasers; ++laser_idx) {
            minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
        }
    }
    lasers.clear();
    
    for (uint32_t laser_i = 0; laser_i < nLasers; laser_i++)
    {
        lasers.push_back(msg->ranges[laser_i]);
    }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
    tf::getYaw(msg->pose.pose.orientation);
    ROS_INFO("Position:(%f, %f) Orientation: %f rad or %f degrees.", posX,posY, yaw, RAD2DEG(yaw));

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
    ros::Subscriber odom = nh.subscribe("odom", 1, odomCallback);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    ros::Rate loop_rate(20);

    geometry_msgs::Twist vel;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    float angular = 0.0;
    float linear = 0.0;
    int direction = 0;


    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();
        
        //fill with your code
        ROS_INFO("Postion: (%f, %f) Orientation: %f degrees Range: %f", posX, posY, RAD2DEG(yaw), minLaserDist);
        if (!lasers.empty()){
            ROS_INFO("%f, %f, %f", lasers[(int)(nLasers/2)], lasers[10], lasers[nLasers-10]);
        }
        
        bool any_bumper_pressed = false;
        for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx) {
            any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
        }
        if (lasers.empty())
        {
            linear = 0;
            angular = 0.0;
        }
        else if (lasers[(int)(nLasers/2 -1)] >0.1 && lasers[10]>0.01 && lasers[nLasers-10]>0.01 )
        {
            linear = 0.25;
            if (direction ==0)
            {
                angular = -0.2;
            }
            else
            {
                angular = 0.2;
            }
            
            
        }
        else if (direction ==0)
        {
            linear = 0.0;
            angular = 0.4;
        }
        else
        {
            linear = 0.0;
            angular = -0.4;
        }
        if (direction ==0 && secondsElapsed >100 && (abs(posX)<0.2 && abs(posY)<0.2))
        {
            direction =1;
        }
        /*
        switch(state)
        {
            case reset:

            
                ROS_INFO("reset");
                if (lasers.empty())
                {
                    break;
                }
                if (lasers[(int)(nLasers/2)] >0.5 && lasers[0]>0.5 && lasers[nLasers-1]>0.5)
                {
                    linear = 0.25;
                    angular = 0.0;
                }
                else
                {
                    if (lasers[(int)(nLasers/2)] <0.5 || lasers[nLasers -1] <0.5)
                    {
                        state = left_wall_turn_right;
                    }
                    else
                    {
                        state =left_wall_follow;
                    }
                }
                break;
            case left_wall_follow:
                ROS_INFO("left wall follow");
                if (lasers[(int)(nLasers/2)] >0.5)
                {
                    linear = 0.25;
                    angular = 0;
                    if (lasers[0]>0.5)
                    {
                        angular = 0.3;
                    }
                    if (lasers[0] <0.25)
                    {
                        angular = -0.3;
                    }
                    
                }
                else
                {
                    state = left_wall_turn_right;
                }
                break;
            case left_wall_turn_right:
                ROS_INFO("left wall turn right");
                cout << lasers[0] << "," << lasers[(int)(nLasers/2)] << "," <<lasers[nLasers-1]  <<endl;
                if (lasers[(int)(nLasers/2)] >0.5 && lasers[nLasers -1]>0.25 && lasers[0]>0.25)
                {
                    state = left_wall_follow;state = left_wall_follow;
                }
                else
                {
                    linear = 0;
                    angular = -0.4;
                    
                }
                break;
*/

                
                
        /*
        if (posX < 0.5 && yaw < M_PI / 12 && !any_bumper_pressed) {
            angular = 0.0;
            linear = 0.2;
        }
        else if (yaw < M_PI / 2 && posX > 0.5 && !any_bumper_pressed) {
            angular = M_PI / 6;
            linear = 0.0;
        }
        else if (minLaserDist > 1. && !any_bumper_pressed) {
            linear = 0.1;
            if (yaw < 17 / 36 * M_PI || posX > 0.6) {
                angular = M_PI / 12.;
            }
            else if (yaw < 19 / 36 * M_PI || posX < 0.4) {
                angular = -M_PI / 12.;
            }
            else {
                angular = 0;
            }
        }
        else {
            angular = 0.0;
            linear = 0.0;
        }
        */
        /*
        // Control logic after bumpers are being pressed.
        if (posX < 0.5 && yaw < M_PI / 12 && !any_bumper_pressed) {
            angular = 0.0;
            linear = 0.2;
        }
        else if (yaw < M_PI / 2 && posX > 0.5 && !any_bumper_pressed) {
            angular = M_PI / 6;
            linear = 0.0;
        }
        else {
            angular = 0.0;
            linear = 0.0;
        }
        */
        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);
        
  
        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    return 0;
}
