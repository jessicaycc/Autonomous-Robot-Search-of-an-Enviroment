#include "frontier_search.h"
#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#include <stdio.h>
#include <cmath>

#include <chrono>

using namespace std;
#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad) * 180. / M_PI)
#define DEG2RAD(deg) ((deg) * M_PI / 180.)

float posX=0.0, posY=0.0, yaw=0.0;
uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
int32_t nLasers=0, desiredNLasers = 0, desiredAngle=5;
std::vector<float> lasers; 
float minLaserDist = std::numeric_limits<float>::infinity();
enum STATE_ENUM {reset, wall_follow};
STATE_ENUM state = reset;
float left_laser_dist, forward_laser_dist , right_laser_dist;
int left_unseen_count = 0;
int right_unseen_count = 0;

/* Occupancy grid callback globals */
uint32_t occ_width = 0;  //Occupancy grid meta data 
uint32_t occ_height = 0;
std::vector<std::vector<int>> occ_grid;  //Occupancy grid map
float pose_pos [3] = {-1, -1, -1}; //xyz
float pose_orientation [4] = {-1, -1, -1, -1}; //Quaternion xyzw

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
    nLasers = (msg->angle_max - msg->angle_min)/msg->angle_increment;
    
    lasers.clear();
    
    for (uint32_t laser_i = 0; laser_i < nLasers; laser_i++)
    {
        lasers.push_back(msg->ranges[laser_i]);
    }
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    float res = msg->info.resolution;
    uint32_t width = msg->info.width;
    uint32_t height = msg->info.height;
    
    /*
    stamp = msg->time
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
    tf::getYaw(msg->pose.pose.orientation);*/
    ROS_INFO("Resolution: %f", res);
    ROS_INFO("Map dim: %d x %d", width, height);
    occ_height = height;
    occ_width = width;
    ROS_INFO("Origin: x: %f, y: %f, z: %f", msg->info.origin.position.x, msg->info.origin.position.y, msg->info.origin.position.z);
    pose_pos[0] = msg->info.origin.position.x;
    pose_pos[1] = msg->info.origin.position.y;
    pose_pos[2] = msg->info.origin.position.z;
    pose_orientation[0] = msg->info.origin.orientation.x;
    pose_orientation[1] = msg->info.origin.orientation.y;
    pose_orientation[2] = msg->info.origin.orientation.z;
    pose_orientation[3] = msg->info.origin.orientation.w;
    /*
    for(int i=0; i<width*height; i++){
        if(i%width==0){
            std::cout << "\n";
        }
        int prob = msg->data[i];
        std::cout << prob << ",";
    }*/
}

float get_min_laser_dist(int min_laser_idx, int max_laser_idx)
{
    return *std::min_element(lasers.begin()+min_laser_idx, lasers.begin() + max_laser_idx);
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
    ros::Subscriber map_sub = nh.subscribe("map", 10, &mapCallback);
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
            
        }
        
        bool any_bumper_pressed = false;
        for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx) {
            any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
        }
        if (!lasers.empty())
        {
            if (direction ==0)
            {
                 left_laser_dist = get_min_laser_dist(nLasers-10, nLasers-1);
                forward_laser_dist = get_min_laser_dist(nLasers/2 - 20, nLasers -50);
                right_laser_dist = get_min_laser_dist(0, 10); 
            }
            else
            {
                left_laser_dist = get_min_laser_dist(nLasers-10, nLasers-1);
                forward_laser_dist = get_min_laser_dist(50, nLasers/2 +20);
                right_laser_dist = get_min_laser_dist(0, 10); 
            }
            
           
        }
       
        
        switch(state)
        {
            case reset:

            
                ROS_INFO("reset");
                if (lasers.empty())
                {
                    break;
                }
                if (left_laser_dist >0.25 && forward_laser_dist>0.5 && right_laser_dist >0.25)
                {
                    linear = 0.25;
                    angular = 0.0;
                }
                else if (right_laser_dist <0.25)
                {
                    linear = 0;
                    angular = 0.4;
                }
                else 
                {
                    state =wall_follow;
                    
                }
                break;
            case wall_follow:
                ROS_INFO(" wall follow");
                if (lasers.empty())
                {
                    linear = 0;
                    angular = 0.0;
                }
                else
                {
                    
                    linear = 0.25;
                    angular = 0;
                    ROS_INFO("%f, %f, %f", left_laser_dist, forward_laser_dist, right_laser_dist);

                    if (forward_laser_dist <.7 || left_laser_dist <.5|| right_laser_dist <.5)
                    {   
                        linear = 0;
                        if (direction == 0) //left
                        {
                            angular = -0.7;
                            if (right_laser_dist <.5)
                            {
                                angular = 0.4;
                            }
                        }
                        else //right
                        {
                            angular = 0.7;
                            if (left_laser_dist <.5)
                            {
                                angular = -0.4;
                            }
                        }
                        left_unseen_count =0;
                        right_unseen_count = 0;
                    }
                    else if (isnan(forward_laser_dist)  && isnan(left_laser_dist) && isnan( right_laser_dist))
                    {
                        if (direction == 0) //left
                        {
                            angular = -0.4;
                        }
                        else
                        {
                            angular = 0.4;
                        }
                        
                        linear = 0;
                        
                        left_unseen_count =0;
                        right_unseen_count = 0;
                    }
                    else if (left_laser_dist > 1 && direction ==0)
                    {
                        linear =0.2;
                        angular = 0.2;
                        left_unseen_count +=1;
                        if (left_unseen_count >70 && left_unseen_count <1000)
                        {
                            linear =0.05;
                            angular = 0.4;
                            ROS_INFO("sharp left");
                        }
                    }
                    else if (right_laser_dist > 1 && direction ==1)
                    {
                        linear =0.2;
                        angular = -0.2;
                        right_unseen_count +=1;
                        if (right_unseen_count >70 && right_unseen_count <1000)
                        {
                            linear =0.05;
                            angular = -0.4;
                            ROS_INFO("sharp right");
                        }
                    }

                    
                }
                break;
        }
        
        if (secondsElapsed > 10){
            std::vector<std::vector<pair<int,int>>> list_of_frontiers = wfd();
            std::vector<pair<int,int>> list_of_medians = get_medians(list_of_frontiers);
            std::cout << "Obtained list of medians: There were " << list_of_medians.size() << std::endl;
        }

        if (secondsElapsed >30)
        {
            direction =1;
        }
        if (any_bumper_pressed)
        {
            angular = 0;
            linear = -0.2;
        }
        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);
        
  
        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    return 0;
}
