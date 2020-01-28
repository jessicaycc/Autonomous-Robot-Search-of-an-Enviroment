#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <time.h>

#include <stdio.h>
#include <cmath>

#include <chrono>

using namespace std;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    float stamp = msg->info.resolution;
    int width = msg->info.width;
    int height = msg->info.height;


    /*
    stamp = msg->time
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
    tf::getYaw(msg->pose.pose.orientation);*/
    ROS_INFO("Resolution: %f", stamp);
    ROS_INFO("Map dim: %d x %d", width, height);
    ROS_INFO("Origin: x: %f, y: %f, z: %f", msg->info.origin.position.x, msg->info.origin.position.y, msg->info.origin.position.z);
    for(int i=0; i<width*height; i++){
        if(i%width==0){
            std::cout << "\n";
        }
        int prob = msg->data[i];
        std::cout << prob << ",";
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    ros::Subscriber map_sub = nh.subscribe("map", 10, &mapCallback);
    ros::Rate loop_rate(10);
    while(ros::ok()) {
        //std::cout << "Printing" << endl;
        ros::spinOnce();
        //ROS_INFO("running");
        loop_rate.sleep();
    }
}   