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
#include "globals.h"
#include "frontier_search.h"
using namespace std;

/* Occupancy grid callback globals */
int occ_width = 0;  //Occupancy grid meta data 
int occ_height = 0;
std::vector<std::vector<int>> occ_grid;  //Occupancy grid map
float pose_pos [3] = {-1, -1, -1}; //xyz
float pose_origin [3] = {-1, -1, -1}; //xyz
float pose_orientation [4] = {-1, -1, -1, -1}; //Quaternion xyzw
float res;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    std::cout <<"MAP LOADED" << endl;
    res = msg->info.resolution;
    occ_height = msg->info.height;
    occ_width = msg->info.width;
    pose_origin[0] = msg->info.origin.position.x;
    pose_origin[1] = msg->info.origin.position.y;
    pose_origin[2] = msg->info.origin.position.z;
    pose_orientation[0] = msg->info.origin.orientation.x;
    pose_orientation[1] = msg->info.origin.orientation.y;
    pose_orientation[2] = msg->info.origin.orientation.z;
    pose_orientation[3] = msg->info.origin.orientation.w;
    occ_grid = vector<vector<int>> (
		occ_height,
		vector<int>(occ_width, -1)
	); //y,x form (y rows of x length)

    for(int i=0; i<occ_width*occ_height; i++){
        int prob = msg->data[i];
        occ_grid[i/occ_width][i%occ_width] = msg->data[i];
    }
}

std::vector<pair<int,int>> frontier_medians(tf::TransformListener &tf_listener)
{
        //std::cout << "Start timing" << endl;
        //auto start = std::chrono::system_clock::now();
        std::vector<pair<int,int>> list_of_medians = wfd(tf_listener);
            //get_medians(list_of_frontiers);
        std::cout << "Obtained list of medians: There were " << list_of_medians.size() << std::endl;
        //auto end = std::chrono::system_clock::now();    
        //auto elapsed =     std::chrono::duration_cast<std::chrono::seconds>(end - start);
        //std::cout << "Elapsed time" << elapsed.count() << '\n';
            
        for(int i = 0; i<list_of_medians.size(); i++){
        std::cout << "x/y = (" << list_of_medians[i].first << ", " << list_of_medians[i].second << endl;
                occ_grid[list_of_medians[i].first][list_of_medians[i].second] = 50;
        }

        return list_of_medians;
}

int main(int argc, char **argv)
{   
    auto start = std::chrono::system_clock::now();
    ros::init(argc, argv, "map_listener");
    ros::NodeHandle nh;
    ros::Subscriber map_sub = nh.subscribe("map", 10, &mapCallback);
    ros::Rate loop_rate(10);

    

    while(ros::ok()) {
        std::cout << "Printing" << endl;
        ros::spinOnce();
        //ROS_INFO("running");
        auto end = std::chrono::system_clock::now();    
        auto elapsed =     std::chrono::duration_cast<std::chrono::seconds>(end - start);
        if(elapsed.count() > 10) {
            tf::TransformListener tf_listener(nh);
            frontier_medians(tf_listener);
        }
        loop_rate.sleep();
    }
}   