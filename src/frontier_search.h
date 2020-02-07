#ifndef FRONTIER_SEARCH_H
#define FRONTIER_SEARCH_H

using namespace std;

#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/PointStamped.h>
#include <time.h>

#include <stdio.h>
#include <cmath>

#include <chrono>

#include <vector>
#include <queue>

//Constants for frontier search labels
enum Marker {MAP_OPEN_LIST=0, MAP_CLOSE_LIST, FRONTIER_OPEN_LIST, FRONTIER_CLOSE_LIST};

bool is_a_frontier_point(const pair<int,int> &p);
vector<pair<int,int>> get_medians(vector<vector<pair<int,int>>> list_of_frontiers);
bool has_open_neighbour(const vector<vector<int>> &marker_list, const pair<int,int> &p);
vector<pair<int,int>> findNeighbours(const pair<int,int> &p);
vector<pair<double,double>> wfd(tf::TransformListener &tf_listener);

#endif
