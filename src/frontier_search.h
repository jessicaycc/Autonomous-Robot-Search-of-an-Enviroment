using namespace std;

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
#include <unordered_map>
#include <vector>


enum Marker {MAP_OPEN_LIST=0, MAP_CLOSE_LIST, FRONTIER_OPEN_LIST, FRONTIER_CLOSE_LIST};

bool is_a_frontier_point(pair<int,int> p);

vector<pair<int,int>> get_medians(vector<vector<pair<int,int>>> list_of_frontiers);

bool is_open_neighbour(unordered_map<pair<int,int>, Marker> marker_list, pair<int,int> p);

vector<pair<int,int>> findNeighbours(pair<int,int> p);

vector< vector<pair<int,int>>> wfd(int argc, char **argv);