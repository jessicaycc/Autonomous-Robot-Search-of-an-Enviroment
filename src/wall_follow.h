#ifndef WALL_FOLLOW_H
#define WALL_FOLLOW_H

#include <cstdio>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <kobuki_msgs/BumperEvent.h>

extern std::vector<double> _lasers;
extern int left_unseen_count;
extern int right_unseen_count;

void _laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
void _bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr &msg);

geometry_msgs::Twist _followWall(int direction);

#endif
