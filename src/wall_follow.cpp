#include <cstdio>
#include <cmath>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <kobuki_msgs/BumperEvent.h>

#include "wall_follow.h"

int bumper[] = {
        kobuki_msgs::BumperEvent::RELEASED,
        kobuki_msgs::BumperEvent::RELEASED,
        kobuki_msgs::BumperEvent::RELEASED
};

int nLasers;
std::vector<double> _lasers;

double left_laser_dist;
double front_laser_dist;
double right_laser_dist;

int backup_count = 0;
int left_unseen_count = 0;
int right_unseen_count = 0;

geometry_msgs::Twist vel;

void _bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr &msg)
{
        bumper[msg->bumper] = msg->state;
}

void _laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
        _lasers.clear();

        nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;

        for (int i = 0; i < nLasers; i++)
        {
                if (std::isnan(msg->ranges[i]))
                        _lasers.push_back(100);
                else
                        _lasers.push_back(msg->ranges[i]);
        }
}

static double get_min_laser_dist(int min_laser_idx, int max_laser_idx)
{
        return *std::min_element(_lasers.begin() + min_laser_idx,
                                 _lasers.begin() + max_laser_idx);
}

geometry_msgs::Twist _followWall(int direction)
{
        using kobuki_msgs::BumperEvent;

        double linear;
        double angular;

        bool any_bumper_pressed = false;

        bool left_bumper_pressed  = (bumper[0] == BumperEvent::PRESSED);
        bool front_bumper_pressed = (bumper[1] == BumperEvent::PRESSED);
        bool right_bumper_pressed = (bumper[2] == BumperEvent::PRESSED);

        // ROS_INFO("Bumpers: %d, %d, %d", left_bumper_pressed,
        //         front_bumper_pressed, right_bumper_pressed);

        for (int i = 0; i < NUM_BUMPER; i++)
                any_bumper_pressed |= (bumper[i] == BumperEvent::PRESSED);

        if (any_bumper_pressed)
        {
                ROS_INFO("Backing up!");

                backup_count = 25;

                if (front_bumper_pressed)
                {
                        linear = -0.1;
                        angular = 0;
                }
                else if (left_bumper_pressed)
                {
                        linear = -0.1;
                        angular = -0.3;
                }
                else if (right_bumper_pressed)
                {
                        linear = -0.1;
                        angular = 0.3;
                }
        }
        else if (backup_count > 0)
        {
                backup_count--;

                linear = vel.linear.x;
                angular = vel.angular.z;
        }
        else if (!_lasers.empty())
        {
                left_laser_dist  = get_min_laser_dist(nLasers-10, nLasers-1);
                front_laser_dist = get_min_laser_dist(nLasers/2-150,
                                                      nLasers/2+150);

                if (direction == 0)
                        right_laser_dist = get_min_laser_dist(0, 10);
                else
                        right_laser_dist = get_min_laser_dist(0, 30);

                // ROS_INFO("Lasers: %f, %f, %f", left_laser_dist,
                //         front_laser_dist, right_laser_dist);

                linear = 0.25;
                angular = 0;

                if ((front_laser_dist < 0.5)
                        || (left_laser_dist < 0.5)
                        || (right_laser_dist < 0.5)
                        || (direction == 1 && right_laser_dist > 50)
                        || (direction == 0 && left_laser_dist > 50))
                {
                        linear = 0;
                        left_unseen_count = 0;
                        right_unseen_count = 0;

                        if (direction == 0)
                                angular = -0.7;
                        else
                                angular = 0.7;
                }
                else if ((front_laser_dist > 50)
                        || (left_laser_dist > 50 && right_laser_dist > 50))
                {
                        linear = 0;
                        left_unseen_count = 0;
                        right_unseen_count = 0;

                        if (direction == 0)
                                angular = -0.4;
                        else
                                angular = 0.4;
                }
                else if ((left_laser_dist > 1)
                        && (left_laser_dist < 50)
                        && (direction == 0))
                {
                        linear = 0.2;
                        angular = 0.2;
                        left_unseen_count++;

                        if ((left_unseen_count > 50)
                                && (left_unseen_count < 1000))
                        {
                                ROS_INFO("Sharp left!");

                                linear = 0.05;
                                angular = 0.4;
                        }
                }
                else if ((right_laser_dist > 1)
                        && (right_laser_dist < 50)
                        && (direction == 1))
                {
                        linear = 0.2;
                        angular = -0.2;
                        right_unseen_count++;

                        if ((right_unseen_count > 40)
                                && (right_unseen_count < 1500))
                        {
                                ROS_INFO("Sharp right!");

                                linear = 0.01;
                                angular = -0.4;
                        }
                }
        }
        else
        {
                linear = 0;
                angular = 0;
        }

        vel.linear.x = linear;
        vel.angular.z = angular;

        return vel;
}
