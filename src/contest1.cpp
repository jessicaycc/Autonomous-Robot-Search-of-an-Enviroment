#include <cstdio>
#include <chrono>
#include <cmath>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <kobuki_msgs/BumperEvent.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include "wall_follow.h"
#include "frontier_search.h"

#include "globals.h"

/*DISABLE RVIZ MARKERS */
//#define DISABLE_VISUALIZATIONS 
/*
#ifndef DISABLE_VISUALIZATIONS
#include "make_marker.h"
#endif*/

/* Occupancy grid callback globals */
int occ_width = 0;  //Occupancy grid meta data
int occ_height = 0;
std::vector<std::vector<int>> occ_grid;  //Occupancy grid map
float pose_pos [3] = {-1, -1, -1}; //xyz
float pose_origin [3] = {-1, -1, -1}; //xyz
float pose_orientation [4] = {-1, -1, -1, -1}; //Quaternion xyzw
float res;

template <typename T> int sgn(T val)
{
        return (T(0) < val) - (val < T(0));
}

class Controller
{
        enum class Direction {
                RIGHT,
                LEFT,
        };

        enum class State {
                NONE,
                FOLLOW_WALL,
                BEE_LINE,
                GO_STRAIGHT,
                STOPPED,
        };

        struct Point2D {
                double x;
                double y;
        };

        struct Pose2D {
                double x;
                double y;
                double theta;
        };

        struct Twist2D {
                double linear;
                double angular;
        };

        struct Line2D{
                double m;
                double b;
        };

public:
        Controller(ros::NodeHandle &nh);

        void start(pair<double,double> point);
        void start(void);
        void reset(void);

        geometry_msgs::Twist update(void);

        bool stopped() { return state == State::STOPPED; }

private:
        const double tol = 0.25;
        const double vlmax = 0.25;
        const double vamax = 1;

        int state_timer;
        bool exploring;
        Controller::State state;
        Controller::Direction dir;

        std::vector<double> lasers;

        Controller::Point2D dest;
        Controller::Twist2D vel;
        Controller::Pose2D pose;
        Controller::Line2D path;

        ros::Subscriber sub_odom;
        ros::Subscriber sub_laser;
        ros::Subscriber sub_bump;
        ros::Subscriber sub_map;

        void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
        void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
        void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr &msg);
        void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);

        bool wallDetected();
        bool onPath();
        bool onTarget();
        geometry_msgs::Twist gotoPoint(void);
        geometry_msgs::Twist followWall(void);
};

Controller::Controller(ros::NodeHandle &nh)
{
        using nav_msgs::Odometry;
        using sensor_msgs::LaserScan;
        using kobuki_msgs::BumperEvent;
        using nav_msgs::OccupancyGrid;

        reset();

        sub_odom = nh.subscribe<Odometry>("/odom", 10,
                &Controller::odomCallback, this);

        sub_laser = nh.subscribe<LaserScan>("/scan", 10,
                &Controller::laserCallback, this);

        sub_bump = nh.subscribe<BumperEvent>("/mobile_base/events/bumper", 10,
                &Controller::bumperCallback, this);

        sub_map = nh.subscribe<OccupancyGrid>("/map", 10,
                &Controller::mapCallback, this);
}

void Controller::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
        pose.x = msg->pose.pose.position.x;
        pose.y = msg->pose.pose.position.y;
        pose.theta = tf::getYaw(msg->pose.pose.orientation);
}

void Controller::laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
        _laserCallback(msg);
        lasers = _lasers;
}

void Controller::bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr &msg)
{
        _bumperCallback(msg);
}

void Controller::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
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

        /* y, x form (y rows of x length).
         */
        occ_grid = vector<vector<int>>(occ_height, vector<int>(occ_width, -1));

        for (int i = 0; i < occ_width*occ_height; i++) {
                int prob = msg->data[i];
                occ_grid[i/occ_width][i%occ_width] = msg->data[i];
        }
}

void Controller::start(pair<double,double> point)
{
        state = State::BEE_LINE;

        dest.x = point.first;
        dest.y = point.second;

        path.m = (dest.y-pose.y) / (dest.x-pose.x);
        path.b = pose.y - path.m*pose.x;
}

void Controller::start()
{
        exploring = true;
        state = State::GO_STRAIGHT;
}

void Controller::reset()
{
        state_timer = 0;
        left_unseen_count = 0;
        right_unseen_count = 0;
        exploring = false;
        state = State::STOPPED;
        dir = Direction::LEFT;
}

bool Controller::wallDetected()
{
        using kobuki_msgs::BumperEvent;

        double min_dist;
        double center_dist;

        for (int i = 0; i < NUM_BUMPER; i++) {
                if (bumper[i] == BumperEvent::PRESSED)
                        return true;
        }

        if (lasers.empty())
                return false;

        min_dist = *std::min_element(lasers.begin(), lasers.end());

        center_dist = *std::min_element(lasers.begin()+300, lasers.begin()+400);

        if ((min_dist > 50) || (min_dist < 0.5) || (center_dist > 50) || (center_dist < 0.5))
                return true;
        else
                return false;
}

bool Controller::onPath()
{
        double y = path.m*pose.x + path.b;

        return (fabs(pose.y-y) < tol);
}

bool Controller::onTarget()
{
        double d = hypot(dest.x-pose.x, dest.y-pose.y);

        return (fabs(d) < tol);
}

geometry_msgs::Twist Controller::gotoPoint()
{
        double dx, dy, da;
        geometry_msgs::Twist ret;

        dx = dest.x - pose.x;
        dy = dest.y - pose.y;
        da = remainder(atan2(dy, dx)-pose.theta, 2*M_PI);

        if (fabs(da) > tol) {
                vel.linear = 0;
                vel.angular = sgn(da)*vamax;
        }
        else {
                vel.linear = vlmax;
                vel.angular = 0;
        }

        ret.linear.x = vel.linear;
        ret.angular.z = vel.angular;

        return ret;
}

geometry_msgs::Twist Controller::followWall()
{
        return _followWall((int) dir);
}

geometry_msgs::Twist Controller::update()
{
        geometry_msgs::Twist ret;

        /* Decide velocity control signal from state.
         */
        switch (state)
        {
        case State::FOLLOW_WALL:
                ret = followWall();
                break;
        case State::BEE_LINE:
                ret = gotoPoint();
                break;
        case State::GO_STRAIGHT:
                ret.linear.x = vlmax;
                ret.angular.z = 0;
                break;
        default:
                ret.linear.x = 0;
                ret.angular.z = 0;
                break;
        };

        /* Do not proceed if the state_timer is set.
         */
        if (state_timer > 0) {
                state_timer--;
                return ret;
        }

        /* Perform a state change if required.
         */
        switch (state)
        {
        case State::FOLLOW_WALL:
                if (exploring && onTarget()) {
                        if (dir == Direction::LEFT)
                                dir = Direction::RIGHT;
                        else
                                dir = Direction::LEFT;
                        ROS_INFO("Position: (%.3f, %.3f)", pose.x, pose.y);
                        state_timer = 400;
                }

                if (!exploring && onPath()) {
                        if (dir == Direction::LEFT)
                                dir = Direction::RIGHT;
                        else
                                dir = Direction::LEFT;
                        left_unseen_count  = 0;
                        right_unseen_count = 0;
                        state = State::BEE_LINE;
                        state_timer = 40;
                }
                break;
        case State::BEE_LINE:
                if (!exploring && wallDetected()) {
                        state = State::FOLLOW_WALL;
                        state_timer = 400;
                }

                if (!exploring && onTarget()) {
                        state = State::STOPPED;
                        state_timer = 400;
                }
                break;
        case State::GO_STRAIGHT:
                if (exploring && wallDetected()) {
                        dest.x = pose.x;
                        dest.y = pose.y;
                        state = State::FOLLOW_WALL;
                        state_timer = 400;
                }
                break;
        default:
                break;
        };

        return ret;
}

std::vector<pair<double,double>> frontier_medians(tf::TransformListener &tf_listener)
{
        /*Returns a vector of pairs of x,y coordinates as destination targets */
        std::vector<pair<double,double>> list_of_medians = wfd(tf_listener);

        /*
        //Debugging code to plot the medians on occ_grid
        std::cout << "Obtained list of medians: There were " << list_of_medians.size() << std::endl;
        for(int i = 0; i<list_of_medians.size(); i++){
                std::cout << "x/y = (" << list_of_medians[i].first << ", " << list_of_medians[i].second << endl;
                occ_grid[list_of_medians[i].first][list_of_medians[i].second] = 50;
        } */
        
        return list_of_medians;
}

int main(int argc, char **argv)
{
        using namespace std::chrono;
        using geometry_msgs::Twist;

        int timer = 0;
        time_point<system_clock> start;
        time_point<system_clock> now;

        int index = 0;
        pair<double,double> dest;
        std::vector<pair<double,double>> list_of_frontiers;

        /* Initialize ros environment.
         */
        ros::init(argc, argv, "terrorizing_turtle");

        ros::NodeHandle nh;
        ros::Publisher pub;

        Controller controller(nh);
        tf::TransformListener tf_listener(nh);

        pub = nh.advertise<Twist>("cmd_vel_mux/input/teleop", 1);

        /* For visualization, disable during real run */
/*
#ifndef DISABLE_VISUALIZATION
        ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
#endif*/

        /* Start the clock, 8 minutes.
         */
        start = system_clock::now();
        ros::Rate rate(20);

        /* Start by wall following an obstacle in front for 3 minutes.
         */
        controller.start();

        while (ros::ok() && (timer < 300)) {
                ros::spinOnce();

                pub.publish(controller.update());

                rate.sleep();

                now = system_clock::now();
                timer = duration_cast<seconds>(now-start).count();

                // ROS_INFO("Time: %d", timer);
        }

        controller.reset();

        /* Use frontier search to finish unexplored areas of the map.
         * Get list of frontiers from frontier search.
         */
        list_of_frontiers = frontier_medians(tf_listener);

/*
#ifndef DISABLE_VISUALIZATION
                generate_markers(marker_pub, list_of_frontiers);
#endif*/

        while (ros::ok() && (timer < 300)) {
                ros::spinOnce();

                if (controller.stopped()) {
                       dest = list_of_frontiers.at(index++);
                       controller.start(dest);
                }

                pub.publish(controller.update());

                rate.sleep();

                now = system_clock::now();
                timer = duration_cast<seconds>(now-start).count();

                // ROS_INFO("Time: %d", timer);
        }

        return 0;
}
