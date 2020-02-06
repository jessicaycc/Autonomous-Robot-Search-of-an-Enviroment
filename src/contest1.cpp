#include <cstdio>
#include <chrono>
#include <cmath>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <kobuki_msgs/BumperEvent.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include "wall_follow.h"
//#include "frontier_search.h"
#include "globals.h"

/* Occupancy grid callback globals *//*
uint32_t occ_width = 0;  //Occupancy grid meta data 
uint32_t occ_height = 0;
std::vector<std::vector<int>> occ_grid;  //Occupancy grid map
float pose_pos [3] = {-1, -1, -1}; //xyz
float pose_origin [3] = {-1, -1, -1}; //xyz
float pose_orientation [4] = {-1, -1, -1, -1}; //Quaternion xyzw
float res;
*/
#define TO_RAD(_DEG)    ((_DEG) * M_PI/180)
#define TO_DEG(_RAD)    ((_RAD) * 180/M_PI)

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

        void set(geometry_msgs::PointStamped &point);
        void start(void);
        void stop(void);

        geometry_msgs::Twist update(void);

        bool arrived() const { return arrived_; }

private:
        bool stopped;
        bool arrived_;
        bool wall_following;

        int time_limit;

        const double tol = 1e-1;
        const double vlmax = 0.25;
        const double vamax = 0.4;

        std::vector<double> lasers;

        Controller::Point2D dest;
        Controller::Twist2D vel;
        Controller::Pose2D pose;
        Controller::Line2D path;

        ros::Subscriber sub_odom;
        ros::Subscriber sub_laser;
        ros::Subscriber sub_bump;

        void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
        void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
        void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr &msg);

        bool detectWall();
        bool onPath();

        geometry_msgs::Twist goStraight(void);
        geometry_msgs::Twist followWall(Controller::Direction dir);
};

Controller::Controller(ros::NodeHandle &nh)
{
        using nav_msgs::Odometry;
        using sensor_msgs::LaserScan;
        using kobuki_msgs::BumperEvent;

        stop();

        arrived_ = false;
        wall_following = false;

        sub_odom = nh.subscribe<Odometry>("/odom", 10,
                &Controller::odomCallback, this);

        sub_laser = nh.subscribe<LaserScan>("/scan", 10,
                &Controller::laserCallback, this);

        sub_bump = nh.subscribe<BumperEvent>("/mobile_base/events/bumper", 10,
                &Controller::bumperCallback, this);
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

void Controller::set(geometry_msgs::PointStamped &point)
{
        arrived_ = false;

        if (point.header.frame_id.compare("/odom")) {
                ROS_ERROR("Expected frame_id /odom, got frame_id %s",
                        point.header.frame_id.c_str());
        }

        dest.x = point.point.x;
        dest.y = point.point.y;

        path.m = (dest.y-pose.y) / (dest.x-pose.x);
        path.b = pose.y - path.m*pose.x;
}

void Controller::start()
{
        stopped = false;
}

void Controller::stop()
{
        stopped = true;
        time_limit = 0;

        vel.linear = 0;
        vel.angular = 0;
}

bool Controller::detectWall()
{
        double min_dist;

        if (lasers.empty())
                return false;

        min_dist = *std::min_element(lasers.begin(), lasers.end());

        if ((min_dist > 50) || (min_dist < 0.5))
                return true;
        else
                return false;
}

bool Controller::onPath()
{
        double y = path.m*pose.x + path.b;

        ROS_INFO("Yt: %.3f, Ye: %.3f", y, pose.y);

        if (fabs(pose.y - y) > tol)
                return false;
        else
                return true;
}

geometry_msgs::Twist Controller::goStraight()
{
        double dx, dy, dl, da;
        geometry_msgs::Twist ret;

        dx = dest.x - pose.x;
        dy = dest.y - pose.y;
        dl = hypot(dx, dy);
        da = remainder(atan2(dy, dx) - pose.theta, 2*M_PI);

        if (fabs(dl) < tol) {
                arrived_ = true;
                stop();
        }
        else if (fabs(da) > tol) {
                vel.linear = 0;
                vel.angular = sgn(da) * vamax;
        }
        else {
                vel.linear = vlmax;
                vel.angular = 0;
        }

        ret.linear.x = vel.linear;
        ret.angular.z = vel.angular;

        return ret;
}

geometry_msgs::Twist Controller::followWall(Controller::Direction dir)
{
        return _followWall((int) dir);
}

geometry_msgs::Twist Controller::update()
{
        geometry_msgs::Twist ret;

        if (stopped)
                return ret;

        if (time_limit == 0) {
                if (wall_following && onPath()) {
                        ROS_INFO("Back on track.");
                        wall_following = false;
                        time_limit = 500;
                }

                if (!wall_following && detectWall()) {
                        ROS_INFO("Wall detected!");
                        wall_following = true;
                        time_limit = 500;
                }
        }
        else {
                time_limit--;
        }

        if (wall_following)
                ret = followWall(Direction::RIGHT);
        else
                ret = goStraight();

        return ret;
}
/*
std::vector<pair<int,int>> frontier_medians(tf::TransformListener &tf_listener)
{
        std::chrono::time_point<std::chrono::system_clock> wfd_time;
        std::vector<pair<int,int>> list_of_medians = wfd(tf_listener);
            //get_medians(list_of_frontiers);
        std::cout << "Obtained list of medians: There were " << list_of_medians.size() << std::endl;
        int secondsElapsed2 = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-wfd_time).count();
        std::cout << "Seconds Elapsed on WFD: " << secondsElapsed2 << std::endl;
            
        for(int i = 0; i<list_of_medians.size(); i++){
        std::cout << "x/y = (" << list_of_medians[i].first << ", " << list_of_medians[i].second << endl;
                occ_grid[list_of_medians[i].first][list_of_medians[i].second] = 50;
        }

        return list_of_medians;
}*/

int main(int argc, char **argv)
{
        using namespace std::chrono;
        using geometry_msgs::Twist;

        int timer = 0;
        time_point<system_clock> start;
        time_point<system_clock> now;

        ros::init(argc, argv, "terrorizing_turtle");

        ros::NodeHandle nh;
        ros::Publisher pub;
        geometry_msgs::PointStamped dest;

        /* Start the clock, 8 minutes.
         */
        start = system_clock::now();

        Controller controller(nh);

        pub = nh.advertise<Twist>("cmd_vel_mux/input/teleop", 1);

        dest.header.frame_id = "/odom";
        dest.header.stamp = ros::Time::now();
        dest.point.x = 5;
        dest.point.y = 0;

        controller.set(dest);
        controller.start();

        ros::Rate rate(20);
        tf::TransformListener tf_listener(nh);


        while (ros::ok() && (timer < 480)) {
                ros::spinOnce();

                pub.publish(controller.update());

                rate.sleep();

                now = system_clock::now();
                timer = duration_cast<seconds>(now-start).count();

                ROS_INFO("Time: %d", timer);
        }

        return 0;
}
