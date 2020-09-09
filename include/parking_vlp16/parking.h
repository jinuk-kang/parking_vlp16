#include "ros/ros.h"
#include "iostream"
#include "cmath"
#include "math.h"

#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Pose2D.h"

#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include "clustering.cpp"

#define GAP 1.3
#define DIFF_ANGLE 0
#define _USE_MATH_DEFINES

using namespace std;

class ParkingLidar {
private:
	// ros
	ros::NodeHandle nh_;
	ros::Subscriber obs_sub_;
	ros::Subscriber gps_sub_;
	ros::Publisher gps_pub_;

	// msgs
	geometry_msgs::Point nearestPoint_;
	geometry_msgs::Point centerPoint_;
	// value
	int count_;

	double gps_x_;
	double gps_y_;
	double gps_theta_; // degree
	double angle_; // degree
	double dist_gps_;
	double dist_lidar_;
	double ex_angle_;

	// flags
	bool obs_exist_flag_;
	bool check_flag_;

public:
	void initSetup();
	void pointCallback(const sensor_msgs::PointCloud2ConstPtr &obstacle);
	void parkingInfoCallback(const geometry_msgs::Pose2DConstPtr &info);

	void run();
	void printCurrentState();
	void checkObstacle();

	double calcAngle(geometry_msgs::Point p){ return -atan(p.y/p.x)/M_PI*180; } 
	double getDist(geometry_msgs::Point p) {return sqrt(p.x*p.x + p.y*p.y); }	
	double getDist(double x, double y){ return sqrt(x*x + y*y); }

	ParkingLidar() { initSetup(); }
};

