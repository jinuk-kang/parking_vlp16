#include "parking_vlp16/parking.h"

void ParkingLidar::initSetup() {
	// ros
	obs_sub_ = nh_.subscribe("/velodyne_points", 1, &ParkingLidar::pointCallback, this );
	gps_sub_ = nh_.subscribe("/parking_info", 1, &ParkingLidar::parkingInfoCallback, this );
	gps_pub_ = nh_.advertise<std_msgs::Bool>("/parking_area", 10);
	marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/parking_points", 10);

	// values
	count_ = 0;
	gps_x_ = 0;
	gps_y_ = 0;
	gps_theta_ = 0;
	angle_ = 0;
	ex_angle_ = 0;

	// flags
	obs_exist_flag_ = false;
	check_flag_ = false;

}

// 0: false 1: true
void ParkingLidar::parkingInfoCallback(const geometry_msgs::Pose2DConstPtr &info){
	gps_x_ = info->x;
	gps_y_ = info->y;
	gps_theta_ = info->theta;

	check_flag_ = true;
}

void ParkingLidar::pointCallback(const sensor_msgs::PointCloud2ConstPtr &input) {

	obstacles_ = Cluster().cluster(input, 0.5, 10, -6 ,0);
	visualize(obstacles_);


/*	if(obs.segments.size() > 0) {
		centerPoint_.x = (obs.segments[0].first_point.x + obs.segments[0].last_point.x) / 2;
		centerPoint_.y = (obs.segments[0].first_point.y + obs.segments[0].last_point.y) / 2;
		//centerPoint_.x = obs.segments[0].first_point.x;
		//centerPoint_.y = obs.segments[0].first_point.y;
		angle_ = calcAngle(centerPoint_);
	
		checkObstacle();
	} else {
		obs_exist_flag_ = false;

		if(check_flag_) {
			std_msgs::Bool obs;
			obs.data = true;
			ros::Duration(2.0).sleep();
			gps_pub_.publish(obs); 
			check_flag_ = false;
		}
	}
	*/
}

void ParkingLidar::checkObstacle() {
	geometry_msgs::Point gps_point;
	
	gps_point.x = centerPoint_.x + 1.3;
	gps_point.y = centerPoint_.y;

	dist_gps_ = getDist(gps_point);
	dist_lidar_ = getDist(centerPoint_);
	ex_angle_ =abs(acos((dist_gps_*dist_gps_ + GAP*GAP - dist_lidar_*dist_lidar_)/(2*GAP*dist_gps_))*180/M_PI);

	if (abs(ex_angle_ - gps_theta_) < 10) {
		// TODO: consider distance
		obs_exist_flag_ = true;

	} else {
		obs_exist_flag_ = false;
	}
	if(check_flag_) {
		if(obs_exist_flag_) {
			std_msgs::Bool obs;
			obs.data = !obs_exist_flag_; //false

			cout << "###############################" << endl;
			cout << "gps_x, gps_y -> " << gps_x_ << ", " << gps_y_ << endl;
			cout << "obs_x, obs_y -> " << centerPoint_.x << ", " << centerPoint_.y << endl;
			cout << "Diff Angle -> " << abs(ex_angle_ - gps_theta_) << endl;
			cout << "Angle from Lidar -> " << angle_ << endl;
			cout << "Angle from GPS -> " << gps_theta_ << endl;
			cout << "Ex_angle -> " << ex_angle_ << endl;
			cout << "###############################" << endl;

			ros::Duration(2.0).sleep();
			gps_pub_.publish(obs); 

			check_flag_ = false;
		}
		else {
			std_msgs::Bool obs;
			obs.data = !obs_exist_flag_; //true

			cout << "###############################" << endl;
			cout << "gps_x, gps_y -> " << gps_x_ << ", " << gps_y_ << endl;
			cout << "obs_x, obs_y -> " << centerPoint_.x << ", " << centerPoint_.y << endl;
			cout << "Diff Angle -> " << abs(ex_angle_ - gps_theta_) << endl;
			cout << "Angle from Lidar -> " << angle_ << endl;
			cout << "Angle from GPS -> " << gps_theta_ << endl;
			cout << "Ex_angle -> " << ex_angle_ << endl;
			cout << "###############################" << endl;

			ros::Duration(2.0).sleep();
			gps_pub_.publish(obs); 
			check_flag_ = false;
			ros::shutdown();
		}	
	}
}

void ParkingLidar::run(){
	if(check_flag_) {
		if(obs_exist_flag_) {
			std_msgs::Bool obs;
			obs.data = !obs_exist_flag_; //false
			gps_pub_.publish(obs); 

			check_flag_ = false;
		}
		else {
			std_msgs::Bool obs;
			obs.data = !obs_exist_flag_; //true
			gps_pub_.publish(obs);
			
			check_flag_ = false;
			ros::shutdown();
		}	
	}
}

void ParkingLidar::visualize(vector<geometry_msgs::Point> input_points){

	visualization_msgs::Marker points;

	points.header.frame_id = "velodyne";
	points.header.stamp = ros::Time::now();
	points.ns = "points_and_lines";
	points.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w = 1.0;
	points.id = 1;
	points.type = visualization_msgs::Marker::POINTS;
	points.scale.x = 0.3;
	points.scale.y = 0.3;
	points.color.a = 1.0;
	points.color.g = 1.0f;

	geometry_msgs::Point p;

	for (auto point : input_points) {
		p.x = point.x;
		p.y = point.y;
		p.z = point.z;
		points.points.push_back(p);
	}

	marker_pub_.publish(points);

}


int main(int argc, char** argv) {

	ros::init(argc, argv, "parking_lidar");
	ParkingLidar pl;
	ros::spin();
}

