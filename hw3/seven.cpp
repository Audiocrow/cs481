//Blank placeholder file

//Blank placeholder file

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <sstream>
using namespace std;
ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
turtlesim::Pose turtlesim_pose;
const double x_min = 0.0;
const double y_min = 0.0;
const double x_max = 11.0;
const double y_max = 11.0;
const double PI = 3.14159265359;
//to move from one point to specified distance

void move(double speed, double distance, bool isForward) {
	ROS_INFO_STREAM("Moving\n");
	geometry_msgs::Twist vel_msg;
	//set a random linear velocity in the x-axis
	if (isForward)
		vel_msg.linear.x =abs(speed);
	else
		vel_msg.linear.x =-abs(speed); //without changing its orientation
	//ignore other axes since the robot will move in straight line
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
	//set a random angular velocity in the y-axis
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z =0;
	double t0 = ros::Time::now().toSec(); //start time
	double current_distance = 0.0;
	ros::Rate loop_rate(100);
	double t1; //current time

	do {
		velocity_publisher.publish(vel_msg);
		t1 = ros::Time::now().toSec(); //current time
		current_distance = speed * (t1-t0); //distance = speed*delta_time where (t1-t0) is delta_time
		ros::spinOnce();
		loop_rate.sleep();
		//cout<<(t1-t0)<<", "<<current_distance <<", "<<distance<<endl;
	} while(current_distance<distance); //robot will keep moving while this condition is true

	vel_msg.linear.x =0; //to pause the robot making speed to zero
	velocity_publisher.publish(vel_msg);
}

void poseCallback(const turtlesim::Pose::ConstPtr & pose_message){
	turtlesim_pose.x=pose_message->x;
	turtlesim_pose.y=pose_message->y;
	turtlesim_pose.theta=pose_message->theta;
}

void rotate (double angular_speed, double relative_angle, bool clockwise) {
	geometry_msgs::Twist vel_msg;
	vel_msg.linear.x =0;
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
	//angular velocity only in 2D space
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	if (clockwise) //negative velocity
		vel_msg.angular.z =-abs(angular_speed);
	else //counter-clockwise is positive velocity
		vel_msg.angular.z =abs(angular_speed);
	double current_angle = 0.0; //amount of angle rotated
	double t0 = ros::Time::now().toSec(); //setting start time
	double t1; //to set the current time
	ros::Rate loop_rate(10);

	do {
		velocity_publisher.publish(vel_msg);
		t1 = ros::Time::now().toSec();
		current_angle = angular_speed * (t1-t0); // ∆θ = ω *∆t from ω = ∆θ/∆t
		ros::spinOnce();
		loop_rate.sleep();
	} while (current_angle<relative_angle);

	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg); //stop rotating
}

double degrees2radians(double angle_in_degrees){
	return angle_in_degrees *PI /180.0;
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "seven");
	ros::NodeHandle n;
	double speed, angular_speed;
	double distance, angle;
	bool isForward, clockwise;
	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);
	//ros::Rate loop_rate(10);
	ROS_INFO_STREAM("\n\n\n******START MOVING************\n");
	speed = 0.5;
	distance = 2;
	isForward = true;
	angular_speed = 15.0;
	angle = 90.0;
	clockwise = false;
	ros::Rate loop_rate(0.5);
	loop_rate.sleep();
	rotate(degrees2radians(angular_speed), degrees2radians(angle), clockwise);
	move(speed, distance, isForward);
	rotate(degrees2radians(angular_speed), degrees2radians(angle), clockwise);
	distance = 1;
	move(speed, distance, isForward);
}
