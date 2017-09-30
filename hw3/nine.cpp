#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "turtlesim/Pose.h"

const double PI = 3.14159265359;

using namespace std;

//Slightly modified professor's code from Intro to ROS pdf
void move(ros::Publisher& velocity_publisher, double speed, double distance, bool isForward)
{
	geometry_msgs::Twist vel_msg;
	if (isForward)
		vel_msg.linear.x =abs(speed);
	else
		vel_msg.linear.x =-abs(speed);
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
	} while(current_distance<distance); //robot will keep moving while this condition is true
	vel_msg.linear.x =0; //to pause the robot making speed to zero
	velocity_publisher.publish(vel_msg);
}
//Slightly modified Professor's code from Intro to ROS pdf
void rotate(ros::Publisher& velocity_publisher, double angular_speed, double relative_angle, bool clockwise)
{
	geometry_msgs::Twist vel_msg;
	vel_msg.linear.x =0;
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
	//angular velocity only in 2D space
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	if (clockwise) //negative velocity
		vel_msg.angular.z = -abs(angular_speed);
	else //counter-clockwise is positive velocity
		vel_msg.angular.z = abs(angular_speed);
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
//Professor's code from Intro to ROS pdf
double degrees2radians(double angle_in_degrees)
{
	return angle_in_degrees *PI /180.0;
}

class NinePose {
public:
	turtlesim::Pose pose;
	void PoseCallback(const turtlesim::Pose::ConstPtr& msg) {
		pose.x = msg->x;
		pose.y = msg->y;
		pose.theta = msg->theta;
	}
	double getX() { return pose.x; }
	double getY() { return pose.y; }
	double getTheta() { return pose.theta; }
};

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "nine");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
	NinePose pose;
	ros::Subscriber sub = nh.subscribe("/turtle1/pose", 10, &NinePose::PoseCallback, &pose);
	ros::Rate rate(0.5);
	rotate(pub, 1.0, degrees2radians(180), false);
	rate.sleep();
	move(pub, 0.5, 10, true);
	rate.sleep();
	rotate(pub, 1.0, degrees2radians(20), false);
	rate.sleep();
	//Code to specifically draw almost a full circle:
	//Strategy: check the turtle's current X and Y, and move+rotate until the X ends up the same but the Y is lower
	cout << "DEBUG: STARTING CIRCLE...\n";
	geometry_msgs::Twist vel_msg;
	vel_msg.linear.x = 0.5;
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;
	vel_msg.angular.z = 0.5;
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	ros::Rate circleRate = 100;
	double currX = pose.getX();
	double currY = pose.getY();
	while((pose.getX() <= currX - 1 && pose.getX() >= currX + 1) || pose.getY() <= currY) {
		pub.publish(vel_msg);
		ros::spinOnce();
		circleRate.sleep();
	}
	vel_msg.angular.z = 0;
	vel_msg.linear.x = 0;
	pub.publish(vel_msg);
	rate.sleep();
	ros::spin();
	return 0;
}
