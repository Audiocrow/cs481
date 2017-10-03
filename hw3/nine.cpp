#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "turtlesim/Pose.h"

const double PI = 3.14159265359;

class NinePose {
public:
	turtlesim::Pose pose;
	void PoseCallback(const turtlesim::Pose::ConstPtr& msg) {
		//std::cout << "DEBUG: MESSAGE CALLBACK!!!!" << std::endl;
		pose.x = msg->x;
		pose.y = msg->y;
		pose.theta = msg->theta;
	}
	double getX() { return pose.x; }
	double getY() { return pose.y; }
	double getTheta() { return pose.theta; }
};

//Moves the turtle in the direction its facing at given speed by given distance
//Uses the subscription to the turtle to determine its position and goal
void move(NinePose& pose, ros::Publisher& pub, double speed, double distance) {
	geometry_msgs::Twist velMsg;
	velMsg.linear.x = speed;
	velMsg.linear.y = 0;
	velMsg.linear.z = 0;
	velMsg.angular.x = 0;
	velMsg.angular.y = 0;
	velMsg.angular.z = 0;
	double goal = pose.getX() + distance*cos(pose.getTheta());
	ros::Rate rate(1); //turtlesim moves for 1 second when receiving a message
	do {
		pub.publish(velMsg);
		ros::spinOnce();
		rate.sleep();
		//Fix overshooting
		if(goal - pose.getX() > speed)
			velMsg.linear.x = goal - pose.getX();
	} while(ros::ok() && pose.getX() < goal);
	velMsg.linear.x = 0;
	pub.publish(velMsg);
	ros::spinOnce();
	rate.sleep();
}

//Rotates the turtle by the given relative amount
//Uses the subscription to the turtle
void rotate(ros::Publisher& pub, double speed, double angle, bool clockwise) {
	geometry_msgs::Twist velMsg;
	velMsg.linear.x = 0;
	velMsg.linear.y = 0;
	velMsg.linear.z = 0;
	velMsg.angular.x = 0;
	velMsg.angular.y = 0;
	velMsg.angular.z = clockwise ? -fabs(speed) : fabs(speed);
	double t0 = ros::Time::now().toSec(), t1;
	double angleRotated = 0.0;
	//ros::Rate rate(100);
	while(ros::ok() && angleRotated < angle) {
		pub.publish(velMsg);
		ros::spinOnce();
		t1 = ros::Time::now().toSec();
		angleRotated = speed * (t1-t0);
		/*if(abs(speed) > abs(angle-angleRotated)) {
			speed = angle-angleRotated;
			velMsg.angular.z = clockwise ? -abs(speed) : abs(speed);
		}*/
	}
	velMsg.angular.z = 0;
	pub.publish(velMsg);
	ros::spinOnce();
	//rate.sleep();
}

//Code from Intro to ROS pdf
double degrees2radians(double angle_in_degrees)
{
	return angle_in_degrees*PI /180.0;
}

//Slightly modified code from Intro to ROS pdf
void setDesiredOrientation (NinePose& pose, ros::Publisher& pub, double desired_angle_radians) {
	double relative_angle_radians = desired_angle_radians - pose.getTheta();
	bool clockwise = ((relative_angle_radians<0)?true:false);
	rotate(pub, degrees2radians(10), abs(relative_angle_radians), clockwise);
}

using namespace std;

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "nine");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	NinePose pose;
	ros::Subscriber sub = nh.subscribe("/turtle1/pose", 10, &NinePose::PoseCallback, &pose);
	ros::spinOnce();
	while(ros::ok()) {
		cout << "DEBUG: TURNING 180 deg...\n";
		rotate(pub, degrees2radians(40), degrees2radians(180), false);
		cout << "DEBUG: MOVING 5...\n";
		move(pose, pub, 0.5, 5);
		cout << "DEBUG: ROTATING 20 deg...\n";
		rotate(pub, degrees2radians(5), degrees2radians(20), true);
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
		while(ros::ok() && ((pose.getX() <= currX - 1 && pose.getX() >= currX + 1) || pose.getY() <= currY)) {
			pub.publish(vel_msg);
			ros::spinOnce();
			circleRate.sleep();
		}
		vel_msg.angular.z = 0;
		vel_msg.linear.x = 0;
		pub.publish(vel_msg);
		ros::spinOnce();
		break;
	}
	return 0;
}
