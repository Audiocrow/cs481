#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "turtlesim/Pose.h"

const double PI = 3.14159265359;

class NineTurtle {
private:
	turtlesim::Pose pose;
public:
	void PoseCallback(const turtlesim::Pose::ConstPtr& msg) {
		pose.x = msg->x;
		pose.y = msg->y;
		pose.theta = msg->theta;
	}
	double getX() { return pose.x; }
	double getY() { return pose.y; }
	double getTheta() { return pose.theta; }
};

double degrees2radians(int angle) {
	return (PI*angle)/180.0;
}

//Moves the turtle in the direction its currently facing by distance
//Goal-finds a tad bit to correct its path to the inital, immutable goal
//To an error of 1%
void move(NineTurtle& turtle, ros::Publisher& pub, double speed, double distance) {
	geometry_msgs::Twist msg;
	msg.linear.y = 0;
	msg.linear.z = 0;
	msg.angular.x = 0;
	msg.angular.y = 0;
	msg.angular.z = 0;
	msg.linear.x = speed;
	double goalX = turtle.getX() + distance*cos(turtle.getTheta());
	double goalY = turtle.getY() + distance*sin(turtle.getTheta());
	double distRemaining = sqrt(pow(turtle.getX() - goalX, 2) + pow(turtle.getY() - goalY, 2));
	std::cout << "Moving from (X,Y) = (" << turtle.getX() << "," << turtle.getY() << ") to ("
		<< goalX << "," << goalY << ")" << std::endl;
	ros::Rate rate(10);
	do {
		//Slight angular adjustment/error-correction
		msg.angular.z = 1 * (atan2(goalY - turtle.getY(), goalX - turtle.getX()) - turtle.getTheta());
		pub.publish(msg);
		ros::spinOnce();
		rate.sleep();
		distRemaining = sqrt(pow(turtle.getX() - goalX, 2) + pow(turtle.getY() - goalY, 2));
		//If the turtle would overshoot, slow down its speed
		if(fabs(distRemaining) < fabs(msg.linear.x))
			msg.linear.x = distRemaining;
		else if(fabs(distRemaining) >= fabs(speed))
			msg.linear.x = speed;
	} while(ros::ok() && distRemaining > 0.01);
	msg.linear.x = 0;
	msg.angular.z = 0;
	pub.publish(msg);
	rate.sleep();
	std::cout << "Ended movement at (X,Y) = (" << turtle.getX() << "," << turtle.getY() << ")" << std::endl;
}

//Rotates the turtle by the desired amount in the desired direction
//Will correct for over or under-rotating to an error of 0.1%
void rotate(NineTurtle& turtle, ros::Publisher& pub, double speed, double angle, bool clockwise=false) {
	geometry_msgs::Twist msg;
	msg.linear.x = 0;
	msg.linear.y = 0;
	msg.linear.z = 0;
	msg.angular.x = 0;
	msg.angular.y = 0;
	msg.angular.z = clockwise ? -fabs(speed) : fabs(speed);
	//double goalTheta = clockwise ? turtle.getTheta() - angle : turtle.getTheta() + angle;
	double goalTheta = turtle.getTheta() + angle;
	if(goalTheta > 2*PI) goalTheta = goalTheta - 2*PI;
	if(goalTheta < -2*PI) goalTheta = goalTheta + 2*PI;
	std::cout << "DEBUG: TurtleTheta=" << turtle.getTheta() << "; angle=" << angle << (clockwise ? " clockwise" : " counter-clockwise");
	std::cout << "; goalTheta=" << goalTheta << std::endl;
	ros::Rate rate(10);
	do {
		pub.publish(msg);
		ros::spinOnce();
		rate.sleep();
		//Fix missing the goal by slowing down the speed and maybe fixing the direction of rotation
		if(fabs(goalTheta - turtle.getTheta()) < fabs(msg.angular.z))
			msg.angular.z = (goalTheta - turtle.getTheta());
	} while(ros::ok() && fabs(turtle.getTheta() - goalTheta) > 0.001);
	msg.angular.z = 0;
	pub.publish(msg);
	rate.sleep();
}

void setDesiredOrientation(NineTurtle& turtle, ros::Publisher& pub, double angle) {
	bool clockwise = (angle-turtle.getTheta()) < 0 ? true: false;
	rotate (turtle, pub, degrees2radians(45), fabs(angle-turtle.getTheta()), clockwise);
}

using namespace std;

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "nine");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	NineTurtle myTurtle;
	ros::Subscriber sub = nh.subscribe("/turtle1/pose", 10, &NineTurtle::PoseCallback, &myTurtle);
	ros::spinOnce();
	geometry_msgs::Twist msg;
	ros::Rate rate(10);
	double t0;
	while(ros::ok()) {
		setDesiredOrientation(myTurtle, pub, degrees2radians(90));
		move(myTurtle, pub, 0.5, 3);
		rotate(myTurtle, pub, degrees2radians(45), degrees2radians(45));
		move(myTurtle, pub, 0.5, 0.7);
		//Move the turtle in a semi circle
		//Strategy: 360 degrees at a speed of 20 deg/sec means ~18 seconds of movement
		msg.linear.x = 0.25; //Adjust until circle is desired size
		msg.linear.y = 0;
		msg.linear.z = 0;
		msg.angular.x = 0;
		msg.angular.y = 0;
		msg.angular.z = degrees2radians(20);
		t0 = ros::Time::now().toSec();
		do {
			pub.publish(msg);
			ros::spinOnce();
			rate.sleep();
		} while(ros::ok() && ros::Time::now().toSec() - t0 < 18);
		msg.angular.z = 0;
		msg.linear.x = 0;
		pub.publish(msg);
		ros::spinOnce();
		rate.sleep();
		break;
	}
	return 0;
}
