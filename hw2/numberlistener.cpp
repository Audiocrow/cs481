#include <exception>
#include <set>
#include <ros/ros.h>
#include <sstream>
#include <std_msgs/String.h>

void numberCallback(const std_msgs::String::ConstPtr& msg)
{
	std::set<int> unique;
	int nums[5];
	std::stringstream ss(msg->data.c_str());
	try {
		//ROS_INFO("The string I received is:%s", ss.str().c_str());
		for(int i=0; i<5; i++) {
			ss >> nums[i];
			unique.insert(nums[i]);
		}
		int sum=0;
		for(std::set<int>::iterator it = unique.begin(); it != unique.end(); ++it)
			sum+=*it;
		ROS_INFO("The sum of unique numbers received is:%d", sum);
		
	}
	catch(std::ios_base::failure e) {
		ROS_INFO("An invalid string of numbers was obtained:\n\tError:%s\n\tString:%s", e.what(), msg->data.c_str());
	}
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "numberlistener");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("hw2numbers", 10, numberCallback);
	ros::spin();
	return 0;
}
