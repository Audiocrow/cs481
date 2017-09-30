#include <exception>
#include <ros/ros.h>
#include <sstream>
#include <std_msgs/String.h>

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "numberpublisher");
	ros::NodeHandle nh;
	ros::Publisher numpub = nh.advertise<std_msgs::String>("hw2numbers", 10);
	ros::Rate loop_rate(10);
	while(ros::ok()) {
		std::cout << "Please enter 5 integers which are between 0-100, seperated by a space:" << std::endl;
		int nums[5];
		try {
			std::cin >> nums[0] >> nums[1] >> nums[2] >> nums[3] >> nums[4];
			//Validate the entered numbers
			for(int i=0; i<5; i++) {
				if(nums[i] < 0 || nums[i] > 100) {
					throw "Invalid input.";
				}
			}
			std::cin.ignore();
			std_msgs::String msg;
			std::stringstream ss;
			for(int i=0; i<5; i++)
				ss << nums[i] << " ";
			msg.data = ss.str();
			numpub.publish(msg);
			ros::spinOnce();
			loop_rate.sleep();
		}
		catch(std::ios_base::failure e) {
			std::cout << "Invalid input:" << e.what() << std::endl;
			//Clear any garbage in the cin buffer that may have been leftover by invalid input
			std::cin.clear();
			std::cin.ignore(1024, '\n');
		}
		catch(const char* e) {
			std::cout << e << std::endl;
			//Clear any garbage in the cin buffer that may have been leftover by invalid input
			std::cin.clear();
			std::cin.ignore(1024, '\n');
		}
	}
	return 0;
}
