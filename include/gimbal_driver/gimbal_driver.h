#ifndef GIMBAL_DRIVER_H
#define GIMBAL_DRIVER_H

#include <vector>
#include <ros/ros.h>
#include <dynamixel_workbench_toolbox/dynamixel_multi_driver.h>
#include <dynamixel_workbench_msgs/DynamixelStateList.h>

namespace gimbal_driver {

	class GimbalDriver {
		private:
			ros::NodeHandle node;
			ros::Publisher state_pub;
			ros::Subscriber sub_gbset;
			std::vector<int> ids;
			dynamixel_multi_driver::DynamixelMultiDriver multi_driver;
		public:
			GimbalDriver();
			void publish();
	};
}


#endif
