#ifndef GIMBAL_DRIVER_H
#define GIMBAL_DRIVER_H

#include <vector>
#include <ros/ros.h>
#include <dynamixel_workbench_toolbox/dynamixel_multi_driver.h>
#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <cmg_msgs/GimbalTarget.h>

namespace gimbal_driver {

	class GimbalDriver {
		private:
			ros::NodeHandle node;
			ros::Publisher state_pub;
			ros::Subscriber sub_gbset;
			std::vector<int> ids;
			dynamixel_multi_driver::DynamixelMultiDriver multi_driver;
			dynamixel::GroupSyncWrite * modeSyncWrite;
			uint8_t current_mode;

			uint32_t convertRad2Val(float radian);
			int32_t convertRps2Val(float rps);
		public:
			GimbalDriver();
			void publish();
			void set_torque(bool on);
			void set_mode(uint8_t on);
			void set_pos(const cmg_msgs::GimbalTarget::ConstPtr & msg);
	};
}


#endif
