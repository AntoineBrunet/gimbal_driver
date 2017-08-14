#include "gimbal_driver/gimbal_driver.h"
#include <string>

using namespace gimbal_driver;

GimbalDriver::GimbalDriver() :
	node("~"),
	multi_driver(
		node.param<std::string>("gimbal_dev","/dev/ttyUSB0"), 
		node.param<int>("gimbal_baud", 57600),
		node.param<float>("gimbal_protocol_version", 2.0)
	) {
	std::string device;
	int baud;
	float protocol_version;

	node.param<float>("gimbal_protocol_version", protocol_version, 2.0);
	node.param<std::string>("gimbal_dev", device, "/dev/ttyUSB0");
	node.param("gimbal_baud", baud, 57600);

	node.getParam("gimbal_ids", ids);
	std::vector<dynamixel_driver::DynamixelInfo*> infos;
	for (int i: ids) {
		ROS_INFO("Init gimbal #%d info", i);
		dynamixel_driver::DynamixelInfo *info = new dynamixel_driver::DynamixelInfo;

		info->lode_info.device_name = device;
		info->lode_info.baud_rate = baud;
		info->lode_info.protocol_version = protocol_version;

		info->model_id = i;

		infos.push_back(info);
	}
	if (multi_driver.loadDynamixel(infos)) {
		ROS_INFO("-----------------------------------");
		ROS_INFO("        Gimbal Driver Infos        ");
		ROS_INFO("-----------------------------------");
		for (auto info: infos) {
			ROS_INFO("%d : %s", info->model_id, info->model_name.c_str());
		}
		ROS_INFO("-----------------------------------");
	} else {
		ROS_ERROR("Can't load Dynamixels.");
	}

	for (dynamixel_driver::DynamixelInfo * info: infos) {
		delete info;
	}

	state_pub = node.advertise<dynamixel_workbench_msgs::DynamixelStateList>("gimbal_state", 10);
}

void GimbalDriver::publish() {
	int size = ids.size();
	multi_driver.readMultiRegister("torque_enable");
	multi_driver.readMultiRegister("moving");
	multi_driver.readMultiRegister("present_position");
	multi_driver.readMultiRegister("goal_position");
	multi_driver.readMultiRegister("present_velocity");
	multi_driver.readMultiRegister("goal_velocity");
	multi_driver.readMultiRegister("present_current");
	multi_driver.readMultiRegister("goal_current");
	dynamixel_workbench_msgs::DynamixelStateList pub_msg;
	for (int i = 0; i < size; i++) {
		dynamixel_workbench_msgs::DynamixelState state;
		state.model_name		= multi_driver.multi_dynamixel_[i]->model_name_;
		state.id				= multi_driver.multi_dynamixel_[i]->id_;
		state.torque_enable		= multi_driver.read_value_["torque_enable"]->at(i);
		state.moving			= multi_driver.read_value_["moving"]->at(i);
		state.present_position	= multi_driver.read_value_["present_position"]->at(i);
		state.goal_position		= multi_driver.read_value_["goal_position"]->at(i);
		state.present_velocity	= multi_driver.read_value_["present_velocity"]->at(i);
		state.goal_velocity		= multi_driver.read_value_["goal_velocity"]->at(i);
		state.present_current	= multi_driver.read_value_["present_current"]->at(i);
		state.goal_current		= multi_driver.read_value_["goal_current"]->at(i);
		pub_msg.dynamixel_state.push_back(state);
	}
	state_pub.publish(pub_msg);
}

int main(int argc, char * argv[]) {
	ros::init(argc, argv, "gimbal_driver");
	GimbalDriver gd;
	int rate; ros::param::param<int>("~gimbal_pub_freq", rate,  1);
	ros::Rate pub_rate(rate);
	while (ros::ok()) {
		gd.publish();
		ros::spinOnce();
		pub_rate.sleep();
	}
}
