#include "gimbal_driver/gimbal_driver.h"
#include <string>
#include <cmath>

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

	multi_driver.initSyncWrite();
	state_pub = node.advertise<dynamixel_workbench_msgs::DynamixelStateList>("gimbal_state", 10);
	sub_gbset = node.subscribe("gimbal_set_position", 10, &GimbalDriver::set_pos, this);
}


uint32_t GimbalDriver::convertRad2Val(float radian)
{
	float min_radian = multi_driver.multi_dynamixel_[0]->min_radian_;
	float max_radian = multi_driver.multi_dynamixel_[0]->max_radian_;
	uint32_t v_zero = multi_driver.multi_dynamixel_[0]->value_of_0_radian_position_;
	uint32_t v_min = multi_driver.multi_dynamixel_[0]->value_of_min_radian_position_;
	uint32_t v_max = multi_driver.multi_dynamixel_[0]->value_of_max_radian_position_;

	while (radian > max_radian) { radian -= 2*M_PI; }
	while (radian < min_radian) { radian += 2*M_PI; }

	//ROS_INFO("Values: %d(%f rad), %d(0 rad) , %d(%f rad)",v_min, min_radian, v_zero, v_max, max_radian);
	if (radian > 0) {
		if (v_max <= v_zero)
			return v_max;
		return v_zero + (radian/max_radian * (v_max - v_zero));
	} else if (radian < 0) {
		if (v_min >= v_zero)
			return v_min;
		return v_zero - (radian/min_radian * (v_zero - v_min));
	} 
	return v_zero;
}


void GimbalDriver::set_pos(const cmg_msgs::GimbalPositions::ConstPtr & msg) {
	std::vector<uint32_t> pos;
	for (float p : msg->positions) {
		pos.push_back(convertRad2Val(p));
	}
	while (pos.size() < multi_driver.multi_dynamixel_.size()) {
		pos.push_back(convertRad2Val(0.));
	}
	if (!multi_driver.syncWritePosition(pos)) {
		ROS_ERROR("Gimbal set position failed");
		for (uint32_t p : pos) {
			ROS_ERROR("%ud",p);
		}
	}
}

void GimbalDriver::set_torque(bool on) {
	std::vector<uint8_t> trs;
	while (trs.size() < multi_driver.multi_dynamixel_.size()) {
		trs.push_back(on);
	}
	if (!multi_driver.syncWriteTorque(trs)) {
		ROS_ERROR("Gimbal set torque failed");
	}
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
	gd.set_torque(true);
	int rate; ros::param::param<int>("~gimbal_pub_freq", rate,  1);
	ros::Rate pub_rate(rate);
	while (ros::ok()) {
		gd.publish();
		ros::spinOnce();
		pub_rate.sleep();
	}
	gd.set_torque(false);
}
