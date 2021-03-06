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
	int t_wait = 1;
	while (!multi_driver.loadDynamixel(infos) && ros::ok()) {
		ROS_ERROR("Can't load Dynamixels. Trying again in %d sec.",t_wait);
		ros::Duration(t_wait).sleep();
		t_wait++;
	}
	ROS_INFO("-----------------------------------");
	ROS_INFO("        Gimbal Driver Infos        ");
	ROS_INFO("-----------------------------------");
	for (auto info: infos) {
		ROS_INFO("%d : %s", info->model_id, info->model_name.c_str());
	}
	ROS_INFO("-----------------------------------");

	for (dynamixel_driver::DynamixelInfo * info: infos) {
		delete info;
	}

	multi_driver.initSyncWrite();
	state_pub = node.advertise<cmg_msgs::DynamixelStateList>("/gimbal/state", 10);
	sub_gbset = node.subscribe("/gimbal/cmd", 3, &GimbalDriver::set_pos, this);
	modeSyncWrite = multi_driver.setSyncWrite("operating_mode");
	current_mode = 12;
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

float GimbalDriver::convertVal2Rad(uint32_t val)
{
	float min_radian = multi_driver.multi_dynamixel_[0]->min_radian_;
	float max_radian = multi_driver.multi_dynamixel_[0]->max_radian_;
	uint32_t v_zero = multi_driver.multi_dynamixel_[0]->value_of_0_radian_position_;
	uint32_t v_min = multi_driver.multi_dynamixel_[0]->value_of_min_radian_position_;
	uint32_t v_max = multi_driver.multi_dynamixel_[0]->value_of_max_radian_position_;
	uint32_t v_2pi = (v_max - v_min)*(2*M_PI / (max_radian - min_radian));
	while (val > v_max) { val -= v_2pi; }
        while (val < v_min) { val += v_2pi; }
	float radians = min_radian + (max_radian - min_radian) * (val-v_min) / (v_max - v_min);
	return radians;
}

int32_t GimbalDriver::convertRps2Val(float rps)
{
	float k = multi_driver.multi_dynamixel_[0]->velocity_to_value_ratio_;
	return (int32_t) (rps*k);
}

float GimbalDriver::convertVal2Rps(int32_t val)
{
	float k = multi_driver.multi_dynamixel_[0]->velocity_to_value_ratio_;
	return val/k;
}


void GimbalDriver::set_pos(const cmg_msgs::GimbalTarget::ConstPtr & msg) {
	if (msg->mode == 0) {
		set_mode(3);
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
		} else {
			ROS_INFO("Gimbal set position ok");
		}
	} else {
		set_mode(1);
		std::vector<int32_t> pos;
		for (float p : msg->positions) {
			pos.push_back(convertRps2Val(p));
			ROS_INFO("%lu velocity -> %ud (%f)", pos.size(), pos[pos.size()-1], p);
		}
		while (pos.size() < multi_driver.multi_dynamixel_.size()) {
			pos.push_back(convertRps2Val(0.));
		}
		if (!multi_driver.syncWriteVelocity(pos)) {
			ROS_ERROR("Gimbal set velocity failed");
			for (int32_t p : pos) {
				ROS_ERROR("%ud",p);
			}
		} else {
			ROS_INFO("Gimbal set velocity ok");
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
	} else {
		ROS_INFO("Gimbal set torque to %s OK", on?"ON":"OFF");
	}
}

void GimbalDriver::set_mode(uint8_t mode) {
	if (mode == current_mode) { return; }
	set_torque(false);
	for (int id : ids) {
		modeSyncWrite->addParam((uint8_t)id, &mode);
	}
	int res = modeSyncWrite->txPacket();
	if (res != COMM_SUCCESS) {
		ROS_ERROR("Mode change error");
	} else {
		current_mode = mode;
		ROS_INFO("Set mode = %d", mode);
	}
	modeSyncWrite->clearParam();
	set_torque(true);
	ros::Duration(0.1).sleep();
}

void GimbalDriver::publish() {
	multi_driver.readMultiRegister("present_position");
	//multi_driver.readMultiRegister("goal_position");
	multi_driver.readMultiRegister("present_velocity");
	//multi_driver.readMultiRegister("goal_velocity");
	multi_driver.readMultiRegister("present_temperature");
	cmg_msgs::DynamixelStateList pub_msg;
	int size = ids.size();
	for (int i = 0; i < size; i++) {
		cmg_msgs::DynamixelState state;
		state.present_position	= convertVal2Rad(multi_driver.read_value_["present_position"]->at(i));
		//state.goal_position	= convertVal2Rad(multi_driver.read_value_["goal_position"]->at(i));
		state.present_velocity	= convertVal2Rps(multi_driver.read_value_["present_velocity"]->at(i));
		//state.goal_velocity	= convertVal2Rps(multi_driver.read_value_["goal_velocity"]->at(i));
		state.present_temperature = multi_driver.read_value_["present_temperature"]->at(i);
		pub_msg.states.push_back(state);
	}
	pub_msg.header.stamp = ros::Time::now();
	state_pub.publish(pub_msg);
}

int main(int argc, char * argv[]) {
	ros::init(argc, argv, "gimbal_driver");
	GimbalDriver gd;
	gd.set_torque(true);
	int rate; ros::param::param<int>("~gimbal_pub_freq", rate,  1);
	if (rate > 50) {
		while (ros::ok()) {
			gd.publish();
			ros::spinOnce();
		}

	} else {
		ros::Rate pub_rate(2*rate);
		bool pub = true;
		while (ros::ok()) {
			if(pub) {
				gd.publish();
			} else {
				ros::spinOnce();
			}
			pub = !pub;
			pub_rate.sleep();
		}
	}
	gd.set_torque(false);
}
