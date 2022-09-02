/*
 * chassis_comm.h
 * Copyright (C) 2022 screw <screw@xavier-NX>
 *
 * Distributed under terms of the MIT license.
 */

#ifndef CHASSIS_COMM_H
#define CHASSIS_COMM_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <time.h>
#include <string.h>
#include <stdio.h>
#include <string>
#include "serial/serial.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Range.h"
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <cmath>

/**
 * @brief chassis serial communication.
 */
class chassis_commu{
public:

	chassis_commu(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
	~chassis_commu();
	void openSerial();
	uint8_t writeRequ(std::string &requ);
	uint8_t readResp(std::string &buf, size_t size);
	uint8_t exeCmd(std::string &cmd, std::string &result, size_t result_size);
	uint8_t getBaud();
	uint8_t resetEnc();
	uint8_t getEnc();
	uint8_t cfgPID();
	uint8_t getSonar();
	void setVelocity(float l_vel,float r_vel);
	void chassisVelCallback(const geometry_msgs::Twist &vel_msg);
	void poll();

private:
	ros::Subscriber vel_sub_;
	ros::Publisher odom_pub_;
	tf::TransformBroadcaster odom_bc_;

	ros::Publisher front_left_sonar_pub_;
	ros::Publisher front_midd_sonar_pub_;
	ros::Publisher front_right_sonar_pub_;
	ros::Publisher back_midd_sonar_pub_;
	
	boost::mutex serial_mutex_;  

	serial::Serial serial_;
	std::string port_;
	int baud_rate_;
	int time_out_;

	int pid_freq_;
	float pid_cycle_;

	long curr_l_enc_;
	long curr_r_enc_;
	
	long last_l_enc_;
	long last_r_enc_;
	
	long diff_l_enc_;
	long diff_r_enc_;

	std::string Kp_;
	std::string Ki_;
	std::string Kd_;
	std::string Ko_;
	
	uint16_t front_left_sonar_;
	uint16_t front_midd_sonar_;
	uint16_t front_right_sonar_;
	uint16_t back_midd_sonar_;
	
	double chassis_wheel_diameter_;
	double chassis_wheel_track_;
	int chassis_encoder_resolution_;
	double chassis_gear_reduction_;
	double chassis_enc_per_meter_;

	double chassis_x_;
	double chassis_y_;
	double chassis_theta_;
	
	ros::Time curr_tm_;
	ros::Time last_tm_;
	ros::Time call_tm_;
};

#endif /* !CHASSIS_COMM_H */
