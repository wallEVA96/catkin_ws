/*
 * mickx4_velocity.cpp
 *                        _ _                 
 *         __      ____ _| | | _____   ____ _ 
 *         \ \ /\ / / _` | | |/ _ \ \ / / _` |
 *          \ V  V / (_| | | |  __/\ V / (_| |
 *           \_/\_/ \__,_|_|_|\___| \_/ \__,_|
 * Copyright (C) 2020 north china.
 *
 * Distributed under terms of the MIT license.
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include "serial/serial.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"

/**
 * @brief robot_info struct
 */
struct robot_info{
	int32_t left_front_encoder;
	int32_t right_front_encoder;
	int32_t left_back_encoder;
	int32_t right_back_encoder;
	uint8_t battery_percent;
	uint16_t battery_voltage;
} mickx4_info = {0, 0, 0, 0, 0, 0};

/**
 * @brief parse_serial_response
 *
 * @param res
 * @param i
 *
 * @return 1 means succ, 0 means fail.
 */
int parse_serial_response(uint8_t *res, size_t size)
{
	if(res[0] != 0xFE || res[1] != 0xEF || res[2] != 0x13)
		return 0;

	uint8_t check_sum = 0;
	for(uint8_t i = 0; i < (size - 1); i++)
		check_sum += res[i];
	if(res[size-1] != check_sum)
		return 0;
	
	mickx4_info.right_front_encoder = (res[3]<<24)+(res[4]<<16)+(res[5]<<8)+res[6];
	mickx4_info.left_front_encoder = (res[7]<<24)+(res[8]<<16)+(res[9]<<8)+res[10];
	mickx4_info.right_back_encoder = (res[11]<<24)+(res[12]<<16)+(res[13]<<8)+res[14];
	mickx4_info.left_back_encoder = (res[15]<<24)+(res[16]<<16)+(res[17]<<8)+res[18];
	mickx4_info.battery_percent = res[19];
	mickx4_info.battery_voltage = ((res[20]<<8)+res[21])*0.1;
/*
	std::cout << "mickx4 robot info:" << std::endl;
	std::cout << "           left_front_encoder:" << std::dec << mickx4_info.left_front_encoder << std::endl;
	std::cout << "           right_front_encoder:" << std::dec << mickx4_info.right_front_encoder << std::endl;
	std::cout << "           left_back_encoder:" << std::dec << mickx4_info.left_back_encoder << std::endl;
	std::cout << "           right_back_encoder:" << std::dec << mickx4_info.right_back_encoder << std::endl;
	std::cout << "           battery_percent:" << static_cast<int>(mickx4_info.battery_percent) << std::endl;
	std::cout << "           battery_voltage:" << std::dec << mickx4_info.battery_voltage << std::endl;
*/
}

/**
 * @brief mickx4_ctrl_sub
 * @param msg
 */
#define WHEEL_DIAMETER 0.154
#define WHEEL_PERIMETER (WHEEL_DIAMETER * M_PI) 
const double mickx4_width = 0.284/0.622222; // m
geometry_msgs::Twist mickx4_ctrl_msg;
void mickx4_ctrl_sub(const geometry_msgs::Twist &msg)
{
	mickx4_ctrl_msg = msg;
/*  ROS_INFO("I heard raw msg: ");
	ROS_INFO("        linear.x:  %lf", mickx4_ctrl_msg.linear.x);
	ROS_INFO("        linear.y:  %lf", mickx4_ctrl_msg.linear.y);
	ROS_INFO("        linear.z:  %lf", mickx4_ctrl_msg.linear.z);
	ROS_INFO("        angular.x:  %lf", mickx4_ctrl_msg.angular.x);
	ROS_INFO("        angular.y:  %lf", mickx4_ctrl_msg.angular.y);
	ROS_INFO("        angular.z:  %lf", mickx4_ctrl_msg.angular.z);
*/
}

/**
 * @brief communicate with mickx4.
 *
 * @param msg
 * @param mickx4_serial
 */
void comm_with_mickx4(const geometry_msgs::Twist &msg, serial::Serial &mickx4_serial)
{
	uint8_t mickx4_array[12] = {0};
	mickx4_array[0] = 0xFE; // frame head 1
	mickx4_array[1] = 0xEF; // frame head 2
	mickx4_array[2] = 0x08;  // data length
/*
 * y^
 *  |     /
 *  |    /
 *  |   / (theta)
 *  |   -------
 *  0 --------------------> x
 * */
	const static double WHEEL_RATIO = 6480.0/3960.0; //6480 6560 6840
	double vel_x = 0.0, vel_theta = 0.0;
	double vel_right = 0.0, vel_left = 0.0;
	
	vel_x = msg.linear.x; // m/s
	vel_theta = msg.angular.z;

	if(vel_theta == 0)
		vel_left = vel_right = vel_x;
	else if(vel_x == 0){
		vel_right = vel_theta * mickx4_width / 2.0;
		vel_left = -vel_right;
	}
	else
	{
		vel_left = vel_x - 0.5 * vel_theta * mickx4_width;
		vel_right = vel_x + 0.5 * vel_theta * mickx4_width;
	}
	/* Ransfer velocity to RPM.
	 * MAX Velocity is 0.48 m/s.
	 */
	
	if(vel_left > 0.48 || vel_right > 0.48)
		ROS_WARN("velocity is ilegal, exceed max of velocity!");

	double rpm_right_accurate = vel_right / WHEEL_PERIMETER * 60.0;
    double rpm_left_accurate = vel_left / WHEEL_PERIMETER * 60.0;
	
	int rpm_right = static_cast<int>(rpm_right_accurate * WHEEL_RATIO);
	int rpm_left = static_cast<int>(rpm_left_accurate * WHEEL_RATIO);
	rpm_right = rpm_right>100?100:rpm_right;
	rpm_right = rpm_right<-100?-100:rpm_right;
	rpm_left = rpm_left>100?100:rpm_left;
	rpm_left = rpm_left<-100?-100:rpm_left;

	// mickx4's datasheet is terrible,
	// left front
	mickx4_array[3] = (uint8_t)((rpm_right&0xff00)>>8); // high bytes
	mickx4_array[4] = (uint8_t)(rpm_right&0xff); // low bytes
	// left back
	mickx4_array[7] = (uint8_t)((rpm_right&0xff00)>>8);
	mickx4_array[8] = (uint8_t)(rpm_right&0xff);
	// right front
	mickx4_array[5] = (uint8_t)((rpm_left&0xff00)>>8);
	mickx4_array[6] = (uint8_t)(rpm_left&0xff);
	// right back
	mickx4_array[9] = (uint8_t)((rpm_left&0xff00)>>8);
	mickx4_array[10] = (uint8_t)(rpm_left&0xff);

	for(uint8_t index = 0; index < 11; index++)
		mickx4_array[11] += mickx4_array[index];
	
	mickx4_serial.write(mickx4_array, sizeof(mickx4_array));

	uint8_t mickx4_rx[23] = {0};
	//ros::Duration(0.0101).sleep();
	while(mickx4_serial.available() == 0);
	if(mickx4_serial.available() != 0)
	{
		mickx4_serial.read(mickx4_rx, sizeof(mickx4_rx));

		//for(uint8_t index = 0; index < 23; index++)
		//	std::cout << std::hex << std::showbase << (mickx4_rx[index]&0XFF) << " ";
		//std::cout << std::endl;
	}
	else
		ROS_WARN("mickx4's response data is unavailable!");

	if(parse_serial_response(mickx4_rx, sizeof(mickx4_rx)) == 0)
		ROS_WARN("mickx4's response data is illegal");

	mickx4_serial.flushInput();
}

/**
 * @brief main procedure.
 *
 * @param argc
 * @param argv
 *
 * @return 
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "mickx4_drv");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/cmd_vel", 100, mickx4_ctrl_sub);
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);

	/* open serial */
	std::string mickx4_port("/dev/mickx4");
	unsigned long mickx4_port_baud = 115200;
	serial::Serial mickx4_serial(mickx4_port, mickx4_port_baud, serial::Timeout::simpleTimeout(1000));
	if(mickx4_serial.isOpen())
		ROS_DEBUG("/dev/mickx4 is opened for mickx4");
	else
		ROS_ERROR("open /dev/mickx4 fail for mickx4");
	/* read init value */
	comm_with_mickx4(mickx4_ctrl_msg, mickx4_serial);
	
	double left_dis = 0, right_dis = 0;
	double left_enc_before = 0, right_enc_before = 0;
	double right_enc = 0, left_enc = 0;
	double x = 0.0, y = 0.0, th = 0.0;
	const double ticks_per_meter = 6840.0 / WHEEL_PERIMETER;
	geometry_msgs::Quaternion odom_quat;
	tf::TransformBroadcaster broadcaster;
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_footprint";
	ros::Time last_time = ros::Time::now();
	ros::Rate loop_rate(20);
	
	right_enc_before = (double)(mickx4_info.right_back_encoder + mickx4_info.right_front_encoder) / 2.0;
	left_enc_before = (double)(mickx4_info.left_back_encoder + mickx4_info.left_front_encoder) / 2.0;

	while(ros::ok())
	{
		double dxy = 0.0;
		double dth = 0.0;
		double dt = 0.0000000001;

		ros::Time current_time = ros::Time::now();
		dt = (current_time - last_time).toSec();
		last_time = current_time;

		/* calcute odometry */
		comm_with_mickx4(mickx4_ctrl_msg, mickx4_serial);

		right_enc = (double)(mickx4_info.right_back_encoder + mickx4_info.right_front_encoder) / 2.0;
		left_enc = (double)(mickx4_info.left_back_encoder + mickx4_info.left_front_encoder) / 2.0;
	
		left_dis = (left_enc - left_enc_before) / ticks_per_meter * 1.045;
		right_dis = (right_enc - right_enc_before) / ticks_per_meter * 1.045;
		right_enc_before = right_enc;
		left_enc_before = left_enc;
		
		dxy = (left_dis + right_dis) / 2.0;
		dth = (right_dis - left_dis) / mickx4_width;
		double vel_xy = dxy / dt;
		double vel_th = dth / dt;

		th += dth;
		x += dxy * cosf(th);
		y += dxy * sinf(th);
		/* update transform */
		odom_trans.header.stamp = current_time;
		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 0;
		odom_quat = tf::createQuaternionMsgFromYaw(th);
		odom_trans.transform.rotation = odom_quat;
		
		/* update odometry */
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";
		odom.child_frame_id = "base_footprint";
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;
		odom.twist.twist.linear.x = vel_xy;
		odom.twist.twist.linear.y = 0.0;
		odom.twist.twist.linear.z = 0.0;
		odom.twist.twist.angular.x = 0.0;
		odom.twist.twist.angular.y = 0.0;
		odom.twist.twist.angular.z = vel_th;
		
		/* publish odometry and odom tf */
		broadcaster.sendTransform(odom_trans);
		odom_pub.publish(odom);

		ros::spinOnce();
		loop_rate.sleep();	
	}
	
	mickx4_serial.close();
	return 0;
}

