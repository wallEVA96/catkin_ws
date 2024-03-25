/*
 * auto_guntower.h
 * Copyright (C) 2023 two <two@ubuntu>
 *
 * Distributed under terms of the MIT license.
 */

#ifndef AUTO_GUNTOWER_H
#define AUTO_GUNTOWER_H

#include <ros/ros.h>
#include <ros/assert.h>
#include <string.h>
#include <stdio.h>
#include <string>
#include <serial/serial.h>
#include <boost/circular_buffer.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <iostream>
#include <chrono>
#include <sys/types.h>
#include <sys/socket.h>
#include <linux/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "time.h"

typedef struct _Track_msg {
    int16_t x;
    int16_t y;
}TrackMsg_t;

/**
 * @brief auto_guntower
 */
class auto_guntower
{
public:
	auto_guntower(ros::NodeHandle& nh, ros::NodeHandle &nh_private);
	~auto_guntower();

	void run();
	void stop();
//	void init_gunsight_serial();
//	void read_gunsight();
//	void analyse_gunsight();

	void init_got_pin(void);
	void init_guntower_serial();
	void guntower_sbus_ctrl(int16_t got, int16_t pitch, int16_t yaw);

	bool get_gunsight_target();
	bool init_remote_ctrl(void);
	void receive_remote_ctrl(void);
	bool create_pipe(void);
	void run_pipe_loop(void);
	void run_task_action(void);
	void run_got_action(uint8_t act);
	void run_shot_action(void);

private:
	ros::NodeHandle nh_, nh_private_;
    int pipe_fd_;
	std::string pipe_fifo_path;
	double pitch_p_;
	double pitch_i_;
	double pitch_d_;
	double pitch_err_;
	double pitch_pre_err_;
	double pitch_err_i_;
	double pitch_slope_;
	double pitch_set_vel_;

	double yaw_p_;
	double yaw_i_;
	double yaw_d_;
	double yaw_err_;
	double yaw_pre_err_;
	double yaw_err_i_;
	double yaw_slope_;
	double yaw_set_vel_;

	int udp_socket_fd_;
	uint8_t rec_ctrl_buff_[14];
	socklen_t src_addr_len_;
	struct sockaddr_in src_addr_;

	std::atomic<TrackMsg_t> gunsight_track_msg_;
	std::atomic<int16_t> remote_pitch_ctrl_;
	std::atomic<int16_t> remote_yaw_ctrl_;
	std::atomic<int8_t> remote_mode_ctrl_;
	std::atomic<int8_t> remote_got_ctrl_;
	std::atomic<int> got_pin_;
	std::atomic<int> auto_shot_ctrl_;

//	serial::Serial gunsight_serial_;
//	std::string gunsight_port_;
//	unsigned int gunsight_bdrate_;
//  std::string gunsight_data_;
// 	int gunsight_buffer_size_;
//  boost::shared_ptr<boost::circular_buffer<char> > gunsight_data_buffer_ptr_;
//  boost::mutex gunsight_m_mutex_;  
//  boost::thread read_gunsight_thread_;
//  boost::thread analy_gunsight_thread_;
    boost::thread gunsight_pipe_thread_;
    boost::thread auto_guntower_thread_;
    boost::thread auto_guntower_shot_thread_;
	boost::mutex guntower_serial_mutex_;  
	boost::mutex got_pin_mutex_;  

private:
	serial::Serial guntower_sbus_serial_;
	std::string guntower_sbus_port_;
	unsigned int guntower_sbus_bdrate_;
	uint8_t guntower_sbus_packet_[25];
	std::string guntower_port_;
	unsigned int guntower_bdrate_;
};

#endif /* !AUTO_GUNTOWER_H */
