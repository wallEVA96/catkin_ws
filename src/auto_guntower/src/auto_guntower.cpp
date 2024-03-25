/*
 * auto_guntower.cpp
 *                        _ _                 
 *         __      ____ _| | | _____   ____ _ 
 *         \ \ /\ / / _` | | |/ _ \ \ / / _` |
 *          \ V  V / (_| | | |  __/\ V / (_| |
 *           \_/\_/ \__,_|_|_|\___| \_/ \__,_|
 * Copyright (C) 2023 two <two@ubuntu>
 *
 * Distributed under terms of the MIT license.
 */

#include "auto_guntower/auto_guntower.h"
#include <thread>
#include <atomic>
#include <iostream>
#include <ros/ros.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <poll.h> 
#include <math.h>
#include <JetsonGPIO.h>

#define PI 3.1415926535859
/**
 * @brief contruct function.
 *
 * @param nh
 * @param nh_private
 */
auto_guntower::auto_guntower(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
							:nh_(nh),
							nh_private_(nh_private),
							pipe_fifo_path("/tmp/track_fifo"),
							got_pin_(7),
							remote_mode_ctrl_(1),
							remote_got_ctrl_(0),
//							gunsight_port_("/dev/com2"),
//							gunsight_bdrate_(115200),
							guntower_port_("/dev/com1"),
							guntower_bdrate_(115200)
{
	TrackMsg_t zero_msg = {0};
	gunsight_track_msg_.store(zero_msg);

	/* init udp */
	if(init_remote_ctrl())
	{
		init_got_pin();
		/* init guntower */
		init_guntower_serial();
		if(create_pipe())
		{
			gunsight_pipe_thread_ = boost::thread(boost::bind(&auto_guntower::run_pipe_loop, this));
		}
		auto_guntower_thread_ = boost::thread(boost::bind(&auto_guntower::run_task_action, this));
		auto_guntower_shot_thread_ = boost::thread(boost::bind(&auto_guntower::run_shot_action, this));
	}


	/* init gunsight */
//	init_gunsight_serial();

//  gunsight_buffer_size_ = 1024;
//  gunsight_data_buffer_ptr_ = boost::shared_ptr<boost::circular_buffer<char> >(new boost::circular_buffer<char>(gunsight_buffer_size_));
//	read_gunsight_thread_ = boost::thread(boost::bind(&auto_guntower::read_gunsight, this));
//	analy_gunsight_thread_ = boost::thread(boost::bind(&auto_guntower::analyse_gunsight, this));

}

/**
 * @brief deconstruct.
 */
auto_guntower::~auto_guntower()
{
//	gunsight_data_buffer_ptr_.reset();
//	read_gunsight_thread_.join();
//	analy_gunsight_thread_.join();
//	if(gunsight_serial_.isOpen())
//		gunsight_serial_.close();

	gunsight_pipe_thread_.join();
	auto_guntower_thread_.join();
	auto_guntower_shot_thread_.join();

	GPIO::cleanup();

	if(guntower_sbus_serial_.isOpen())
		guntower_sbus_serial_.close();

	if(udp_socket_fd_ > 0)
		close(udp_socket_fd_);
	
	if(pipe_fd_ > 0)
		close(pipe_fd_);
}

/**
 * @brief init gunsight serial.
 */
//void auto_guntower::init_gunsight_serial()
//{
//	while(ros::ok())
//	{
//		try
//		{
//			gunsight_serial_.setPort(gunsight_port_);
//			gunsight_serial_.setBaudrate(gunsight_bdrate_);
//			serial::Timeout to = serial::Timeout::simpleTimeout(1000);
//			gunsight_serial_.setTimeout(to);
//			gunsight_serial_.open();
//			
//			if(gunsight_serial_.isOpen())
//			{
//				ROS_INFO("Opened Gunsight Serial");
//				break;
//			}
//		}
//		catch(serial::IOException &e)
//		{
//			ROS_INFO("Open Gunsight Serial Failed, Err: %s", e.what());
//			sleep(1);
//		}
//	}
//}

/**
 * @brief read serial gunsigh
 */
//void auto_guntower::read_gunsight()
//{
//	ROS_INFO("Gunsight Read");
//	ros::Rate rate(180);
//	while(ros::ok())
//	{
//		{
//			boost::mutex::scoped_lock lock(gunsight_m_mutex_);
//			if(gunsight_serial_.available())
//			{
//				gunsight_data_ = gunsight_serial_.read(gunsight_serial_.available());
//				ROS_INFO("Gunsight Read");
//				{
//					for(int i = 0; i < gunsight_data_.length(); i++)
//					{
//						gunsight_data_buffer_ptr_->push_back(gunsight_data_[i]);
//					}
//				}
//			}
//		}
//
//		rate.sleep();
//	}
//}

/**
 * @brief  gunsight receive protocol: ee 16 08 48 0a 00 00 00   00 00 00 00 00 00 00 5e
 */
//void auto_guntower::analyse_gunsight()
//{
//	uint8_t data = 0x00;
//	long flags = 0;
//	ros::Rate rate(120);
//	while(ros::ok())
//	{
//        while(!gunsight_data_buffer_ptr_->empty())
//		{
//			boost::mutex::scoped_lock lock(gunsight_m_mutex_);
//			data = uint8_t(gunsight_data_buffer_ptr_->front());
//			gunsight_data_buffer_ptr_->pop_front();
//			if(data == 0xee)
//			{
//				ROS_INFO("Receive Count: %ld", flags++);
//				break;
//			}
//		}
//		rate.sleep();
//	}
//}

/**
 * @brief init guntower serial.
 */
void auto_guntower::init_guntower_serial()
{
	while(ros::ok())
	{
		try
		{
			guntower_sbus_serial_.setPort(guntower_port_);
			guntower_sbus_serial_.setBaudrate(guntower_bdrate_);
			serial::Timeout to = serial::Timeout::simpleTimeout(1000);
			guntower_sbus_serial_.setTimeout(to);
			guntower_sbus_serial_.open();
			
			if(guntower_sbus_serial_.isOpen())
			{
				ROS_INFO("Opened Guntower Serial");
				break;
			}
		}
		catch(serial::IOException &e)
		{
			ROS_INFO("Open Guntower Serial Failed, Err: %s", e.what());
			sleep(1);
		}
	}
}

/**
 * @brief target get.
 */
bool auto_guntower::get_gunsight_target()
{
//	const float pixel_size = 0.0029; //mm
	if(pipe_fd_ > 0)
	{
//		double a_pix = atan((gunsight_track_msg_.load().x * pixel_size) / 5.4) / PI * 180;	
		ROS_INFO("T x: %d, T y: %d", gunsight_track_msg_.load().x, gunsight_track_msg_.load().y);	
		return true;
	}
	else
		ROS_ERROR("Can't Get Gunsight Target.");

	return false;
}

/**
 * @brief guntower sbus ctrl 
 */
void auto_guntower::guntower_sbus_ctrl(int16_t got, int16_t pitch, int16_t yaw)
{
	boost::mutex::scoped_lock gs_lock(guntower_serial_mutex_);
	uint16_t channels[16] = {0};

	pitch = -pitch;
	yaw = -yaw;

	//	pitch += 1000;
//	yaw += 1000;
//	got += 1000;
	if(pitch > 0) // 160 dead area.
		pitch += 1160;
	else if(pitch == 0)
		pitch += 1000;
	else
		pitch += 805;

	/* 100 blank area */
	if(yaw > 0)
		yaw += 1110; //1160
	else if(yaw == 0)
		yaw += 1000;
	else
		yaw += 890;

	got += 1000;

	if(got > 1800) got = 1800;
	if(got < 200)  got =  200;
	
	if(pitch > 1800) pitch = 1800;
	if(pitch < 200)  pitch =  200;
	
	if(yaw > 1800) yaw = 1800;
	if(yaw < 200)  yaw =  200;

	/* 10 channel */
	// 192 ~ 1792, 992.
	channels[9] = got;
	// up to down, 0~2047
	channels[14] = yaw;
	// right to left, 0~2047
	channels[15] = pitch;
	
	// SBUS header
    guntower_sbus_packet_[0] = 0x0F; 

    // 16 channels of 11 bit data
//    guntower_sbus_packet_[1]  = (unsigned char) ((channels[0] & 0x07FF));
//    guntower_sbus_packet_[2]  = (unsigned char) ((channels[0] & 0x07FF)>>8   | (channels[1] & 0x07FF)<<3);
//    guntower_sbus_packet_[3]  = (unsigned char) ((channels[1] & 0x07FF)>>5   | (channels[2] & 0x07FF)<<6);
//    guntower_sbus_packet_[4]  = (unsigned char) ((channels[2] & 0x07FF)>>2);
//    guntower_sbus_packet_[5]  = (unsigned char) ((channels[2] & 0x07FF)>>10  | (channels[3] & 0x07FF)<<1);
//    guntower_sbus_packet_[6]  = (unsigned char) ((channels[3] & 0x07FF)>>7   | (channels[4] & 0x07FF)<<4);
//    guntower_sbus_packet_[7]  = (unsigned char) ((channels[4] & 0x07FF)>>4   | (channels[5] & 0x07FF)<<7);
//    guntower_sbus_packet_[8]  = (unsigned char) ((channels[5] & 0x07FF)>>1);
//    guntower_sbus_packet_[9]  = (unsigned char) ((channels[5] & 0x07FF)>>9   | (channels[6] & 0x07FF)<<2);
//    guntower_sbus_packet_[10] = (unsigned char) ((channels[6] & 0x07FF)>>6   | (channels[7] & 0x07FF)<<5);
//    guntower_sbus_packet_[11] = (unsigned char) ((channels[7] & 0x07FF)>>3);
//    guntower_sbus_packet_[12] = (unsigned char) ((channels[8] & 0x07FF));
    guntower_sbus_packet_[13] = (unsigned char) ((channels[8] & 0x07FF)>>8   | (channels[9] & 0x07FF)<<3);
    guntower_sbus_packet_[14] = (unsigned char) ((channels[9] & 0x07FF)>>5   | (channels[10] & 0x07FF)<<6);  
//    guntower_sbus_packet_[15] = (unsigned char) ((channels[10] & 0x07FF)>>2);
//    guntower_sbus_packet_[16] = (unsigned char) ((channels[10] & 0x07FF)>>10 | (channels[11] & 0x07FF)<<1);
//    guntower_sbus_packet_[17] = (unsigned char) ((channels[11] & 0x07FF)>>7  | (channels[12] & 0x07FF)<<4);
//    guntower_sbus_packet_[18] = (unsigned char) ((channels[12] & 0x07FF)>>4  | (channels[13] & 0x07FF)<<7);
//    guntower_sbus_packet_[19] = (unsigned char) ((channels[13] & 0x07FF)>>1);
    guntower_sbus_packet_[20] = (unsigned char) ((channels[13] & 0x07FF)>>9  | (channels[14] & 0x07FF)<<2);
    guntower_sbus_packet_[21] = (unsigned char) ((channels[14] & 0x07FF)>>6  | (channels[15] & 0x07FF)<<5);
    guntower_sbus_packet_[22] = (unsigned char) ((channels[15] & 0x07FF)>>3);

    // flags
    guntower_sbus_packet_[23] = 0x00;

    // footer
    guntower_sbus_packet_[24] = 0x00;
	
	/* 282 ~ 1722 */
	if(guntower_sbus_serial_.isOpen())
	{
	//	ROS_INFO("Guntower Ctrl");
		guntower_sbus_serial_.write(guntower_sbus_packet_, 25);
	}
}

/**
 * @brief init remote ctrl.
 */
bool auto_guntower::init_remote_ctrl(void)
{
	src_addr_len_ = sizeof(struct sockaddr_in);
	
	udp_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
	if(udp_socket_fd_ < 0)
	{
		ROS_ERROR("socket create fail.");
		return false;
	}
	else
		ROS_INFO("socket create success.");

	struct sockaddr_in udp_addr;
	memset(&udp_addr, 0 , sizeof(struct sockaddr_in));
	udp_addr.sin_family = AF_INET;
	udp_addr.sin_port = htons(9000);
	udp_addr.sin_addr.s_addr = inet_addr("192.168.45.154");

	if(bind(udp_socket_fd_, (struct sockaddr*)&udp_addr, sizeof(udp_addr)) == -1)
	{
		ROS_ERROR("socket bind fail.");
		return false;
	}
	else
		ROS_INFO("socket bind success.");

	struct timeval tv;
	tv.tv_sec = 1;
	tv.tv_usec = 0;
	if(setsockopt(udp_socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0)
	{
		ROS_INFO("socket option SO_RCVTIMEO not support");
		return false;
	}
	else
	{
		ROS_INFO("socket set timeout: %ld sec %ld usec.", tv.tv_sec, tv.tv_usec);
	}

	int s_rr = 1;
	if(setsockopt(udp_socket_fd_, SOL_SOCKET, SO_REUSEADDR, &s_rr, sizeof(s_rr)) < 0)
	{
		ROS_INFO("socket option SO_REUSEADDR not support");
		return false;
	}
	else
	{
		ROS_INFO("socket set SO_REUSEADDR.");
	}

	return true;
}

/**
 * @brief receive_remote_ctrl 
 */
void auto_guntower::receive_remote_ctrl()
{
	if(udp_socket_fd_ <= 0)
	{
		ROS_ERROR("socket init fail, system disable.");
		return;
	}

	std::string mode_str;
	int8_t rz = recvfrom(udp_socket_fd_, rec_ctrl_buff_, sizeof(rec_ctrl_buff_), 0, (struct sockaddr*)&src_addr_, &src_addr_len_);

	if(rz > 0)
	{
//		for(uint8_t i = 0; i < 14; i++)
//			printf("%2x ", rec_ctrl_buff_[i]);
//		printf("\n");
		
		if(rec_ctrl_buff_[0] == 0x55 && rec_ctrl_buff_[1] == 0xff &&
		   rec_ctrl_buff_[12] == 0xff && rec_ctrl_buff_[13] == 0xfe)
		{
			remote_yaw_ctrl_   = rec_ctrl_buff_[9] | rec_ctrl_buff_[10] << 8;
			remote_pitch_ctrl_ =  rec_ctrl_buff_[7] | rec_ctrl_buff_[8] << 8;
			remote_got_ctrl_ = rec_ctrl_buff_[6];
			remote_mode_ctrl_ = rec_ctrl_buff_[5];
			if(remote_mode_ctrl_ == 1)
				mode_str = "manual";
			else
				mode_str = "auto";
			ROS_INFO("remote ctrl mode: %s, pitch ctrl: %d, yaw ctrl: %d, got ctrl: %d", mode_str.c_str(), remote_pitch_ctrl_.load(), remote_yaw_ctrl_.load(), remote_got_ctrl_.load());
		}
		else
			ROS_ERROR("remote receive invalid ctrl.");
	}
	else
	{
		ROS_ERROR("socket recevive overtime, err: %d", rz);
		if(remote_mode_ctrl_ == 1){
			ROS_INFO("socket ot, manual ctrl, stop action.");
			stop();
		}
	}
}

/**
 * @brief stop.
 */
void auto_guntower::stop()
{
	guntower_sbus_ctrl(0, 0, 0);
	std::this_thread::sleep_for(std::chrono::milliseconds(5));
	guntower_sbus_ctrl(0, 0, 0);
	std::this_thread::sleep_for(std::chrono::milliseconds(5));
	guntower_sbus_ctrl(0, 0, 0);
}

/**
 * @brief create_pipe
 *
 * @return 
 */
bool auto_guntower::create_pipe()
{
    int rv = -1;
    if (access(pipe_fifo_path.data(), F_OK) != 0)
    {
        rv = mkfifo(pipe_fifo_path.data(), 0664);
        if (rv == -1 && errno != EEXIST)
        {
			ROS_ERROR("create pipe file failed."); 
            return false;
        }
    }
	
    return true;
}

/**
 * @brief run_pipe_loop
 */
void auto_guntower::run_pipe_loop()
{
	ROS_INFO("starting open pipe list."); 
    pipe_fd_ = open(pipe_fifo_path.data(), O_RDONLY /*| O_NONBLOCK*/);
    if (pipe_fd_ < 0) {
		ROS_ERROR("open fifo path error."); 
        close(pipe_fd_);
        pipe_fd_ = 0;
        return ;
    }
	ROS_INFO("open fifo path success."); 

	int ret = 0;
    struct pollfd fds[1];
    fds[0].fd = pipe_fd_;
    fds[0].events = POLLIN;
   
	while(ros::ok())
	{
		ret = poll(fds, 1, 3000);
        if (ret < 0) {
			ROS_ERROR("poll error."); 
            break;
        }
        else if ( 0 == ret) {
			ROS_INFO("poll over, but no event."); 
            continue;
        }

        if(fds[0].revents & POLLIN) {
            ret = read(pipe_fd_, &gunsight_track_msg_, sizeof(TrackMsg_t));
            if (ret < 0) {
				ROS_ERROR("read pipe msg err."); 
                break;
            }
            else if (0 == ret) {
				ROS_ERROR("poll ret = 0."); 
                break;
            }
            else {
			//	ROS_INFO("receive new target."); 
		//		ROS_INFO("T x: %d, T y: %d", gunsight_track_msg_.load().x, gunsight_track_msg_.load().y);	
            }
        }
	}
}

/**
 * @brief init_got_pin
 */
void auto_guntower::init_got_pin()
{
    GPIO::setmode(GPIO::BOARD);
    GPIO::setup(got_pin_, GPIO::OUT, GPIO::HIGH);
    GPIO::output(got_pin_, GPIO::HIGH);
}

/**
 * @brief run got_ action.
 */
void auto_guntower::run_got_action(uint8_t act)
{
	boost::mutex::scoped_lock lock(got_pin_mutex_);
	if(act == 1){
		GPIO::output(got_pin_, GPIO::LOW);
		std::this_thread::sleep_for(std::chrono::milliseconds(85));
//		ROS_INFO("remote impl shotttttt action +++-----------------> (10).");
		GPIO::output(got_pin_, GPIO::HIGH);
		std::this_thread::sleep_for(std::chrono::milliseconds(15));
	}
	else
		GPIO::output(got_pin_, GPIO::HIGH);
}

/**
 * @brief implement attack task.
 */
void auto_guntower::run_task_action()
{
	ros::Rate rate(100);
	uint8_t reduce_shot_fre = 0;

	pitch_p_ = 6.6; //3.6
	pitch_i_ = 0.018;
	pitch_d_ = 0.43;

	yaw_p_ = 7.86; //8.6
	yaw_i_ = 0.101;// 0.041; //0.018
	yaw_d_ = 0.34; //0.33

	while(ros::ok())
	{
		/* manual shot */
		if(remote_mode_ctrl_ == 1)
		{
			guntower_sbus_ctrl(0, remote_pitch_ctrl_, remote_yaw_ctrl_);
		}
		else if(remote_mode_ctrl_ == 0)
		{
			// ROS_INFO("run auto shot!");
			if(get_gunsight_target())
			{
				pitch_err_ = gunsight_track_msg_.load().y;
				pitch_err_i_ += pitch_err_;
				if(pitch_err_i_ >=  640) pitch_err_i_ = 640;
				if(pitch_err_i_ <= -640) pitch_err_i_ = -640;
				
				pitch_slope_ = pitch_err_ - pitch_pre_err_;

				pitch_set_vel_ = pitch_p_ * pitch_err_ + pitch_i_ * pitch_err_i_ + pitch_d_ * pitch_slope_;	
				pitch_pre_err_ = pitch_err_;
				ROS_INFO("pitch_set_vel_: %lf, pitch_err_i: %lf, pitch_err_i*i: %f, pitch slope: %lf. ", pitch_set_vel_, pitch_err_i_, pitch_err_i_*pitch_i_, pitch_set_vel_);
			
				yaw_err_ = gunsight_track_msg_.load().x;
				yaw_err_i_ += yaw_err_;
				if(yaw_err_i_ >=  640) yaw_err_i_ = 640;
				if(yaw_err_i_ <= -640) yaw_err_i_ = -640;
				
				yaw_slope_ = yaw_err_ - yaw_pre_err_;

				yaw_set_vel_ = yaw_p_ * yaw_err_ + yaw_i_ * yaw_err_i_ + yaw_d_ * yaw_slope_;

				yaw_pre_err_ = yaw_err_;
				ROS_INFO("yaw_set_vel_: %lf, yaw_err_i: %lf, yaw_err_i*i: %lf, yaw slope: %lf. ", yaw_set_vel_, yaw_err_i_, yaw_err_i_*yaw_i_, yaw_set_vel_);
			
				guntower_sbus_ctrl(0, pitch_set_vel_, yaw_set_vel_);
			}
			if(pitch_err_  <= 20)
				auto_shot_ctrl_ = 1;
			else
				auto_shot_ctrl_ = 0;
		}
		rate.sleep();
	}
}

/**
 * @brief 
 */
void auto_guntower::run_shot_action()
{
	uint8_t reduce_shot_fre = 0;
	ros::Rate rate(100);

	while(ros::ok())
	{
		if(remote_mode_ctrl_ == 1)
		{
			if(remote_got_ctrl_ == 1)
			{
				reduce_shot_fre++;
				if(reduce_shot_fre % 6 == 0)// 60ms per
				{
					run_got_action(1);
					if(reduce_shot_fre >= 198)
						reduce_shot_fre = 0;
				}
			}
			else
			{
				/* stop shot */
				// ROS_INFO("remote stop shotting");
				run_got_action(0);
			}
		}
		else if(remote_mode_ctrl_ == 0)
		{
			/* auto attack */
			if(auto_shot_ctrl_ == 1)
				run_got_action(1);
			else
				run_got_action(0);
		}

		rate.sleep();
	}
}

