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

#include "chassis_commu/chassis_commu.hpp"


#define CHASSIS_PID_FREQ 30
#define CHASSIS_PID_CYCLE (1000.0/30)

/**
 * @brief chassisCOMM  
 *
 * @param port
 * @param baud_rate
 * @param time_out
 */
chassis_commu::chassis_commu(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
							:port_("/dev/chassisCOM4")
							,odom_frame_("odom")
							,base_footprint_frame_("base_footprint")
							,baud_rate_(115200)
							,time_out_(500)
							,pid_freq_(CHASSIS_PID_FREQ)
							,pid_cycle_(CHASSIS_PID_CYCLE)
							,curr_l_enc_(0)
							,curr_r_enc_(0)
							,Kp_("50")
							,Ki_("0")
							,Kd_("20")
							,Ko_("50")
							,front_left_sonar_(0)
							,front_midd_sonar_(0)
							,front_right_sonar_(0)
							,back_midd_sonar_(0)
{
	nh_private.param<std::string>("chassis_commu_port", port_, port_);
	nh_private.param<std::string>("odom_frame", odom_frame_, odom_frame_);
	nh_private.param<std::string>("base_footprint_frame", base_footprint_frame_, base_footprint_frame_);
    nh_private.param<int>("chassis_commu_baudrate", baud_rate_, baud_rate_);
    nh_private.param<int>("chassis_commu_timeout", time_out_, time_out_);
	nh_private.param<int>("chassis_encoder_resolution", chassis_encoder_resolution_, 1200);
	nh_private.param<double>("chassis_wheel_diameter",chassis_wheel_diameter_, 0.1288);
	nh_private.param<double>("chassis_wheel_track", chassis_wheel_track_, 0.3559);
	nh_private.param<double>("chassis_gear_reduction", chassis_gear_reduction_, 1.0);
	
	vel_sub_ = nh.subscribe("/cmd_vel", 100, &chassis_commu::chassisVelCallback, this);
	odom_pub_ = nh.advertise<nav_msgs::Odometry>("/odom", 100);
	
	front_left_sonar_pub_ = nh.advertise<sensor_msgs::Range>("/front_left_sonar", 5);
	front_midd_sonar_pub_ = nh.advertise<sensor_msgs::Range>("/front_midd_sonar", 5);
	front_right_sonar_pub_ = nh.advertise<sensor_msgs::Range>("/front_right_sonar", 5);
	back_midd_sonar_pub_ = nh.advertise<sensor_msgs::Range>("/back_midd_sonar", 5);

	chassis_enc_per_meter_ = chassis_encoder_resolution_*chassis_gear_reduction_/(chassis_wheel_diameter_*M_PI);
	ROS_INFO("Chassis Enc Per Meter: %f", chassis_enc_per_meter_);	
	
	openSerial();
	
	/* wait for chassis wakeup */
	ros::Duration(2).sleep();
	
	ROS_INFO("Create Connect With ChassisCOM.");
	if(getBaud() && resetEnc() && cfgPID())
		ROS_INFO("Succeed To Connect With ChassisCOM.");
	else
		ROS_WARN("Failed To Connect With ChassisCOM.");
}

/**
 * @brief releae resources
 */
chassis_commu::~chassis_commu()
{
	ROS_INFO("Exit Chassis Communication, Stop Robot.");
	setVelocity(0, 0);
	ROS_INFO("Close %s Device.", port_.c_str());
	if(serial_.isOpen())
		serial_.close();
}

/**
 * @brief open chassis serial port
 */
void chassis_commu::openSerial()
{
	for(;;){
		try{
			ROS_INFO("Cfg ChassisCOM at Port %s, Baudrate %d", port_.c_str(), baud_rate_);
			serial_.setPort(port_);
			serial_.setBaudrate(baud_rate_);
			serial::Timeout t_o = serial::Timeout::simpleTimeout(time_out_);
			serial_.setTimeout(t_o);	
			serial_.open();
			if(serial_.isOpen())
			{
				ROS_INFO("Succeed To Open ChassisCOM.");
				break;
			}
			else{
				ROS_WARN("Fail To Open ChassisCOM");
				ros::Duration(1).sleep();
			}
		}
		catch(serial::IOException &e)
		{
			ROS_WARN("Fail To Open ChassisCOM, Error Info: %s", e.what());
			ros::Duration(5).sleep();
		}
	}
}

/**
 * @brief write cmd to serial.
 *
 * @param cmd
 */
uint8_t chassis_commu::writeRequ(std::string &requ)
{
	if(serial_.isOpen())
	{
		serial_.write(requ);
		return 1;
	}
	else
		ROS_WARN("Chassis Comm Closed");
	return 0;
}

/**
 * @brief read from serial.
 *
 * @param buf
 * @param size
 */
uint8_t chassis_commu::readResp(std::string &buf, size_t size = 1)
{
	clock_t to_start, to_end;
	double to_duration = 0;
	float trans_ratio = 1000.0/CLOCKS_PER_SEC;

	to_start = clock();
	while(!serial_.available())
	{
		to_end = clock();
		to_duration = (to_end - to_start) * trans_ratio;
		if(to_duration > time_out_)
		{
			//ROS_WARN("ChassisCOM Read Timeout Over %f ms", to_duration);
			return 9;
		}
	}

	size_t read_size = serial_.readline(buf, size, "\r");
	if(read_size == 0){
		ROS_WARN("Chassis Comm Read Zero Size");
		return 0;
	}
	else
		return 1;
}

/**
 * @brief execute cmd by serial port
 *
 * @param cmd
 * @param result
 * @param result_size
 *
 * @return 
 */
uint8_t chassis_commu::exeCmd(std::string &cmd, std::string &result, size_t result_size)
{
	boost::mutex::scoped_lock write_lock(serial_mutex_);	
	serial_.flushInput();
	if(writeRequ(cmd))
	{
		uint8_t ret_val = readResp(result, result_size);
		if(ret_val == 1)
			return 1;
		else
		{
			if(ret_val == 9)
			{
				ROS_WARN("Exe Chassis Cmd %s", cmd.c_str());
				ROS_WARN("Cmd Resp Timeout %d ms", time_out_);
			}
		}
	}
		
	return 0;
}

/**
 * @brief get chassis_commu baud
 */
uint8_t chassis_commu::getBaud()
{
	std::string get_baud_cmd = "b\r";
	std::string baud_cmd_resp;

	uint8_t ret_val = exeCmd(get_baud_cmd, baud_cmd_resp, 8);
	ROS_INFO("Get Chassis Baud: %s", baud_cmd_resp.c_str());
	return ret_val;
}

/**
 * @brief reset encoder val.
 */
uint8_t chassis_commu::resetEnc()
{
	std::string reset_enc_cmd = "r\r";
	std::string reset_enc_cmd_resp;

	uint8_t ret_val = exeCmd(reset_enc_cmd, reset_enc_cmd_resp, 5);
	ROS_INFO("Reset Encoder Val Result: %s", reset_enc_cmd_resp.c_str());
	return ret_val;
}


/**
 * @brief get encorder val 
 *
 * @return get enc result
 */
uint8_t chassis_commu::getEnc()
{
	std::string get_enc_cmd = "e\r";
	std::string enc_cmd_resp;
	
#define ENC_CMD_RESP_SIZE 20
#define EAI_ENC_ACCUM_MAX 32768
#define EAI_ENC_ACCUM_MIN -32768
#define EAI_ENC_UPPER_LIMIT 32000
#define EAI_ENC_LOWER_LIMIT -32000

	long EAI_ENC_RANGE = EAI_ENC_ACCUM_MAX - EAI_ENC_ACCUM_MIN;

	if(exeCmd(get_enc_cmd, enc_cmd_resp, ENC_CMD_RESP_SIZE))
	{
		char cmd_resp_str[ENC_CMD_RESP_SIZE] = {0};
		strncpy(cmd_resp_str, enc_cmd_resp.c_str(), ENC_CMD_RESP_SIZE);
		
		char *p_l_enc = NULL, *p_r_enc = NULL, *p_str_save = NULL;
		p_l_enc = strtok_r(cmd_resp_str, " ", &p_str_save);
		p_r_enc = strtok_r(NULL, "\0", &p_str_save);
		if(p_r_enc == NULL || p_r_enc == NULL)
		{
			return 0;
		}
		last_l_enc_ = curr_l_enc_;
		last_r_enc_ = curr_r_enc_;
		curr_l_enc_ = atoi(p_l_enc);
		curr_r_enc_ = atoi(p_r_enc);
		
		/* EAI Chassis Happen Overflow */
		int8_t l_encoder_over_limit = 0, r_encoder_over_limit = 0;
		if(curr_l_enc_ < EAI_ENC_LOWER_LIMIT && last_l_enc_ > EAI_ENC_UPPER_LIMIT)
			l_encoder_over_limit = 1;
		else if(curr_l_enc_ > EAI_ENC_UPPER_LIMIT && last_l_enc_ < EAI_ENC_LOWER_LIMIT)
			l_encoder_over_limit = -1;
		else
			l_encoder_over_limit = 0;
		if(curr_r_enc_ < EAI_ENC_LOWER_LIMIT && last_r_enc_ > EAI_ENC_UPPER_LIMIT)
			r_encoder_over_limit = 1;
		else if(curr_r_enc_ > EAI_ENC_UPPER_LIMIT && last_r_enc_ < EAI_ENC_LOWER_LIMIT)
			r_encoder_over_limit = -1;
		else
			r_encoder_over_limit = 0;

		diff_l_enc_ = curr_l_enc_+l_encoder_over_limit*EAI_ENC_RANGE-last_l_enc_;
		diff_r_enc_ = curr_r_enc_+r_encoder_over_limit*EAI_ENC_RANGE-last_r_enc_;

		//ROS_INFO("Diff L Enc: %ld,Diff R Enc: %ld", diff_l_enc_, diff_r_enc_);
		return 1;
	}
	
	return 0;
}

/**
 * @brief get sonar sensor data.
 */
uint8_t chassis_commu::getSonar()
{
	std::string get_sonar_cmd = "p\r";
	std::string sonar_cmd_resp;
#define SONAR_CMD_RESP_SIZE 20
	if(exeCmd(get_sonar_cmd, sonar_cmd_resp, SONAR_CMD_RESP_SIZE))
	{
		if(sonar_cmd_resp.length() >= 8)
		{
			char cmd_resp_str[SONAR_CMD_RESP_SIZE] = {0};
			strncpy(cmd_resp_str, sonar_cmd_resp.c_str(), SONAR_CMD_RESP_SIZE);

			char *p_str_save = NULL, *p_front_l_dis = NULL, *p_front_m_dis = NULL, *p_front_r_dis = NULL, *p_back_m_dis = NULL;
			p_front_l_dis = strtok_r(cmd_resp_str, " ", &p_str_save);
			p_front_m_dis = strtok_r(NULL, " ", &p_str_save);
			p_front_r_dis = strtok_r(NULL, " ", &p_str_save);
			p_back_m_dis = strtok_r(NULL, "\r", &p_str_save);
			
			if(p_front_l_dis == NULL || p_front_r_dis == NULL || p_front_m_dis == NULL || p_back_m_dis == NULL)
			{
				ROS_WARN("Sonar Data Addr Is NULL");
				return 0;
			}
			
			front_left_sonar_ = atoi(p_front_l_dis);
			front_right_sonar_ = atoi(p_front_r_dis);
			front_midd_sonar_ = atoi(p_front_m_dis);
			back_midd_sonar_ = atoi(p_back_m_dis);
			return 1;
		}
	}
	else
		ROS_WARN("Sonar Reply Raw: %s", sonar_cmd_resp.c_str());
	
	return 0;
}

/**
 * @brief cfg chassis PID params
 *
 * @return 
 */
uint8_t chassis_commu::cfgPID()
{
	std::string cfg_pid_cmd = "u "+Kp_+":"+Kd_+":"+Ki_+":"+Ko_+"\r";
	std::string pid_cmd_resp;

	uint8_t ret_val = exeCmd(cfg_pid_cmd, pid_cmd_resp, 5);
	ROS_INFO("Cfg PID Result: %s", pid_cmd_resp.c_str());
	return ret_val;
}

/**
 * @brief set chassis velocity target.
 *
 * @param l_vel  m/s
 * @param r_vel  m/s
 */
void chassis_commu::setVelocity(float l_vel,float r_vel)
{
	long l_enc_per_sec = std::round(l_vel * chassis_enc_per_meter_ / pid_freq_);
	long r_enc_per_sec = std::round(r_vel * chassis_enc_per_meter_ / pid_freq_);
	
	//std::string set_vel_cmd = "z "+std::to_string(l_enc_per_sec)+" "+std::to_string(r_enc_per_sec)+";\r";
	std::string set_vel_cmd = "m "+std::to_string(l_enc_per_sec)+" "+std::to_string(r_enc_per_sec)+"\r";
	std::string vel_cmd_resp;

//	ROS_INFO("set vel cmd: %s", set_vel_cmd.c_str());
	exeCmd(set_vel_cmd, vel_cmd_resp, 5);
//	ROS_INFO("set vel result: %s", vel_cmd_resp.c_str());
}

/**
 * @brief subscriber's callback function.
 *
 * @param vel_msg
 */
void chassis_commu::chassisVelCallback(const geometry_msgs::Twist &vel_msg)
{
	double vel_x = 0.0, vel_theta = 0.0;
	double vel_left = 0.0, vel_right = 0.0;

	vel_x = vel_msg.linear.x;
	vel_theta = vel_msg.angular.z;

	call_tm_ = ros::Time::now();
	if(vel_theta == 0)
		vel_left = vel_right = vel_x;
	else if(vel_x == 0)
	{
		vel_right = vel_theta * chassis_wheel_track_ * 0.5;
		vel_left = -vel_right;
	}
	else
	{
		vel_left = vel_x - 0.5 * vel_theta * chassis_wheel_track_;
		vel_right = vel_x + 0.5 * vel_theta * chassis_wheel_track_;
	}
	setVelocity(vel_left, vel_right);
//	ROS_INFO("Set Chassis Velocity: %lf, %lf", vel_left, vel_right);
}

/**
 * @brief poll ros task.
 */
void chassis_commu::poll()
{
	nav_msgs::Odometry odom_info;
	geometry_msgs::TransformStamped odom_trans;
	geometry_msgs::Quaternion odom_quat;
	double l_enc_dis = 0.0, r_enc_dis = 0.0;
	double d_xy = 0.0, d_th = 0.0, d_t = 0.0000001, d_x = 0.0, d_y = 0.0;
	double vel_xy = 0.0, vel_th = 0.0;	
	uint vel_ot_flag = 0;

	curr_tm_ = ros::Time::now();
	last_tm_ = curr_tm_;
	call_tm_ = curr_tm_;
	ros::Rate rate(50);

	/* set odom covariance */
	odom_info.pose.covariance[0] = 1e-3;
	odom_info.pose.covariance[7] = 1e-3;
	odom_info.pose.covariance[14] = 1e6;
	odom_info.pose.covariance[21] = 1e6;
	odom_info.pose.covariance[28] = 1e6;
	odom_info.pose.covariance[35] = 1e3;
	odom_info.twist.covariance[0] = 1e-3;
	odom_info.twist.covariance[7] = 1e-3;
	odom_info.twist.covariance[14] = 1e6;
	odom_info.twist.covariance[21] = 1e6;
	odom_info.twist.covariance[28] = 1e6;
	odom_info.twist.covariance[35] = 1e3;
	
	while(ros::ok())
	{
		curr_tm_ = ros::Time::now();	
		/* Timeout 100ms*/
		if((curr_tm_-call_tm_).toSec() > 0.5)
		{
			if(vel_ot_flag == 0)
			{
				ROS_WARN("Velocity Receive Time Out");
				setVelocity(0.0, 0.0);
				ROS_WARN("Stop Robot Chassis");
				vel_ot_flag = 1;
			}
		}
		else
			vel_ot_flag = 0;

		if(getEnc())
		{
			d_t = (curr_tm_ - last_tm_).toSec();
			last_tm_ = curr_tm_;
			
			l_enc_dis = diff_l_enc_/chassis_enc_per_meter_;
			r_enc_dis = diff_r_enc_/chassis_enc_per_meter_;
			
			d_xy = (l_enc_dis + r_enc_dis)/2.0;
			d_th = (r_enc_dis - l_enc_dis)/chassis_wheel_track_;


			if(d_xy != 0)
			{
				d_x =  cos(d_th) * d_xy;
				d_y = -sin(d_th) * d_xy;
				chassis_x_ += (cos(chassis_theta_) * d_x - sin(chassis_theta_) * d_y);
				chassis_y_ += (sin(chassis_theta_) * d_x + cos(chassis_theta_) * d_y);
			}
			
			if(d_th != 0)
				chassis_theta_ += d_th;
			/* Calcute Robot velocity And Angular */
			vel_xy = d_xy / d_t;
			vel_th = d_th / d_t;
			//ROS_INFO("Vel: %f, Vel Th %f", vel_xy, vel_th);

			odom_trans.header.stamp = curr_tm_;
			odom_trans.header.frame_id = odom_frame_;
			odom_trans.child_frame_id = base_footprint_frame_;
			odom_trans.transform.translation.x = chassis_x_;
			odom_trans.transform.translation.y = chassis_y_;
			odom_trans.transform.translation.z = 0.0;
			odom_quat = tf::createQuaternionMsgFromYaw(chassis_theta_);
			odom_trans.transform.rotation = odom_quat;
			odom_bc_.sendTransform(odom_trans);

			odom_info.header.stamp = curr_tm_;
			odom_info.header.frame_id = odom_frame_;
			odom_info.child_frame_id = base_footprint_frame_;
			odom_info.pose.pose.position.x = chassis_x_;
			odom_info.pose.pose.position.y = chassis_y_;
			odom_info.pose.pose.position.z = 0.0;
			odom_info.pose.pose.orientation = odom_quat;
			odom_info.twist.twist.linear.x = vel_xy;
			odom_info.twist.twist.linear.y = 0.0;
			odom_info.twist.twist.angular.z = vel_th;
			odom_pub_.publish(odom_info);
		}
		else
			ROS_WARN("Get Enc Errorly, Attention! ");
		/* measure sonar sensor */
		if(getSonar())
		{
#define FIELD_OF_VIEW_RAD (30*3.14/180)
			//ROS_INFO("Sonar Range: %fm, %fm, %fm, %fm", front_left_sonar_/100.0, front_midd_sonar_/100.0, front_right_sonar_/100.0, back_midd_sonar_/100.0);	
			sensor_msgs::Range sonar_sensor;
			float sonar_range;
			sonar_sensor.radiation_type = sensor_msgs::Range::ULTRASOUND;
			sonar_sensor.field_of_view = FIELD_OF_VIEW_RAD;
			sonar_sensor.header.stamp = curr_tm_;
			sonar_sensor.max_range = 0.5;
			sonar_sensor.min_range = 0.0;
			sonar_sensor.header.frame_id = "front_left_sonar";
			sonar_range = front_left_sonar_/100.0;
			sonar_range = (sonar_range==0.0?1.0:sonar_range);
			sonar_range = (sonar_range>=0.5?0.5:sonar_range);
			sonar_sensor.range = sonar_range;
			front_left_sonar_pub_.publish(sonar_sensor);
		
			sonar_sensor.header.frame_id = "front_midd_sonar";
			sonar_range = front_midd_sonar_/100.0;
			sonar_range = (sonar_range==0.0?1.0:sonar_range);
			sonar_range = (sonar_range>=0.5?0.5:sonar_range);
			sonar_sensor.range = sonar_range;
			front_midd_sonar_pub_.publish(sonar_sensor);

			sonar_sensor.header.frame_id = "front_right_sonar";
			sonar_range = front_right_sonar_/100.0;
			sonar_range = (sonar_range==0.0?1.0:sonar_range);
			sonar_range = (sonar_range>=0.5?0.5:sonar_range);
			sonar_sensor.range = sonar_range;
			front_right_sonar_pub_.publish(sonar_sensor);
			
			sonar_sensor.header.frame_id = "back_midd_sonar";
			sonar_range = back_midd_sonar_/100.0;
			sonar_range = (sonar_range==0.0?1.0:sonar_range);
			sonar_range = (sonar_range>=0.5?0.5:sonar_range);
			sonar_sensor.range = sonar_range;
			back_midd_sonar_pub_.publish(sonar_sensor);
		}

		ros::spinOnce();
		rate.sleep();
	}
}

