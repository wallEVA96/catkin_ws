/*
 * mickx4_velocity.cpp
 *                        _ _                 
 *         __      ____ _| | | _____   ____ _ 
 *         \ \ /\ / / _` | | |/ _ \ \ / / _` |
 *          \ V  V / (_| | | |  __/\ V / (_| |
 *           \_/\_/ \__,_|_|_|\___| \_/ \__,_|
 * Copyright (C) 2020 south china.
 *
 * Distributed under terms of the MIT license.
 */

#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include <signal.h>
#include <stdio.h>
#ifndef _WIN32
# include <termios.h>
# include <unistd.h>
#else
# include <windows.h>
#endif

#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_SPACE 0x20
#define KEYCODE_Q 0x71

/**
 * @brief KeyboardReader
 */
class KeyboardReader
{
public:
  KeyboardReader()
#ifndef _WIN32
    : kfd(0)
#endif
  {
#ifndef _WIN32
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    struct termios raw;
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
#endif
  }
  void readOne(char * c)
  {
#ifndef _WIN32
    int rc = read(kfd, c, 1);
    if (rc < 0)
    {
      throw std::runtime_error("read failed");
    }
#endif
  }
  void shutdown()
  {
#ifndef _WIN32
    tcsetattr(kfd, TCSANOW, &cooked);
#endif
  }
private:
#ifndef _WIN32
  int kfd;
  struct termios cooked;
#endif
};

KeyboardReader input;

/**
 * @brief quit
 *
 * @param sig
 */
void quit(int sig)
{
  (void)sig;
  input.shutdown();
  ros::shutdown();
  exit(0);
}

/**
 * @brief print_usage
 */
inline void print_usage()
{
	std::cout << "---------------------------------------------------------------" << std::endl;
	std::cout << "| Welcome to China North Industries Group Corporation Limited |" << std::endl;
	std::cout << "---------------------------------------------------------------" << std::endl;
	
	std::cout << "Usage:" << std::endl;
	std::cout << "      Up key to Forward" << std::endl;
	std::cout << "      Down key to Back" << std::endl;
	std::cout << "      Left key to Turn Left" << std::endl;
	std::cout << "      Right key to Turn Right" << std::endl;
	std::cout << "      Space key to Stop" << std::endl;
	std::cout << "      Q key to Quit Procedure" << std::endl;
	std::cout << "---------------------------------------------------------------" << std::endl;
}

/**
 * @brief 
 *
 * @param argc
 * @param argv
 *
 * @return 
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "mickx4_control");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000); // save 1000 cache message.
	signal(SIGINT,quit);

	ros::Rate loop_rate(40);
	char c;
	bool dirty=false;
	double linear_ = 0, angular_ = 0;
#define M_S (1)
	double l_scale_ = 0.3 * M_S, // m/s
		   a_scale_ = M_PI /2 * M_S; // rad/s.

	print_usage();

	while(ros::ok())
	{
		geometry_msgs::Twist mickx4_twist;	
//		geometry_msgs::Vector3 mickx4_linear;
//		geometry_msgs::Vector3 mickx4_angular;
		try
	    {
	      input.readOne(&c);
	    }
	    catch (const std::runtime_error &)
	    {
	      perror("read():");
	      return -1;
	    }
		linear_=angular_=0;
		ROS_DEBUG("value: 0x%02X\n", c);
		
		switch(c)
		{
		  case KEYCODE_LEFT:
		    ROS_INFO("LEFT");
		    angular_ = 1.0;
		    dirty = true;
		    break;
		  case KEYCODE_RIGHT:
		    ROS_INFO("RIGHT");
		    angular_ = -1.0;
		    dirty = true;
		    break;
		  case KEYCODE_UP:
		    ROS_INFO("UP");
		    linear_ = 1.0;
		    dirty = true;
		    break;
		  case KEYCODE_DOWN:
		    ROS_INFO("DOWN");
		    linear_ = -1.0;
		    dirty = true;
		    break;
		  case KEYCODE_SPACE:
		    ROS_INFO("STOP");
		    linear_ = 0.0;
		    angular_ = 0.0;
		    dirty = true;
		    break;
		  case KEYCODE_Q:
		    ROS_WARN("Quit Procedure");
			input.shutdown();
  			ros::shutdown();
  			exit(0);
		}
		mickx4_twist.angular.z = a_scale_*angular_;
	    mickx4_twist.linear.x = l_scale_*linear_;
   
//		mickx4_twist.linear = mickx4_linear;
//		mickx4_twist.angular = mickx4_angular;
		if(dirty == true)
		{
			dirty = false;
			chatter_pub.publish(mickx4_twist);
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

