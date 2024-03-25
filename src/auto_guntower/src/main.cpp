/*
 * main.cpp
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


/**
 * @brief main entry.
 *
 * @param argc
 * @param argv
 *
 * @return 
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "auto_guntower_node");
	ros::NodeHandle nh, nh_private("~");

	auto_guntower kuayuexianzu2023(nh, nh_private);
	
	ros::Rate rate(40);

	while(ros::ok())
	{
		kuayuexianzu2023.receive_remote_ctrl();
		ros::spinOnce();
		rate.sleep();
	}
	
	kuayuexianzu2023.stop();
	return 0;
}

