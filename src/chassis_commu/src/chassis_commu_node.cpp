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
#include "signal.h"

/* it will cause desconstruct function won't be call.
 void signal_quit(int sig)
{
	ros::shutdown();
	exit(0);
}
*/
/**
 * @brief chassis_commu main entry
 *
 * @param argc
 * @param argv
 *
 * @return 
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "chassis_commu_node");
	ros::NodeHandle nh, nh_private("~");
//	signal(SIGINT, signal_quit);
	
	chassis_commu circle_chassis_comm(nh, nh_private);
	/* chassis control and publish odom */
	circle_chassis_comm.poll();

	return 0;
}

