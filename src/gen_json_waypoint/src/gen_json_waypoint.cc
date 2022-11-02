/*
 * gen_json_waypoint.cc
 *                        _ _                 
 *         __      ____ _| | | _____   ____ _ 
 *         \ \ /\ / / _` | | |/ _ \ \ / / _` |
 *          \ V  V / (_| | | |  __/\ V / (_| |
 *           \_/\_/ \__,_|_|_|\___| \_/ \__,_|
 * Copyright (C) 2022 screw <screw@xavier-NX>
 *
 * Distributed under terms of the MIT license.
 */

#include "gen_json_waypoint.h"

/**
 * @brief genJsonWaypoint
 *
 * @param nh
 * @param nh_private
 */
genJsonWaypoint::genJsonWaypoint(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
								:curr_waypoint_index(0)
								,goal_frame("map")
								,move_base_client("move_base", true)
								,json_raw_path("/home/screw/screw_ws/src/gen_json_waypoint/files/multi_waypoint.json")
								,js_file(json_raw_path.c_str())

{
	loadWaypointByJson();
}

/**
 * @brief  decontructure json waypoint.
 */
genJsonWaypoint::~genJsonWaypoint()
{	
	if(js_file.is_open())
		js_file.close();
	ros::shutdown();
}
/**
 * @brief load Waypoint By Json File.
 *
 * @param path_str
 */
bool genJsonWaypoint::loadWaypointByJson()
{
	if(!js_file.is_open())
	{
		ROS_ERROR("Open Waypoint Json File Error");
		return false;
	}
	else
	{
		raw_json_data = json::parse(js_file);
		if(raw_json_data.is_object())
		{
			ROS_INFO("Get A Json Array.");
			json multi_wp_obj = raw_json_data["multi_waypoint"];
			size_t multi_wp_obj_size = multi_wp_obj.size();
			//ROS_INFO("%s", multi_wp_obj.dump(-1).c_str());
			for(int i = 0; i < multi_wp_obj_size; i++)
			{
				json i_obj = multi_wp_obj.at(i);
				ROS_INFO("Load Waypoint: %s", i_obj.dump(-1).c_str());
				geometry_msgs::Pose tmp_waypoint;
				/* get object from array */
				json point_obj = i_obj["Point"];
				double p_x = point_obj["x"].get<double>();
				double p_y = point_obj["y"].get<double>();
				double p_z = point_obj["z"].get<double>();
				json quat_obj  = i_obj["Quaternion"];
				double q_x = quat_obj["x"].get<double>();
				double q_y = quat_obj["y"].get<double>();
				double q_z = quat_obj["z"].get<double>();
				double q_w = quat_obj["w"].get<double>();
				tmp_waypoint.position.x = p_x;
				tmp_waypoint.position.y = p_y;
				tmp_waypoint.position.z = p_z;
				tmp_waypoint.orientation.x = q_x;
				tmp_waypoint.orientation.y = q_y;
				tmp_waypoint.orientation.z = q_z;
				tmp_waypoint.orientation.w = q_w;
				poseVec.push_back(tmp_waypoint);
				curr_waypoint_index = i;
			/*
			    ROS_INFO("%lf %lf %lf %lf %lf %lf %lf", tmp_waypoint.position.x,
                                            tmp_waypoint.position.y, 
                                            tmp_waypoint.position.z, 
                                            tmp_waypoint.orientation.x,
                                            tmp_waypoint.orientation.y,
                                            tmp_waypoint.orientation.z,
			                                tmp_waypoint.orientation.w);
			*/
			}
		}
		else
		{
			ROS_INFO("Json Obejct is Illegal");
			return false;
		}
		return true;
	}
}

/**
 * @brief set goal id
 *
 * @param goal_id
 * @param id_prefix
 *
 * @return 
bool genJsonWaypoint::setGoalID(std::string &goal_id, std::string id_prefix)
{
	if(curr_waypoint_index >= 0 && curr_waypoint_index < poseVec.size())
	{
		goal_id = id_prefix + std::to_string(curr_waypoint_index);
		ROS_INFO("Goal ID: %s", goal_id.c_str());
		return true;
	}
	else
	{
		ROS_INFO("Exceed Waypoint Size");
		return false;
	}
}
 */

/**
 * @brief callback for waypoint complete.
 *
 * @param goal_sta
 * @param result_ptr
 */
void genJsonWaypoint::wayPointDoneCallback(const actionlib::SimpleClientGoalState &goal_sta,
										   const move_base_msgs::MoveBaseResult::ConstPtr &result_ptr)
{
	ROS_INFO("Goal ID Complete.");
}

/**
 * @brief wayPoint Feedback Callback
 *
 * @param feedback_ptr
 */
void genJsonWaypoint::wayPointFeedbackCallback(const move_base_msgs::MoveBaseFeedback::ConstPtr &feedback_ptr)
{
	ROS_INFO("Goal ID Feebback.");
}

/**
 * @brief set new action goal to curr action goal.
 *
 * @param action_goal
 *
 * @return 
 */
bool genJsonWaypoint::sendCurrGoal(void)
{
	ros::Time curr_tm = ros::Time::now();
	curr_move_base_goal.target_pose.header.stamp = curr_tm;
	curr_move_base_goal.target_pose.header.frame_id = goal_frame;
	curr_move_base_goal.target_pose.pose = poseVec[curr_waypoint_index];

//	move_base_client.sendGoal(curr_move_base_goal, 
//							  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>::SimpleDoneCallback(),
//							  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>::SimpleActiveCallback(),
//							  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>::SimpleFeedbackCallback());
	move_base_client.sendGoal(curr_move_base_goal,
							  boost::bind(&genJsonWaypoint::wayPointDoneCallback, this, _1, _2),
							  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>::SimpleActiveCallback(),
							  boost::bind(&genJsonWaypoint::wayPointFeedbackCallback, this, _1)
							 );
	ROS_INFO("Send A New Goal.");
}

/**
 * @brief  set next goal index.
 *
 * @return goal id;
 */
uint64_t genJsonWaypoint::jumpToNextGoalIndex()
{
	curr_waypoint_index++;
	if(curr_waypoint_index == poseVec.size())
		curr_waypoint_index = 0;
	return curr_waypoint_index;
}

/**
 * @brief waypoint scheduling.
 */
void genJsonWaypoint::wayPointScheduling(void)
{
	ROS_INFO("Wait For Move Base Server.");
	move_base_client.waitForServer();
	if(move_base_client.isServerConnected())
		ROS_INFO("Connected To Move Base Server.");
	else
	{
		ROS_INFO("Can't Connect To Server.");
		return ;
	}
	sendCurrGoal();

	while(ros::ok())
	{
		move_base_client.waitForResult(ros::Duration(1.0));
		ROS_INFO("Move Base State: %s", move_base_client.getState().toString().c_str());
		if(move_base_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			jumpToNextGoalIndex();
			sendCurrGoal();
		}
		else
			ROS_INFO("Unable To Get Goal State");
	}
}

/**
 * @brief main entry
 *
 * @param argc
 * @param argv
 *
 * @return 
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "gen_json_waypoint");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	genJsonWaypoint gen_multi_waypoint(nh, nh_private);	
	gen_multi_waypoint.wayPointScheduling();

	return 0;
}

