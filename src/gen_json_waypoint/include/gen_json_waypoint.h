/*
 * gen_json_waypoint.h
 * Copyright (C) 2022 screw <screw@xavier-NX>
 *
 * Distributed under terms of the MIT license.
 */

#ifndef GEN_JSON_WAYPOINT_H
#define GEN_JSON_WAYPOINT_H

#include <iostream>
#include <string>
#include <fstream> 
#include <nlohmann/json.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

using json = nlohmann::json;


class genJsonWaypoint
{
public:
	genJsonWaypoint(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
	~genJsonWaypoint();
	bool loadWaypointByJson(void);
	std::string json_raw_path;
	std::ifstream js_file;
	json raw_json_data;
//	bool setGoalID(std::string &goal_id, std::string id_prefix = "waypoint_");
	inline bool sendCurrGoal(void);
	uint64_t jumpToNextGoalIndex(void);
	void wayPointScheduling(void);
	void wayPointDoneCallback(const actionlib::SimpleClientGoalState &goal_sta,const move_base_msgs::MoveBaseResult::ConstPtr &result_ptr);
	void wayPointFeedbackCallback(const move_base_msgs::MoveBaseFeedback::ConstPtr &result_ptr);

private:
	//move_base_msgs::MoveBaseActionGoal curr_action_goal;
	move_base_msgs::MoveBaseGoal curr_move_base_goal;
	std::vector<geometry_msgs::Pose> poseVec;
	std::string goal_frame;
	uint64_t curr_waypoint_index;
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client;
};

#endif /* !GEN_JSON_WAYPOINT_H */
