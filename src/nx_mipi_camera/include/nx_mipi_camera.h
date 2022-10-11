/*
 * nx_mipi_camera.h
 *                        _ _                 
 *         __      ____ _| | | _____   ____ _ 
 *         \ \ /\ / / _` | | |/ _ \ \ / / _` |
 *          \ V  V / (_| | | |  __/\ V / (_| |
 *           \_/\_/ \__,_|_|_|\___| \_/ \__,_|
 * Copyright (C) 2022 screw <screw@xavier-NX>
 *
 * Distributed under terms of the MIT license.
 */


#ifndef __NX_MIPI_CAMERA_H_
#define __NX_MIPI_CAMERA_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <vision_msgs/Classification2D.h>
#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/VisionInfo.h>
#include <jetson-utils/videoSource.h>
#include "image_converter.h"
#include "detectNet.h"

/**
 * @brief nvidia mipi camera application.
 */
class nx_mipi_camera{

public:
	nx_mipi_camera(ros::NodeHandle &, ros::NodeHandle &);
	~nx_mipi_camera();
	int initSuccess();
	bool measureFrame();
	void loop();
	ros::NodeHandle nh;
	ros::NodeHandle nh_private;
	ros::Publisher img_pub;
	image_transport::ImageTransport img_tp;
	image_transport::ImageTransport *p_img_tp;
	image_transport::Publisher img_tp_pub;
	/* detectnet*/
	detectNet *net;
	detectNet::Detection *detections = NULL;
	const uint32_t overlayFlags;
private:
	videoOptions video_options;
	videoSource *video_stream;
	imageConverter *p_img_cvt;
	std::string resource_str;
	std::string codec_str;
	std::string flip_str;
	int video_width;
	int video_height;
	int video_loop;
	int video_rtspLatency;
	float video_framerate;

};


#endif

