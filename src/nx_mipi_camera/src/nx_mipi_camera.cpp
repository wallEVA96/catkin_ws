/*
 * nx_mipi_camera.cpp
 *                        _ _                 
 *         __      ____ _| | | _____   ____ _ 
 *         \ \ /\ / / _` | | |/ _ \ \ / / _` |
 *          \ V  V / (_| | | |  __/\ V / (_| |
 *           \_/\_/ \__,_|_|_|\___| \_/ \__,_|
 * Copyright (C) 2022 screw <screw@xavier-NX>
 *
 * Distributed under terms of the MIT license.
 */

#include "nx_mipi_camera.h"

/**
 * @brief nx_mipi_camera construct
 */
nx_mipi_camera::nx_mipi_camera(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
				:nh(nh)
				,nh_private(nh_private)
				,resource_str("csi://0")
				,codec_str("unknown")
				,flip_str("none")
				,video_width(0)
				,video_height(0)
				,video_loop(0)
				,video_rtspLatency(2000)
				,video_framerate(0)
{
	nh_private.getParam("resource", resource_str);
	nh_private.getParam("codec", codec_str);
	nh_private.getParam("width", video_width);
	nh_private.getParam("height", video_height);
	nh_private.getParam("loop", video_loop);
	nh_private.getParam("rtsplatency", video_rtspLatency);
	nh_private.getParam("framerate", video_framerate);

	if(resource_str.size() != 0 && flip_str.size() != 0)
	{
		video_options.codec = videoOptions::CodecFromStr(codec_str.c_str());
		video_options.flipMethod = videoOptions::FlipMethodFromStr(flip_str.c_str());
	}
	video_options.width = video_width;
	video_options.height = video_height;
	video_options.rtspLatency = video_rtspLatency;
	video_options.Print("/nx_mipi_camera");

	video_stream = videoSource::Create(resource_str.c_str(), video_options);
	if(!video_stream)
	{
		ROS_ERROR("Failed To Open Video Source");
		return ;
	}

	/* create image converter */
	img_cvt = new imageConverter();
	if(!img_cvt)
	{
		ROS_ERROR("Failed To Create ImageConverter");
		return ;	
	}
	/* advertise img raw topic */
	img_pub = nh_private.advertise<sensor_msgs::Image>("raw", 2);
	img_tp = new image_transport::ImageTransport(nh_private);
	img_tp_pub = img_tp->advertise("img_tp_raw", 1);
}

/**
 * @brief 
 */
nx_mipi_camera::~nx_mipi_camera()
{
	delete img_tp;
	delete img_cvt;
	ROS_INFO("Desconstruct Nx MIPI Camera");
}

/**
 * @brief get nx_mipi_camera is init success or not. 
 *
 * @return 0 or 1 
 */
int nx_mipi_camera::initSuccess()
{
	if(video_stream == NULL || img_cvt == NULL)
		return 0;
	
	/* Try To Open camera Stream */
	if(!video_stream->Open())
	{
		ROS_ERROR("Failed To Open Video Source.");
		return 0;
	}

	return 1;
}

/**
 * @brief get frame and publish.
 *
 * @return 
 */
bool nx_mipi_camera::measureFrame()
{
	imageConverter::PixelType *next_frame= NULL;
	
	if(!video_stream->Capture(&next_frame, 1000))
	{
		ROS_ERROR("Failed To Capture Next Frame.");
		return false;
	}
	
	if(!img_cvt->Resize(video_stream->GetWidth(), video_stream->GetHeight(), imageConverter::ROSOutputFormat))
	{
		ROS_ERROR("Failed To Resize Img Cvt");
		return false;	
	}
	
	sensor_msgs::Image img_msg;
	if(!img_cvt->Convert(img_msg, imageConverter::ROSOutputFormat, next_frame))
	{
		ROS_ERROR("Failed To Convert Video Stream");
		return false;
	}
	img_pub.publish(img_msg);
	img_tp_pub.publish(img_msg);
	return true;
}

/**
 * @brief task loop
 */
void nx_mipi_camera::loop()
{
	ros::Rate loop_rate(30);
	while(ros::ok())
	{
		if(!measureFrame())
			ROS_ERROR("Measure Img Frame Error.");

		if(ros::ok())
			ros::spinOnce();
		loop_rate.sleep();
	}
}

/**
 * @brief nx mipi camera main entry.
 *
 * @return 
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "nx_mipi_camera");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	nx_mipi_camera nx_csi0_camera(nh, nh_private);
	if( nx_csi0_camera.initSuccess() == 1)
	{
		nx_csi0_camera.loop();
	}
	else
		ROS_ERROR("NX CSI 0 Camera Init Fail");
}

