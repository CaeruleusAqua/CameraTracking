#pragma once 
#include <pylon/PylonIncludes.h>
#include <pylon/gige/BaslerGigEInstantCamera.h>
#include <image_transport/image_transport.h>
#include <camera_pylon/CameraTrackingConfig.h>
#include <ros/ros.h>
struct global_s
{
	ros::NodeHandle 					   							*phNode;
	image_transport::ImageTransport 								*imageTransporter;
	Pylon::CBaslerGigEInstantCamera 								*Camera;
	image_transport::Publisher 										pub;
	Pylon::CImageFormatConverter 									*fc;
	std::string 													encoding;
	int																opencv_format;

} global;
