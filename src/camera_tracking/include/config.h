#pragma once 

struct global_s
{
	int         							xRoi;
	int         							yRoi;
	int         							widthRoi;
	int										widthRoiMin;
	int										widthRoiMax;
	int         							heightRoi;
	int										heightRoiMin;
	int										heightRoiMax;

	int                                     widthSensor;
	int                                     heightSensor;

	const char                             *pszPixelformat;
	unsigned								nBytesPixel;
	ros::NodeHandle 					   *phNode;
	ArvCamera 							   *pCamera;
	ArvDevice 							   *pDevice;
	int										mtu;
	int										Acquire;
	const char							   *keyAcquisitionFrameRate;

} global;
