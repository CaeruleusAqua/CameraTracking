// Grab_UsingGrabLoopThread.cpp
/*
 This sample illustrates how to grab and process images using the grab loop thread
 provided by the Instant Camera class.
 */

// Include files to use the PYLON API.
#include <pylon/PylonIncludes.h>
#include <unistd.h>
#include <sys/time.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// Include files used by samples.
#include <pylon/ImageFormatConverter.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

#include <pylon/gige/BaslerGigEInstantCamera.h>
#include <config.h>
#include <tracking.h>
#include <reconfigure_callback.h>

using namespace cv;
using namespace Basler_GigECameraParams;
using namespace Pylon;
using namespace std;

image_transport::Publisher pub;

long getTimeDiff(timeval begin, timeval end) {
	long seconds = end.tv_sec - begin.tv_sec;
	long useconds = end.tv_usec - begin.tv_usec;
	useconds += seconds * 1000000L;
	return useconds;
}
class CSampleImageEventHandler: public CImageEventHandler {
public:
	virtual void OnImageGrabbed(CInstantCamera& camera, const CGrabResultPtr& ptrGrabResult) {

		if (ptrGrabResult->GrabSucceeded()) {
			CPylonImage image;
			global.fc->Convert(image, ptrGrabResult);
			Mat cv_img = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), global.opencv_format, (uint8_t*) image.GetBuffer());
			sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), global.encoding, cv_img).toImageMsg();
			pub.publish(msg);
			if (global.display) {
				Size size(700, 700); //the dst image size,e.g.100x100
				resize(cv_img, cv_img, size); //resize image
				imshow("CV_Image", cv_img);
				waitKey(1);
			}
			else
				destroyWindow("CV_Image");

		}
	}

};

void setDefault() {
	global.fc = new Pylon::CImageFormatConverter();
	global.Camera->OffsetX.SetValue(0);
	global.Camera->OffsetY.SetValue(0);
	global.Camera->Width.SetValue(1600);
	global.Camera->Height.SetValue(1200);
	if (GenApi::IsAvailable(global.Camera->PixelFormat.GetEntry(PixelFormat_Mono8)))
		global.Camera->PixelFormat.SetValue(PixelFormat_Mono8);
	global.Camera->GainAuto.SetValue(GainAuto_Continuous);
	global.Camera->ExposureAuto.SetValue(ExposureAuto_Off);
	if (GenApi::IsWritable(global.Camera->ExposureTimeRaw))
		global.Camera->ExposureTimeRaw.SetValue(14000);
	global.Camera->ShutterMode.SetValue(ShutterMode_Rolling);
	global.Camera->AcquisitionFrameRateEnable.SetValue(false);
	global.Camera->AcquisitionFrameRateAbs.SetValue(20);
	global.fc->OutputPixelFormat = PixelType_Mono8;
	global.opencv_format = CV_8UC1;
	global.encoding = "mono8";

}

int main(int argc, char* argv[]) {
	cout << "Camera Device Information" << endl << "=========================" << endl;
	ros::init(argc, argv, "pylon_driver_node");

	int exitCode = 0;
	CGrabResultPtr ptrGrabResult;
	Pylon::PylonAutoInitTerm autoInitTerm;

	global.Camera = new CBaslerGigEInstantCamera(CTlFactory::GetInstance().CreateFirstDevice());
	global.Camera->Open();

	// Get camera device information.
	cout << "Camera Device Information" << endl << "=========================" << endl;
	cout << "Vendor : " << global.Camera->DeviceVendorName.GetValue() << endl;
	cout << "Model  : " << global.Camera->DeviceModelName.GetValue() << endl;
	cout << "Firmware version : " << global.Camera->DeviceFirmwareVersion.GetValue() << endl << endl;

	// Camera settings.
	cout << "Set Camera default Settings" << endl << "======================" << endl;
	setDefault();
	cout << "OffsetX       : " << global.Camera->OffsetX.GetValue() << endl;
	cout << "OffsetY       : " << global.Camera->OffsetY.GetValue() << endl;
	cout << "Width         : " << global.Camera->Width.GetValue() << endl;
	cout << "Height        : " << global.Camera->Height.GetValue() << endl;
	cout << "PixelFormat   : " << global.Camera->PixelFormat.ToString() << " (" << global.Camera->PixelFormat.GetValue() << ")" << endl;
	cout << "Auto Exposure : " << global.Camera->ExposureAuto.ToString() << " (" << global.Camera->ExposureTimeRaw.GetValue() << ")" << endl;
	cout << "Auto Gain : " << global.Camera->GainAuto.ToString() << " (" << global.Camera->GainRaw.GetValue() << ")" << endl;
	cout << "Shutter Mode  : " << global.Camera->ShutterMode.ToString() << endl;
	cout << "Frame Rate    : " << (global.Camera->AcquisitionFrameRateEnable.GetValue() ? "On" : "off") << " ("
			<< global.Camera->AcquisitionFrameRateAbs.GetValue() << ")" << endl;

	global.phNode = new ros::NodeHandle();
	global.imageTransporter = new image_transport::ImageTransport(*global.phNode);
	pub = (*global.imageTransporter).advertise("camera/image", 1);
	ros::Duration(1.0).sleep();

	try {

		global.Camera->RegisterConfiguration(new CAcquireContinuousConfiguration, RegistrationMode_ReplaceAll, Cleanup_Delete);
		global.Camera->RegisterImageEventHandler(new CSampleImageEventHandler, RegistrationMode_Append, Cleanup_Delete);
		global.Camera->MaxNumBuffer = 10;
		global.Camera->StartGrabbing(GrabStrategy_LatestImageOnly, GrabLoop_ProvidedByInstantCamera);
		//doing ROS-things
		dynamic_reconfigure::Server<Config> reconfigureServer;
		dynamic_reconfigure::Server<Config>::CallbackType reconfigureCallback;
		reconfigureCallback = boost::bind(&RosReconfigure_callback, _1, _2);
		reconfigureServer.setCallback(reconfigureCallback);
		ros::spin();
	} catch (GenICam::TimeoutException &e) {

		cerr << "An exception occurred." << endl << e.GetDescription() << endl;
		exitCode = 1;
	}
	return exitCode;
}

