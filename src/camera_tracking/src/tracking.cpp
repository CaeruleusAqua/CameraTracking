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
#include <reconfigure_callback.h>

typedef Pylon::CBaslerGigEInstantCamera Camera_t;
using namespace cv;
using namespace Basler_GigECameraParams;
using namespace Pylon;
using namespace std;

CPylonImage image;
CImageFormatConverter fc;
ros::NodeHandle *nh;
image_transport::ImageTransport *it;
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
		fc.OutputPixelFormat = PixelType_Mono8;
		if (ptrGrabResult->GrabSucceeded()) {
			fc.Convert(image, ptrGrabResult);
			Mat cv_img = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC1, (uint8_t*) image.GetBuffer());
            //Mat temp = cv_img.clone();
            //double tmp[] = {-0.0071051186191983485, 0.010475801567272592, -0.002036391860118614, 0.0015917568818403511, 0.0 };
            //std::vector<double> dis(tmp, tmp + sizeof tmp / sizeof tmp[0]);
            //Matx33d m(809.7768378862996, 0.0, 762.2499917249673, 0.0, 809.7990091872273, 570.3036262589774, 0.0, 0.0, 1.0);
            //undistort(temp,cv_img,m,dis);
			//imshow("CV_Image", cv_img);
			sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", cv_img).toImageMsg();
            pub.publish(msg);
			//Size size(700,700);//the dst image size,e.g.100x100
			//resize(cv_img,cv_img,size);//resize image

			//waitKey(1);
		}
	}

};

int main(int argc, char* argv[]) {
	cout << "Camera Device Information" << endl << "=========================" << endl;
	ros::init(argc, argv, "pylon_driver_node");

	int exitCode = 0;
	CGrabResultPtr ptrGrabResult;
	Pylon::PylonAutoInitTerm autoInitTerm;

	Camera_t camera(CTlFactory::GetInstance().CreateFirstDevice());
	camera.Open();

	// Get camera device information.
	cout << "Camera Device Information" << endl << "=========================" << endl;
	cout << "Vendor : " << camera.DeviceVendorName.GetValue() << endl;
	cout << "Model  : " << camera.DeviceModelName.GetValue() << endl;
	cout << "Firmware version : " << camera.DeviceFirmwareVersion.GetValue() << endl << endl;

	// Camera settings.
	cout << "Camera Device Settings" << endl << "======================" << endl;
	cout << "OffsetX : " << camera.OffsetX.GetValue() << endl;
	cout << "OffsetY : " << camera.OffsetY.GetValue() << endl;
	cout << "Width   : " << camera.Width.GetValue() << endl;
	cout << "Height  : " << camera.Height.GetValue() << endl;
	camera.Width.SetValue(1600);
	camera.Height.SetValue(1200);
	PixelFormatEnums oldPixelFormat = camera.PixelFormat.GetValue();
	cout << "Old PixelFormat : " << camera.PixelFormat.ToString() << " (" << oldPixelFormat << ")" << endl;

	if (GenApi::IsAvailable(camera.PixelFormat.GetEntry(PixelFormat_Mono8))) {
		camera.PixelFormat.SetValue(PixelFormat_Mono8);
		cout << "New PixelFormat : " << camera.PixelFormat.ToString() << " (" << camera.PixelFormat.GetValue() << ")" << endl;
	}

	camera.GainAuto.SetValue(GainAuto_Continuous);
	camera.ExposureAuto.SetValue(ExposureAuto_Off);
	if (GenApi::IsWritable(camera.ExposureTimeRaw))
		camera.ExposureTimeRaw.SetValue(14000);
	else
		cout << "not::settet" << endl;

	camera.ShutterMode.SetValue(ShutterMode_Rolling);
    //camera.AcquisitionFrameRateEnable.SetValue( true );
    //camera.AcquisitionFrameRateAbs.SetValue( 20 );

	dynamic_reconfigure::Server<Config> reconfigureServer;
	dynamic_reconfigure::Server<Config>::CallbackType reconfigureCallback;
	reconfigureCallback = boost::bind(&RosReconfigure_callback, _1, _2);
	reconfigureServer.setCallback(reconfigureCallback);

	nh = new ros::NodeHandle();

	it = new image_transport::ImageTransport(*nh);
	pub = (*it).advertise("camera/image", 1);
	ros::Duration(2.0).sleep();

	try {

		camera.RegisterConfiguration(new CAcquireContinuousConfiguration, RegistrationMode_ReplaceAll, Cleanup_Delete);
		camera.RegisterImageEventHandler(new CSampleImageEventHandler, RegistrationMode_Append, Cleanup_Delete);
		camera.MaxNumBuffer = 10;

		camera.StartGrabbing(GrabStrategy_LatestImageOnly, GrabLoop_ProvidedByInstantCamera);
		ros::spin();
	} catch (GenICam::TimeoutException &e) {

		cerr << "An exception occurred." << endl << e.GetDescription() << endl;
		exitCode = 1;
	}
	return exitCode;
}

