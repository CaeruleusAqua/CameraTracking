// Grab_UsingGrabLoopThread.cpp
/*
 This sample illustrates how to grab and process images using the grab loop thread
 provided by the Instant Camera class.
 */

// Include files to use the PYLON API.
#include <pylon/PylonIncludes.h>
#include <unistd.h>
#include <sys/time.h>

// Include files used by samples.
#include <pylon/ImageFormatConverter.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

#include <pylon/gige/BaslerGigEInstantCamera.h>

typedef Pylon::CBaslerGigEInstantCamera Camera_t;
using namespace cv;
using namespace Basler_GigECameraParams;
using namespace Pylon;
using namespace std;

struct timeval speed;
double fps = 0;
CPylonImage image;
CImageFormatConverter fc;

long getTimeDiff(timeval begin, timeval end) {
	long seconds = end.tv_sec - begin.tv_sec;
	long useconds = end.tv_usec - begin.tv_usec;
	useconds += seconds * 1000000L;
	return useconds;
}
class CSampleImageEventHandler: public CImageEventHandler {
public:
	virtual void OnImageGrabbed(CInstantCamera& camera, const CGrabResultPtr& ptrGrabResult) {
//		fc.OutputPixelFormat = PixelType_BGR8packed;
//		if (ptrGrabResult->GrabSucceeded()) {
//		fc.Convert(image, ptrGrabResult);
//		std::cout << image.GetHeight()<<endl;;
//			Mat cv_img = cv::Mat(ptrGrabResult->GetHeight(),
//					ptrGrabResult->GetWidth(), CV_8UC3,
//					(uint8_t*) image.GetBuffer());
//			Size size(700,700);//the dst image size,e.g.100x100
//			//resize(cv_img,cv_img,size);//resize image
//			imshow("CV_Image", cv_img);
//			waitKey(1);
		struct timeval tmp;
		gettimeofday(&tmp, (struct timezone *) 0);
		if (fps == 0)
			fps = (((double) (getTimeDiff(speed, tmp))) / 1000000.);
		else
			fps = (fps * 0.9) + (0.1 * (1. / (((double) (getTimeDiff(speed, tmp))) / 1000000.)));
		cout << "fps:" << fps << std::endl;
		speed = tmp;
		}
//	}
};

int main(int argc, char* argv[]) {
	int exitCode = 0;
	CGrabResultPtr ptrGrabResult;
	gettimeofday(&speed, (struct timezone *) 0);
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
	PixelFormatEnums oldPixelFormat = camera.PixelFormat.GetValue();
	cout << "Old PixelFormat : " << camera.PixelFormat.ToString() << " (" << oldPixelFormat << ")" << endl;

	// Set pixel format
	if (GenApi::IsAvailable(camera.PixelFormat.GetEntry(PixelFormat_BayerRG8))) {
		camera.PixelFormat.SetValue(PixelFormat_BayerRG8);
		cout << "New PixelFormat : " << camera.PixelFormat.ToString() << " (" << camera.PixelFormat.GetValue() << ")" << endl;
	}

	try {

		camera.RegisterConfiguration(new CAcquireContinuousConfiguration, RegistrationMode_ReplaceAll, Cleanup_Delete);
		camera.RegisterImageEventHandler(new CSampleImageEventHandler, RegistrationMode_Append, Cleanup_Delete);
		camera.MaxNumBuffer = 10;

		camera.StartGrabbing(GrabStrategy_LatestImageOnly, GrabLoop_ProvidedByInstantCamera);
		do {
			camera.WaitForFrameTriggerReady(100, TimeoutHandling_ThrowException);
			sleep(10);

		} while (true);

	} catch (GenICam::TimeoutException &e) {

		cerr << "An exception occurred." << endl << e.GetDescription() << endl;
		exitCode = 1;
	}
	return exitCode;
}

