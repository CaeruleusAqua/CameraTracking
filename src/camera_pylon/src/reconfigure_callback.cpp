#include <reconfigure_callback.h>
#include <config.h>
#include <pylon/gige/BaslerGigEInstantCamera.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include <cv_bridge/cv_bridge.h>

using namespace Basler_GigECameraParams;
using namespace Pylon;
using namespace std;
using namespace cv;



void RosReconfigure_callback(Config &config, uint32_t level) {

	//ExposureTime
	if (config.ExposureAuto == camera_pylon::CameraTracking_Continuous) {
		global.Camera->ExposureAuto.SetValue(ExposureAuto_Continuous);
		cout << "Auto Exposure : " << global.Camera->ExposureAuto.ToString() << " (" << global.Camera->ExposureTimeRaw.GetValue() << ")" << endl;
	}
	if (config.ExposureAuto == camera_pylon::CameraTracking_Once) {
		global.Camera->ExposureAuto.SetValue(ExposureAuto_Once);
		cout << "Auto Exposure : " << global.Camera->ExposureAuto.ToString() << " (" << global.Camera->ExposureTimeRaw.GetValue() << ")" << endl;
	}
	if (config.ExposureAuto == camera_pylon::CameraTracking_Off_) {
		global.Camera->ExposureAuto.SetValue(ExposureAuto_Off);
		if (GenApi::IsWritable(global.Camera->ExposureTimeRaw))
			global.Camera->ExposureTimeRaw.SetValue(config.ExposureTimeAbs);
		cout << "Auto Exposure : " << global.Camera->ExposureAuto.ToString() << " (" << global.Camera->ExposureTimeRaw.GetValue() << ")" << endl;
	}
	config.ExposureTimeAbs = global.Camera->ExposureTimeRaw.GetValue();

	//Gain
	if (config.GainAuto == camera_pylon::CameraTracking_Off_) {
		global.Camera->GainAuto.SetValue(GainAuto_Off);
		if (GenApi::IsWritable(global.Camera->GainRaw))
			global.Camera->GainRaw.SetValue(config.Gain);
		cout << "Auto Gain : " << global.Camera->GainAuto.ToString() << " (" << global.Camera->GainRaw.GetValue() << ")" << endl;

	}
	if (config.GainAuto == camera_pylon::CameraTracking_Once) {
		global.Camera->GainRaw.SetValue(GainAuto_Once);
		cout << "Auto Gain : " << global.Camera->GainAuto.ToString() << " (" << global.Camera->GainRaw.GetValue() << ")" << endl;
	}
	if (config.GainAuto == camera_pylon::CameraTracking_Continuous) {
		global.Camera->GainAuto.SetValue(GainAuto_Continuous);
		cout << "Auto Gain : " << global.Camera->GainAuto.ToString() << " (" << global.Camera->GainRaw.GetValue() << ")" << endl;
	}
	config.Gain = global.Camera->GainRaw.GetValue();

	//AcquisitionMode
	if (config.AcquisitionMode == camera_pylon::CameraTracking_Continuous_) {
		global.Camera->AcquisitionMode.SetValue(AcquisitionMode_Continuous);
		cout << "AcquisitionMode : " << global.Camera->AcquisitionMode.ToString() << endl;
	}
	if (config.AcquisitionMode == camera_pylon::CameraTracking_MultiFrame) {
		global.Camera->AcquisitionMode.SetValue(AcquisitionMode_MultiFrame);
		cout << "AcquisitionMode : " << global.Camera->AcquisitionMode.ToString() << endl;
	}
	if (config.AcquisitionMode == camera_pylon::CameraTracking_SingleFrame) {
		global.Camera->AcquisitionMode.SetValue(AcquisitionMode_SingleFrame);
		cout << "AcquisitionMode : " << global.Camera->AcquisitionMode.ToString() << endl;
	}
	if (global.Camera->AcquisitionMode.GetValue() == AcquisitionMode_Continuous)
		config.AcquisitionMode = camera_pylon::CameraTracking_Continuous_;
	if (global.Camera->AcquisitionMode.GetValue() == AcquisitionMode_MultiFrame)
		config.AcquisitionMode = camera_pylon::CameraTracking_MultiFrame;
	if (global.Camera->AcquisitionMode.GetValue() == AcquisitionMode_SingleFrame)
		config.AcquisitionMode = camera_pylon::CameraTracking_SingleFrame;

	//FrameRate
	global.Camera->AcquisitionFrameRateEnable.SetValue(config.FixedFrameRate);
	if (global.Camera->AcquisitionFrameRateEnable.GetValue()) {
		global.Camera->AcquisitionFrameRateAbs.SetValue(config.FrameRate);
	}
	config.FrameRate = global.Camera->AcquisitionFrameRateAbs.GetValue();
	config.FixedFrameRate = global.Camera->AcquisitionFrameRateEnable.GetValue();
	cout << "Frame Rate    : " << (global.Camera->AcquisitionFrameRateEnable.GetValue() ? "On" : "off") << " ("
			<< global.Camera->AcquisitionFrameRateAbs.GetValue() << ")" << endl;


	global.Camera->StopGrabbing();

	if (config.PixelFormat == camera_pylon::CameraTracking_Bayer_RG8) {
		global.Camera->PixelFormat.SetValue(PixelFormat_BayerRG8);
		global.fc->OutputPixelFormat = PixelType_BGR8packed;
		global.opencv_format = CV_8UC3;
		global.encoding="bgr8";
		cout << "PixelFormat   : " << global.Camera->PixelFormat.ToString() << " (" << global.Camera->PixelFormat.GetValue() << ")" << endl;
	}
	if (config.PixelFormat == camera_pylon::CameraTracking_YUV_422) {
		global.Camera->PixelFormat.SetValue(PixelFormat_YUV422Packed);
		global.fc->OutputPixelFormat = PixelType_BGR8packed;
		global.opencv_format = CV_8UC3;
		global.encoding="bgr8";
		cout << "PixelFormat   : " << global.Camera->PixelFormat.ToString() << " (" << global.Camera->PixelFormat.GetValue() << ")" << endl;
	}
	if (config.PixelFormat == camera_pylon::CameraTracking_Mono_8) {
		global.Camera->PixelFormat.SetValue(PixelFormat_Mono8);
		global.fc->OutputPixelFormat = PixelType_Mono8;
		global.opencv_format = CV_8UC1;
		global.encoding="mono8";
		cout << "PixelFormat   : " << global.Camera->PixelFormat.ToString() << " (" << global.Camera->PixelFormat.GetValue() << ")" << endl;
	}
	global.Camera->StartGrabbing(GrabStrategy_LatestImageOnly, GrabLoop_ProvidedByInstantCamera);

}
