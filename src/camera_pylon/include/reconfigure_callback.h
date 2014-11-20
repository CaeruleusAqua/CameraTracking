#pragma once
#include <camera_pylon/CameraTrackingConfig.h>
typedef camera_pylon::CameraTrackingConfig Config;

void RosReconfigure_callback(Config &config, uint32_t level);
