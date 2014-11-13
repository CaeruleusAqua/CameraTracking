#pragma once
#include <camera_tracking/CameraTrackingConfig.h>
typedef camera_tracking::CameraTrackingConfig Config;

void RosReconfigure_callback(Config &config, uint32_t level);
