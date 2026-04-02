#ifndef MEGOLDAS_GYOR24_PATH_AND_STEERING_UTILS_H
#define MEGOLDAS_GYOR24_PATH_AND_STEERING_UTILS_H

#include <vector>

#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>

namespace path_and_steering_utils {

float MapValue(float x, float in_min, float in_max, float out_min, float out_max);

std::vector<geometry_msgs::Point> BuildSteeringPoints(
    double speed_cmd,
    double steering_angle,
    double wheelbase);

void TrimPath(nav_msgs::Path &path, int path_size);

}  // namespace path_and_steering_utils

#endif