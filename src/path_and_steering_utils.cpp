#include "megoldas_gyor24/path_and_steering_utils.h"

#include <algorithm>
#include <cmath>

namespace path_and_steering_utils {

float MapValue(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

std::vector<geometry_msgs::Point> BuildSteeringPoints(
    double speed_cmd,
    double steering_angle,
    double wheelbase) {
    std::vector<geometry_msgs::Point> points;
    double marker_pos_x = 0.0;
    double marker_pos_y = 0.0;
    double theta = 0.0;

    if (speed_cmd < 0) {
        for (int i = 0; i < 10; ++i) {
            marker_pos_x += 0.01 * 10 * std::cos(theta);
            marker_pos_y += 0.01 * 10 * std::sin(theta);
            theta += 0.01 * 10 / wheelbase * std::tan(steering_angle);
            geometry_msgs::Point p;
            p.x = marker_pos_x;
            p.y = marker_pos_y;
            points.push_back(p);
        }
        std::reverse(points.begin(), points.end());

        marker_pos_x = 0.0;
        marker_pos_y = 0.0;
        theta = 0.0;
        for (int i = 0; i < 10 + int(-20 * speed_cmd); ++i) {
            marker_pos_x -= 0.01 * 10 * std::cos(theta);
            marker_pos_y -= 0.01 * 10 * std::sin(theta);
            theta -= 0.01 * 10 / wheelbase * std::tan(steering_angle);
            geometry_msgs::Point p;
            p.x = marker_pos_x;
            p.y = marker_pos_y;
            points.push_back(p);
        }
    } else {
        for (int i = 0; i < 10 + int(20 * speed_cmd); ++i) {
            marker_pos_x += 0.01 * 10 * std::cos(theta);
            marker_pos_y += 0.01 * 10 * std::sin(theta);
            theta += 0.01 * 10 / wheelbase * std::tan(steering_angle);
            geometry_msgs::Point p;
            p.x = marker_pos_x;
            p.y = marker_pos_y;
            points.push_back(p);
        }
    }

    return points;
}

void TrimPath(nav_msgs::Path &path, int path_size) {
    if (path_size <= 0) {
        path.poses.clear();
        return;
    }

    if (path.poses.size() > static_cast<size_t>(path_size)) {
        const int shift = static_cast<int>(path.poses.size()) - path_size;
        path.poses.erase(path.poses.begin(), path.poses.begin() + shift);
    }
}

} 