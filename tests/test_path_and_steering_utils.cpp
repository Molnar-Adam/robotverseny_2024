#include <cmath>

#include <gtest/gtest.h>

#include <geometry_msgs/PoseStamped.h>

#include "megoldas_gyor24/path_and_steering_utils.h"

TEST(PathAndSteeringUtilsTest, MapValueMidpoint) {
    const float mapped = path_and_steering_utils::MapValue(0.75f, 0.0f, 1.5f, 0.0f, 10.0f);
    EXPECT_NEAR(mapped, 5.0f, 1e-6f);
}

TEST(PathAndSteeringUtilsTest, BuildSteeringPointsForwardCountAndDirection) {
    const double speed_cmd = 1.0;
    const auto points = path_and_steering_utils::BuildSteeringPoints(speed_cmd, 0.0, 0.3187);

    ASSERT_EQ(points.size(), 10 + int(20 * speed_cmd));
    ASSERT_FALSE(points.empty());
    EXPECT_GT(points.back().x, 0.0);
    EXPECT_NEAR(points.back().y, 0.0, 1e-9);
}

TEST(PathAndSteeringUtilsTest, BuildSteeringPointsReverseCountAndDirection) {
    const double speed_cmd = -0.5;
    const auto points = path_and_steering_utils::BuildSteeringPoints(speed_cmd, 0.0, 0.3187);

    ASSERT_EQ(points.size(), 20 + int(-20 * speed_cmd));
    ASSERT_FALSE(points.empty());
    EXPECT_LT(points.back().x, 0.0);
    EXPECT_NEAR(points.back().y, 0.0, 1e-9);
}

TEST(PathAndSteeringUtilsTest, BuildSteeringPointsAreFiniteForNonZeroSteering) {
    const auto points = path_and_steering_utils::BuildSteeringPoints(0.8, 0.3, 0.3187);

    ASSERT_FALSE(points.empty());
    for (const auto &p : points) {
        EXPECT_TRUE(std::isfinite(p.x));
        EXPECT_TRUE(std::isfinite(p.y));
    }
}

TEST(PathAndSteeringUtilsTest, TrimPathKeepsLastNPoses) {
    nav_msgs::Path path;
    for (int i = 0; i < 6; ++i) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = i;
        path.poses.push_back(pose);
    }

    path_and_steering_utils::TrimPath(path, 3);

    ASSERT_EQ(path.poses.size(), 3u);
    EXPECT_DOUBLE_EQ(path.poses[0].pose.position.x, 3.0);
    EXPECT_DOUBLE_EQ(path.poses[1].pose.position.x, 4.0);
    EXPECT_DOUBLE_EQ(path.poses[2].pose.position.x, 5.0);
}

TEST(PathAndSteeringUtilsTest, TrimPathClearsWhenConfiguredToZero) {
    nav_msgs::Path path;
    geometry_msgs::PoseStamped pose;
    path.poses.push_back(pose);

    path_and_steering_utils::TrimPath(path, 0);

    EXPECT_TRUE(path.poses.empty());
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}