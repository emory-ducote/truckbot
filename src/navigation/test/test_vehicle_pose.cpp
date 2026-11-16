// Unit tests for VehiclePose::compose: chaining global->odom and odom->base in 2D.

#include <gtest/gtest.h>

#include <cmath>

#include "VehiclePose.h"

namespace {

using navigation::VehiclePose;

constexpr double kTol = 1e-9;

// Identity parent passes the local pose through unchanged.
TEST(VehiclePoseComposeTest, IdentityParentPassesThrough) {
    auto pose = VehiclePose(0.0, 0.0, 0.0).compose(VehiclePose(1.5, -2.0, 0.3));
    EXPECT_NEAR(pose.x, 1.5, kTol);
    EXPECT_NEAR(pose.y, -2.0, kTol);
    EXPECT_NEAR(pose.theta, 0.3, kTol);
}

// A translated parent offsets the pose.
TEST(VehiclePoseComposeTest, TranslatedParentOffsetsPose) {
    auto pose = VehiclePose(2.0, 3.0, 0.0).compose(VehiclePose(1.0, 1.0, 0.5));
    EXPECT_NEAR(pose.x, 3.0, kTol);
    EXPECT_NEAR(pose.y, 4.0, kTol);
    EXPECT_NEAR(pose.theta, 0.5, kTol);
}

// A rotated parent rotates the local position before offsetting it.
TEST(VehiclePoseComposeTest, RotatedParentRotatesPosition) {
    auto pose = VehiclePose(1.0, 0.0, M_PI / 2.0).compose(VehiclePose(2.0, 0.0, 0.0));
    EXPECT_NEAR(pose.x, 1.0, kTol);
    EXPECT_NEAR(pose.y, 2.0, kTol);
    EXPECT_NEAR(pose.theta, M_PI / 2.0, kTol);
}

// Yaws that sum past pi wrap back into [-pi, pi].
TEST(VehiclePoseComposeTest, YawWrapsAroundPi) {
    auto pose = VehiclePose(0.0, 0.0, 3.0).compose(VehiclePose(0.0, 0.0, 0.5));
    EXPECT_NEAR(pose.theta, 3.5 - 2.0 * M_PI, kTol);
}

}
