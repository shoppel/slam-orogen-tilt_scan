#ifndef __TILT_SCAN_TYPES_HPP__
#define __TILT_SCAN_TYPES_HPP__

#include <base/Pose.hpp>

namespace tilt_scan
{
struct Configuration
{
    Configuration()
	: max_lines( 200 ),
	max_pose_change( 0.01, 1.0 / 180.0 * M_PI )
    {}

    /** maximum number of lines until the scan is considered complete */
    int max_lines;

    /** maximum pose change until the vehicle is considered moving */
    base::PoseUpdateThreshold max_pose_change;

    /** minimum angle value to use when sweeping in rad */
    float sweep_angle_min;

    /** maximum angle to use when sweeping in rad */
    float sweep_angle_max;

    /** sweep velocity in rad/s */
    float sweep_velocity;

    /** name of the servo to sweep */
    std::string sweep_servo_name;
};

}

#endif
