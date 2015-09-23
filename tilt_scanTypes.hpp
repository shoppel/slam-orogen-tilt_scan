#ifndef __TILT_SCAN_TYPES_HPP__
#define __TILT_SCAN_TYPES_HPP__

#include <base/Pose.hpp>
#include <stdint.h>

namespace tilt_scan
{
struct Configuration
{
    Configuration()
	: mode(CONTINUOUS_SWEEPING),
	max_lines( 200 ),
	max_pose_change( 0.01, 1.0 / 180.0 * M_PI ),
	min_sweep_velocity( 0.0 )
    {}
    
    enum Mode {
	CONTINUOUS_SWEEPING,
	GROUND_BASED_SWEEPING
    };
    
    /** current mode */
    Mode mode;

    /** maximum number of lines until the scan is considered complete */
    int max_lines;

    /** maximum pose change until the vehicle is considered moving */
    base::PoseUpdateThreshold max_pose_change;

    /** minimum angle value to use when sweeping in rad */
    float sweep_angle_min;

    /** maximum angle to use when sweeping in rad */
    float sweep_angle_max;

    /** sweep velocity up in rad/s */
    float sweep_velocity_up;
    
    /** sweep velocity down in rad/s. Note the point cloud is only generated while sweeping down */
    float sweep_velocity_down;

    /** name of the servo to sweep */
    std::string sweep_servo_name;
    
    /** minimum sweep velocity in rad/s the servo can handle */
    float min_sweep_velocity;
    
    /** sweep velocity in meter/s on ground */
    float sweep_velocity_on_ground;
    
    /** maximum ground distance for which a sweep velocity is computed */
    float max_distance_on_ground;
    
    /** distance between sensor and ground */
    float distance_sensor2ground;
};

    struct SweepStatus
    {
        SweepStatus() : counter(0), curState(NOT_SWEEPING)
        {};
        
        enum State {
            NOT_SWEEPING,
            SWEEPING_UP,
            SWEEPING_DOWN,
        };
        ///counter of the sweeps, wraps at 255
        uint8_t counter;
        
        ///current state of sweeping
        State curState;
        
        ///name of the sweeping device
        std::string sourceName;
        
        /**
         * Return true if one ore more sweeps
         * in respect to the given state werde done.
         * 
         * Also returns true, if not sweeping at all.
         * */
        bool isNextSweep(SweepStatus lastState)
        {
            return NOT_SWEEPING || lastState.counter != counter;
        }
    };

}

#endif
