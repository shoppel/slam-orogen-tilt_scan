#include "Task.hpp"

#include <base-logging/Logging.hpp>

using namespace tilt_scan;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}

void Task::trigger_sweep()
{
	trigger = true;
}

void Task::scan_samplesTransformerCallback(const base::Time &ts, const ::base::samples::LaserScan &scan_samples_sample)
{
}

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    // copy configuration 
    mConfiguration = _config.get();
    scan_running = false;

    mSweepStatus.sourceName = mConfiguration.sweep_servo_name;
    return true;
}

bool Task::startHook()
{
	if (! TaskBase::startHook())
		return false;

	//drive initially to min angle
	servoCmd.names.push_back( mConfiguration.sweep_servo_name );
	base::JointState state;
	state.position = mConfiguration.sweep_angle_max;
	state.speed = mConfiguration.sweep_velocity_up;
	servoCmd.elements.push_back( state );
	_tilt_cmd.write( servoCmd );
	lastCmdTime = base::Time::now();

	trigger = false;
	mSweepStatus.counter = 0;
	mSweepStatus.curState = SweepStatus::SweepStatus::SWEEPING_UP;
	_sweep_status.write(mSweepStatus);

	return true;
}

void Task::updateHook()
{
	TaskBase::updateHook();
}

void Task::errorHook()
{
	TaskBase::errorHook();
}

void Task::stopHook()
{
	servoCmd.clear();
	TaskBase::stopHook();
}

void Task::cleanupHook()
{
	TaskBase::cleanupHook();
}
