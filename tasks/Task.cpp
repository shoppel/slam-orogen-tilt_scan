#include "Task.hpp"

#include <base-logging/Logging.hpp>

using namespace tilt_scan;

typedef std::vector<base::Point> PointVector;

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
	mTrigger = true;
}

void Task::checkTiltStatus()
{
	base::samples::Joints joints;
	while(_tilt_status.read(joints) == RTT::NewData)
	{
		base::JointState jointState;
		jointState = joints.getElementByName(mConfiguration.sweep_servo_name);
		
		if((mSweepStatus.curState == SweepStatus::SWEEPING_UP) && fabs(jointState.position - mConfiguration.sweep_angle_max) < 0.1)
		{
			mSweepStatus.curState = SweepStatus::REACHED_UP_POSITION;
		}
		
		if((mSweepStatus.curState == SweepStatus::SWEEPING_DOWN) && fabs(jointState.position - mConfiguration.sweep_angle_min) < 0.1)
		{
			mSweepStatus.curState = SweepStatus::SWEEPING_UP;
		}
	}
}

void Task::sendPointcloud()
{
	Eigen::Affine3d odometry2body;
	try
	{
		_odometry2body.get(mLastScanTime, odometry2body, true);
	}catch(std::exception &e)
	{
		LOG_ERROR("%s", e.what());
		return;
	}
	
	base::samples::Pointcloud result;
	for(PointVector::iterator it = mPointcloud.points.begin(); it < mPointcloud.points.end(); ++it)
	{
		result.points.push_back(odometry2body * (*it));
	}
	result.time = mLastScanTime;
	_pointcloud.write(result);
	mPointcloud.points.clear();
}

void Task::scanTransformerCallback(const base::Time &ts, const base::samples::LaserScan &scan)
{
	// Do nothing if sweeping is inactive
	if(mSweepStatus.curState == SweepStatus::NOT_SWEEPING)
	{
		return;
	}
	
	// Get pose of the laser in odometry frame
	mLastScanTime = ts;
	Eigen::Affine3d laser2odometry;
	try
	{
		_laser2odometry.get(ts, laser2odometry, true);
	}catch(std::exception &e)
	{
		LOG_ERROR("%s", e.what());
		return;
	}
	
	// Convert to pointcloud and add to global cloud
	PointVector points;
	scan.convertScanToPointCloud(points, laser2odometry);
	for(PointVector::iterator it = points.begin(); it < points.end(); ++it)
	{
		mPointcloud.points.push_back(*it);
	}
}

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    mConfiguration = _config.get();
    mSweepStatus.sourceName = mConfiguration.sweep_servo_name;
    return true;
}

bool Task::startHook()
{
	if (! TaskBase::startHook())
		return false;

	//drive initially to min angle
	mTiltCommand.names.push_back( mConfiguration.sweep_servo_name );
	base::JointState state;
	state.position = mConfiguration.sweep_angle_max;
	state.speed = mConfiguration.sweep_velocity_up;
	mTiltCommand.elements.push_back( state );
	mTiltCommand.time = base::Time::now();
	_tilt_cmd.write( mTiltCommand );

	mTrigger = false;
	mSweepStatus.counter = 0;
	mSweepStatus.curState = SweepStatus::SWEEPING_UP;
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
	mTiltCommand.clear();
	TaskBase::stopHook();
}

void Task::cleanupHook()
{
	TaskBase::cleanupHook();
}
