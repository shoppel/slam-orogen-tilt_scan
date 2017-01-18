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
	// Get current joint state
	base::samples::Joints joints;
	base::JointState jointState;
	bool tilt_ok = false;
	while(_tilt_status_samples.read(joints) == RTT::NewData)
	{
		jointState = joints.getElementByName(mConfiguration.sweep_servo_name);
		tilt_ok = true;
	}
	
	if(!tilt_ok)
	{
		LOG_WARN("Could not get joint status from tilt unit.");
		return;
	}
		
	// Initialized to up posiition
	if((mSweepStatus.curState == SweepStatus::INITIALIZING))
	{
		if(fabs(jointState.position - mConfiguration.sweep_angle_max) < 0.1)
		{
			if(mConfiguration.mode == Configuration::CONTINUOUS_SWEEPING)
			{
				_tilt_cmd.write( mTiltDownCommand );
				mSweepStatus.curState = SweepStatus::SWEEPING_DOWN;
				mPointcloud.points.clear();
			}else
			{
				mSweepStatus.curState = SweepStatus::REACHED_UP_POSITION;
			}
		}else
		{
                        
			_tilt_cmd.write( mTiltUpCommand );
		}
		return;
	}
	
	// Reached upper end point
	if((mSweepStatus.curState == SweepStatus::SWEEPING_UP) && fabs(jointState.position - mConfiguration.sweep_angle_max) < 0.1)
	{
		if(mConfiguration.mode == Configuration::CONTINUOUS_SWEEPING)
		{
			if(mConfiguration.sweep_back_and_forth)
			{
				sendPointcloud();
			}
			mPointcloud.points.clear();
			_tilt_cmd.write( mTiltDownCommand );
			mSweepStatus.curState = SweepStatus::SWEEPING_DOWN;
		}else
		{
			mSweepStatus.curState = SweepStatus::REACHED_UP_POSITION;
		}
		return;
	}
	
	// Reached lower endpoint
	if((mSweepStatus.curState == SweepStatus::SWEEPING_DOWN) && fabs(jointState.position - mConfiguration.sweep_angle_min) < 0.1)
	{
		sendPointcloud();
		_tilt_cmd.write( mTiltUpCommand );
		mSweepStatus.curState = SweepStatus::SWEEPING_UP;
		return;
	}
	
	// Received trigger signal
	if((mSweepStatus.curState == SweepStatus::REACHED_UP_POSITION) && mTrigger)
	{
		mTrigger = false;
		mPointcloud.points.clear();
		_tilt_cmd.write( mTiltDownCommand );
		mSweepStatus.curState = SweepStatus::SWEEPING_DOWN;
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
	mSweepStatus.counter++;
}

void Task::scan_samplesTransformerCallback(const base::Time &ts, const base::samples::LaserScan &scan)
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
	
	// Check sweep and update status
	checkTiltStatus();
	_sweep_status.write(mSweepStatus);
}

bool Task::configureHook()
{
	if (! TaskBase::configureHook())
		return false;

	mConfiguration = _config.get();
	mSweepStatus.sourceName = mConfiguration.sweep_servo_name;
	
        // List of servo names replaces mConfiguration.sweep_servo_name if used.
        std::vector<std::string> servo_names = mConfiguration.sweep_servo_names;
        
	base::JointState state;	
	state.position = mConfiguration.sweep_angle_max;
	state.speed = mConfiguration.sweep_velocity_up;
        
        if(servo_names.size() == 0) {
            mTiltUpCommand.names.push_back( mConfiguration.sweep_servo_name );
            mTiltUpCommand.elements.push_back( state );
        } else {
            std::vector<std::string>::iterator it = servo_names.begin();
            for(; it != servo_names.end(); it++) {
                mTiltUpCommand.names.push_back(*it);
                mTiltUpCommand.elements.push_back(state);
            }
        }
       
	state.position = mConfiguration.sweep_angle_min;
	state.speed = mConfiguration.sweep_velocity_down;
        
        if(servo_names.size() == 0) {
            mTiltDownCommand.names.push_back( mConfiguration.sweep_servo_name );
            mTiltDownCommand.elements.push_back( state );
        } else {
            std::vector<std::string>::iterator it = servo_names.begin();
            for(; it != servo_names.end(); it++) {
                mTiltDownCommand.names.push_back(*it);
                mTiltDownCommand.elements.push_back(state);
            }
        }
	return true;
}

bool Task::startHook()
{
	if (! TaskBase::startHook())
		return false;

	mTrigger = false;
	mSweepStatus.counter = 0;
	mSweepStatus.curState = SweepStatus::INITIALIZING;
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
	mTiltUpCommand.clear();
	mTiltDownCommand.clear();
	TaskBase::stopHook();
}

void Task::cleanupHook()
{
	TaskBase::cleanupHook();
}
