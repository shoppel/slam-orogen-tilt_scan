/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <envire/maps/LaserScan.hpp>
#include <envire/maps/TriMesh.hpp>
#include <envire/operators/ScanMeshing.hpp>
#include <orocos/envire/Orocos.hpp>
#include <base/Logging.hpp>
#include <base/samples/Joints.hpp>

using namespace tilt_scan;
using namespace envire;

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

void Task::triggerSweep()
{
    trigger = true;
}

bool Task::handleSweep()
{
    if(config.mode != Configuration::TRIGGERED_SWEEPING)
    {
        trigger = true;
    }
    
    if( _tilt_cmd.connected() )
    {
        base::samples::Joints jointStatus;
        base::JointState state;
        bool gotCmd = false;
        while(_tilt_status_samples.read(jointStatus) == RTT::NewData)
	{
	    state = jointStatus.getElementByName(config.sweep_servo_name);
	    
	    if(fabs(state.position - config.sweep_angle_max) < 0.1)
	    {
                status.curState = SweepStatus::REACHED_UP_POSITION;
	    }
	    
	    if(fabs(state.position - config.sweep_angle_min) < 0.1)
	    {
                servoCmd.elements[0].position = config.sweep_angle_max;
                servoCmd.elements[0].speed = config.sweep_velocity_up;
                gotCmd = true;
                if(this->status.curState != SweepStatus::SWEEPING_UP)
                    this->status.counter++;
                
                this->status.curState = SweepStatus::SWEEPING_UP;
	    }
	}

	if(status.curState == SweepStatus::REACHED_UP_POSITION && trigger)
        {
            servoCmd.elements[0].position = config.sweep_angle_min;
            servoCmd.elements[0].speed = config.sweep_velocity_down;
            gotCmd = true;
            this->status.curState = SweepStatus::SWEEPING_DOWN;
            trigger = false;
        }
        // adapt sweeping speed
        if((config.mode == Configuration::GROUND_BASED_SWEEPING) && (this->status.curState == SweepStatus::SWEEPING_DOWN))
        {
            // get dynamixel2body frame
            Eigen::Affine3d lower_dynamixel2body;
            if( !_lower_dynamixel2body.get( jointStatus.time, lower_dynamixel2body ) )
                return false;

            // compute the absolute tilt angle in [0,PI/2]
            base::Angle tilt_angle = base::Angle::fromRad(state.position) + base::Angle::fromRad(base::getPitch(Eigen::Quaterniond(lower_dynamixel2body.linear())));
            double abs_tilt_angle = std::abs(std::abs(tilt_angle.getRad()) - M_PI_2);
            // compute intersection with ground plane
            double ground_intersection = config.distance_sensor2ground * tan(abs_tilt_angle);
            
            if( !base::isNaN<double>( ground_intersection ) &&
                ground_intersection <= config.max_distance_on_ground)
            {
                if(ground_intersection > config.sweep_velocity_on_ground)
                {
                    double next_tilt_angle = atan2( ground_intersection - config.sweep_velocity_on_ground, config.distance_sensor2ground );
                    servoCmd.elements[0].speed = abs_tilt_angle - next_tilt_angle;
                }
                else
                {
                    servoCmd.elements[0].speed = atan2( config.sweep_velocity_on_ground, config.distance_sensor2ground );
                }
                
                // limit minimal servo speed
                if(servoCmd.elements[0].speed < config.min_sweep_velocity)
                    servoCmd.elements[0].speed = config.min_sweep_velocity;
            }
            gotCmd = true;
        }

        base::Time curTime = base::Time::now();
        if(gotCmd || curTime - lastCmdTime > base::Time::fromMilliseconds(100))
        {

            servoCmd.time = curTime;
            lastCmdTime = curTime;
            _tilt_cmd.write( servoCmd );
            _sweep_status.write(this->status);
        }
        return true;
    }

    return false;
}

void Task::resetEnv()
{
    // generate new environment
    env = boost::shared_ptr<envire::Environment>( new envire::Environment() );

    // as well as a merge operator has the pointcloud as output
    mergeOp = new MergePointcloud();
    env->attachItem( mergeOp );
}

void Task::addScanLine( const ::base::samples::LaserScan &scan, const Eigen::Affine3d& laser2Odometry )
{
    lastScanTime = scan.time;

    Pointcloud *cloud = new Pointcloud();
    
    scan.convertScanToPointCloud(cloud->vertices, laser2Odometry);
    env->setFrameNode(cloud, env->getRootNode());

    // add to merge operator
    mergeOp->addInput( cloud );
}

void Task::writePointcloud(const Eigen::Affine3d& body2odometry )
{
    LOG_DEBUG_S << "got a pointcloud!" << std::endl;

    // set-up new structure
    // Framenode for the body to odometry
    envire::FrameNode *pcFrame = new envire::FrameNode( body2odometry );
    env->getRootNode()->addChild( pcFrame );

    // and a pointlcoud with the final scan
    envire::Pointcloud *targetPointcloud = new envire::Pointcloud();
    env->setFrameNode( targetPointcloud, pcFrame );

    mergeOp->addOutput( targetPointcloud );
    mergeOp->updateAll();

    
    // generate add-hoc environment that only contains
    // target pointcloud
    envire::Environment envOut;
    envire::FrameNode *pcFrameOut = new envire::FrameNode( body2odometry );
    envOut.attachItem( pcFrameOut );
    envOut.getRootNode()->addChild( pcFrameOut );
    EnvironmentItem::Ptr targetPointcloudOut  = env->detachItem(targetPointcloud);
    envOut.attachItem( targetPointcloudOut.get() );
    targetPointcloud->setFrameNode( pcFrameOut );

    // write environment, if path is given
    if( !_environment_debug_path.value().empty() )
    {
	std::string debug_path = _environment_debug_path.value() + "-" + getName();
	LOG_DEBUG_S << "write to " << debug_path << std::endl;
	envOut.serialize(debug_path);
    }

    if( _envire_events.connected() )
    {
	envire::OrocosEmitter emitter(_envire_events);
	emitter.setTime( lastScanTime );
	emitter.attach( &envOut );
	emitter.flush();
    }

    if( _pointcloud.connected() )
    {
	base::samples::Pointcloud pc;
	pc.time = lastScanTime;
	pc.points.resize( targetPointcloud->vertices.size() );
	std::copy( targetPointcloud->vertices.begin(), targetPointcloud->vertices.end(), pc.points.begin() );
	_pointcloud.write( pc );
    }
}

void Task::scan_samplesTransformerCallback(const base::Time &ts, const ::base::samples::LaserScan &scan_samples_sample)
{
    if(!generatePointCloud)
	return;
    // start building up scans until the odometry changes more than a delta
    // with respect to the first recorded pose of the scan, then start a new.
    // only consider scans of a specific number of lines valid

    Eigen::Affine3d body2odometry, laser2body;
    if( !_body2odometry.get( ts, body2odometry, true ) || !_laser2body.get( ts, laser2body, true ) )
	return;

    if( scan_running )
    {
	// setup an environment if there is none
	if( !env )
	    resetEnv();

        Eigen::Affine3d laser2Odometry(body2odometry * laser2body);
        
	// add the current scan-line
	addScanLine( scan_samples_sample, laser2Odometry );
        
        body2odometryAtLastScan = body2odometry;
    }

    if( !scan_running && env )
    {
	writePointcloud(body2odometryAtLastScan);
	env = boost::shared_ptr<envire::Environment>();
    }

    // test for conditions to stop the current scan
    /*
    int numScans = env->getInputs( mergeOp.get() ).size();
    if( numScans > config.max_lines )
    {
	writePointcloud();
	env = boost::shared_ptr<envire::Environment>();
    }
    */

    // TODO test that the robot has not moved depending on configuration
    //if( scanFrame && !_config.value().max_pose_change.test( scan_body2odometry, body2odometry ) )
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    // copy configuration 
    config = _config.value();
    scan_running = false;
    generatePointCloud = _generate_point_cloud.get();    

    status.sourceName = config.sweep_servo_name;
    
    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;

    //drive initially to min angle
    servoCmd.names.push_back( config.sweep_servo_name );
    base::JointState state;
    state.position = config.sweep_angle_max;
    state.speed = config.sweep_velocity_up;
    servoCmd.elements.push_back( state );
    _tilt_cmd.write( servoCmd );
    lastCmdTime = base::Time::now();
    
    trigger = false;
    status.counter = 0;
    status.curState = SweepStatus::SweepStatus::SWEEPING_UP;
    _sweep_status.write(status);

    return true;
}
void Task::updateHook()
{
    if( handleSweep() )
    {
	scan_running = status.curState == SweepStatus::SWEEPING_DOWN;
    }
    TaskBase::updateHook();
}
// void Task::errorHook()
// {
//     TaskBase::errorHook();
// }
void Task::stopHook()
{
    servoCmd.clear();
    TaskBase::stopHook();
}

void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}

