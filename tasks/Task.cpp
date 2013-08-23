/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <envire/maps/LaserScan.hpp>
#include <envire/maps/TriMesh.hpp>
#include <envire/operators/ScanMeshing.hpp>
#include <orocos/envire/Orocos.hpp>
#include <base/Logging.hpp>

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

bool Task::handleSweep()
{
    if( _tilt_cmd.connected() )
    {
	const base::Time sweep_time = 
	    base::Time::fromSeconds( 
		    fabs(config.sweep_angle_max - config.sweep_angle_min) 
		    / config.sweep_velocity );

	// change of direction is purely open loop
	if( (last_sweep_change + sweep_time) < base::Time::now() )
	{
	    // set position command
	    base::commands::Joints joints;
	    joints.names.push_back( config.sweep_servo_name );
	    base::JointState cmd;
	    cmd.position = sweep_forward ? 
		config.sweep_angle_min : config.sweep_angle_max;
	    cmd.speed = config.sweep_velocity;
	    joints.elements.push_back( cmd );
	    _tilt_cmd.write( joints );
	    
	    // store current state
	    sweep_forward = !sweep_forward;
	    last_sweep_change = base::Time::now();

	    return true;
	}
    }

    return false;
}

void Task::resetEnv( const Eigen::Affine3d& body2odometry )
{
    // generate new environment
    env = boost::shared_ptr<envire::Environment>( new envire::Environment() );

    // set-up new structure
    // Framenode for the body to odometry
    scanFrame = new envire::FrameNode( body2odometry );
    env->getRootNode()->addChild( scanFrame.get() );

    // and a pointlcoud with the final scan
    targetPointcloud = new envire::Pointcloud();
    env->setFrameNode( targetPointcloud.get(), scanFrame.get() );

    // as well as a merge operator has the pointcloud as output
    mergeOp = new MergePointcloud();
    env->attachItem( mergeOp.get() );
    mergeOp->addOutput( targetPointcloud.get() );

    // set the current scan_frame for the threshold
    scan_body2odometry = body2odometry;
}

void Task::addScanLine( const ::base::samples::LaserScan &scan, const Eigen::Affine3d& laser2body )
{
    lastScanTime = scan.time;

    // get the laserscan and set up the operator chain
    FrameNode* laserFrame = new FrameNode( laser2body );
    env->addChild( scanFrame.get(), laserFrame );

    LaserScan *scanNode = new LaserScan();
    scanNode->setXForward();
    scanNode->addScanLine( 0, scan );
    env->setFrameNode( scanNode, laserFrame );

    TriMesh *laserPc = new TriMesh();
    env->setFrameNode( laserPc, laserFrame );

    ScanMeshing *smOp = new ScanMeshing();
    env->attachItem( smOp );
    smOp->addInput( scanNode );
    smOp->addOutput( laserPc );
    smOp->updateAll();

    // detach input nodes
    smOp->detach();
    scanNode->detach();

    // add to merge operator
    mergeOp->addInput( laserPc );
}

void Task::writePointcloud()
{
    mergeOp->updateAll();

    LOG_DEBUG_S << "got a pointcloud!" << std::endl;

    // generate add-hoc environment that only contains
    // target pointcloud
    envire::Environment env;
    env.attachItem( scanFrame.get() );
    env.getRootNode()->addChild( scanFrame.get() );
    env.attachItem( targetPointcloud.get() );
    targetPointcloud->setFrameNode( scanFrame.get() );

    // write environment, if path is given
    if( !_environment_debug_path.value().empty() )
    {
	LOG_DEBUG_S << "write to " << _environment_debug_path.value() << std::endl;
	env.serialize(_environment_debug_path.value() );
    }

    if( _envire_events.connected() )
    {
	envire::OrocosEmitter emitter(_envire_events);
	emitter.setTime( lastScanTime );
	emitter.attach( &env );
	emitter.flush();
    }
}

void Task::scan_samplesTransformerCallback(const base::Time &ts, const ::base::samples::LaserScan &scan_samples_sample)
{
    // start building up scans until the odometry changes more than a delta
    // with respect to the first recorded pose of the scan, then start a new.
    // only consider scans of a specific number of lines valid

    Eigen::Affine3d body2odometry, laser2body;
    if( !_body2odometry.get( ts, body2odometry ) || !_laser2body.get( ts, laser2body, true ) )
	return;

    if( scan_running )
    {
	// setup an environment if there is none
	if( !env )
	    resetEnv( body2odometry );

	// add the current scan-line
	// TODO what if the body does move? add the body2odometry transform in a better way
	addScanLine( scan_samples_sample, laser2body );
    }

    if( !scan_running && env )
    {
	writePointcloud();
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
    sweep_forward = true;
    scan_running = false;

    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;

    return true;
}
void Task::updateHook()
{
    if( handleSweep() )
    {
	scan_running = sweep_forward;
    }
    TaskBase::updateHook();
}
// void Task::errorHook()
// {
//     TaskBase::errorHook();
// }
void Task::stopHook()
{
    TaskBase::stopHook();
}

void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}

