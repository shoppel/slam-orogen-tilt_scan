/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <envire/maps/LaserScan.hpp>
#include <envire/maps/TriMesh.hpp>
#include <envire/operators/ScanMeshing.hpp>

#include <stereo/densestereo.h>
#include "opencv2/highgui/highgui.hpp"

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

void Task::left_frame_inTransformerCallback(const base::Time &ts, const ::base::samples::frame::Frame &left_frame_in_sample)
{
    Eigen::Affine3d lcamera2body;
    if( !_lcamera2body.get( ts, lcamera2body ) )
	return;

    double angle = acos( -lcamera2body.linear()(1,1) );
    if( abs((rightFrame.time - leftFrame.time).toMilliseconds()) > 15 || angle < best_angle )
    {
	leftFrame = left_frame_in_sample;
	best_angle = angle;
	best_lcamera2body = lcamera2body;
    }
}

void Task::right_frame_inTransformerCallback(const base::Time &ts, const ::base::samples::frame::Frame &right_frame_in_sample)
{
    Eigen::Affine3d lcamera2body;
    if( !_lcamera2body.get( ts, lcamera2body ) )
	return;

    double angle = acos( -lcamera2body.linear()(1,1) );
    if( abs((rightFrame.time - leftFrame.time).toMilliseconds()) > 15 || angle < best_angle )
    {
	rightFrame = right_frame_in_sample;
	best_angle = angle;
	best_lcamera2body = lcamera2body;
    }
}

void Task::scan_samplesTransformerCallback(const base::Time &ts, const ::base::samples::LaserScan &scan_samples_sample)
{
    // start building up scans until the odometry changes more than a delta
    // with respect to the first recorded pose of the scan, then start a new.
    // only consider scans of a specific number of lines valid

    Eigen::Affine3d body2odometry, laser2body;
    if( !_body2odometry.get( ts, body2odometry ) || !_laser2body.get( ts, laser2body ) )
	return;

    // for now set to 1cm and 1degree 
    // TODO parametrize
    base::PoseUpdateThreshold pose_change_threshold( 0.01, 1.0 / 180.0 * M_PI ); 

    if( scanFrame && !pose_change_threshold.test( scan_body2odometry, body2odometry ) )
    {
	// store laserline for pose 
	
	// get the laserscan and set up the operator chain
	FrameNode* laserFrame = new FrameNode( laser2body );
	env->addChild( scanFrame.get(), laserFrame );

	LaserScan *scanNode = new LaserScan();
	scanNode->addScanLine( 0, scan_samples_sample );
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
    else
    {
	if( scanFrame )
	{
	    // if the number of scans in the frame is enough, keep and store it
	    size_t numScans = env->getInputs( mergeOp.get() ).size();

	    size_t numScansThreshold = 100;
	    if( numScans > numScansThreshold )
	    {
		std::cout << "lines in scan: " << numScans << std::endl;
		mergeOp->updateAll();
		// TODO reproject pointcloud into distance frame and write to
		// output port if connected

		int time_diff = abs((leftFrame.time - rightFrame.time).toMilliseconds());
		if( time_diff < 15 )
		{
		    std::cout << "has left/right camera image " << leftFrame.time.toString() << std::endl;
		    _left_frame.write( leftFrame );
		    _right_frame.write( rightFrame );

		    stereo::DenseStereo dense; 
		    frame_helper::StereoCalibration calib = 
			frame_helper::StereoCalibration::fromMatlabFile( _stereo_calibration.get() );

		    dense.setStereoCalibration( calib, 
			    leftFrame.getWidth(), leftFrame.getHeight() );

		    base::samples::DistanceImage ldistImage, rdistImage;
		    cv::Mat 
			ldist = dense.createLeftDistanceImage( ldistImage ),
			rdist = dense.createRightDistanceImage( rdistImage );
		    ldistImage.clear();
		    ldistImage.time = leftFrame.time;

		    rdistImage.clear();
		    rdistImage.time = leftFrame.time;

		    envire::Pointcloud *points = dynamic_cast<Pointcloud*>( *env->getOutputs( mergeOp.get() ).begin() );
		    if( points )
		    {
			Eigen::Affine3d lcamera2rcamera = calib.extrinsic.getTransform();
			Eigen::Affine3d body2lcamera = best_lcamera2body.inverse();
			for( size_t i=0; i<points->vertices.size(); i++ )
			{
			    Eigen::Vector3d p = body2lcamera * points->vertices[i];
			    size_t x, y;
			    if( ldistImage.getImagePoint( p, x, y ) )
				ldistImage.data[y*ldistImage.width+x] = p.z();

			    p = lcamera2rcamera * p;

			    if( rdistImage.getImagePoint( p, x, y ) )
				rdistImage.data[y*rdistImage.width+x] = p.z();
			}

			_left_distance_frame.write( ldistImage );
			_right_distance_frame.write( rdistImage );
		    }
		}
	    }
	    else
	    {
		env->detachItem( scanFrame.get(), true );
		// TODO remove individual scanlines
	    }

	    // detach all the other connections
	    std::list<Layer*> lines = env->getInputs( mergeOp.get() );
	    for( std::list<Layer*>::iterator it = lines.begin(); it != lines.end(); it++ )
	    {
		CartesianMap* map = dynamic_cast<CartesianMap*>( *it );
		if( map )
		{
		    map->getFrameNode()->detach();
		    map->detach();
		}
	    }
	    mergeOp->detach();
	}

	// set-up new structure
	// Framenode for the body to odometry
	scanFrame = new envire::FrameNode( body2odometry );
	env->getRootNode()->addChild( scanFrame.get() );

	// and a pointlcoud with the final scan
	envire::Pointcloud *pc = new envire::Pointcloud();
	env->setFrameNode( pc, scanFrame.get() );

	// as well as a merge operator has the pointcloud as output
	mergeOp = new MergePointcloud();
	env->attachItem( mergeOp.get() );
	mergeOp->addOutput( pc );

	// set the current scan_frame for the threshold
	scan_body2odometry = body2odometry;
	best_angle = 2.0 * M_PI;
    }
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    env = new envire::Environment();
    return true;
}
// bool Task::startHook()
// {
//     if (! TaskBase::startHook())
//         return false;
//     return true;
// 
//     env = new envire::Environment();
// }
// void Task::updateHook()
// {
//     TaskBase::updateHook();
// }
// void Task::errorHook()
// {
//     TaskBase::errorHook();
// }
// void Task::stopHook()
// {
//     TaskBase::stopHook();
// }

void Task::cleanupHook()
{
    TaskBase::cleanupHook();

    // write environment, if path is given
    if( !_environment_debug_path.value().empty() )
    {
	env->serialize(_environment_debug_path.value() );
    }

    delete env;
}

