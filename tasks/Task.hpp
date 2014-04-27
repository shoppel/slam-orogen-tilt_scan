/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef TILT_SCAN_TASK_TASK_HPP
#define TILT_SCAN_TASK_TASK_HPP

#include "tilt_scan/TaskBase.hpp"
#include <tilt_scanTypes.hpp>
#include <envire/Core.hpp>
#include <envire/operators/MergePointcloud.hpp>
#include <boost/shared_ptr.hpp>

namespace tilt_scan {
    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:

        virtual void scan_samplesTransformerCallback(const base::Time &ts, const ::base::samples::LaserScan &scan_samples_sample);

        Eigen::Affine3d body2odometryAtLastScan;
	boost::shared_ptr<envire::Environment> env;
	envire::MergePointcloud *mergeOp;

	base::Time lastScanTime;

	void addScanLine( const ::base::samples::LaserScan &scan, const Eigen::Affine3d& laser2Odometry );
	void writePointcloud(const Eigen::Affine3d& body2odometry );
	void resetEnv();

	bool handleSweep();
	base::Time last_sweep_change;
	bool sweep_forward;
	bool scan_running;
	bool generatePointCloud;

	Configuration config;
        SweepStatus status;
        base::commands::Joints servoCmd;
        base::Time lastCmdTime;
    public:
        Task(std::string const& name = "tilt_scan::Task");
        Task(std::string const& name, RTT::ExecutionEngine* engine);

	~Task();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         *
         *   task_context "TaskName" do
         *     needs_configuration
         *     ...
         *   end
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        // void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();
    };
}

#endif

