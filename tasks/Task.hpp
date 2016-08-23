#ifndef TILT_SCAN_TASK_TASK_HPP
#define TILT_SCAN_TASK_TASK_HPP

#include <tilt_scan/TaskBase.hpp>
#include <tilt_scanTypes.hpp>

#include <base/commands/Joints.hpp>

namespace tilt_scan
{
	class Task : public TaskBase
	{
	friend class TaskBase;
	protected:

		// Callbacks
		virtual void scan_samplesTransformerCallback(const base::Time &ts, const ::base::samples::LaserScan &scan_samples_sample);
		virtual void trigger_sweep();

		// Internal methods

		// Members
		bool trigger;
		bool scan_running;

		base::Time last_sweep_change;
		base::Time lastScanTime;
		base::Time lastCmdTime;

		base::commands::Joints servoCmd;

		Configuration mConfiguration;
		SweepStatus mSweepStatus;

	public:
		Task(std::string const& name = "tilt_scan::Task");
		Task(std::string const& name, RTT::ExecutionEngine* engine);
		~Task();

		bool configureHook();
		bool startHook();
		void updateHook();
		void errorHook();
		void stopHook();
		void cleanupHook();
	};
}

#endif

