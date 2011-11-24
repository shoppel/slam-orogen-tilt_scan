#! /usr/bin/env ruby

require 'orocos'
require 'orocos/log'
require 'asguard'
require 'vizkit'
include Orocos

if ARGV.size < 2 then 
    puts "usage: process_logs.rb log_dir calibration_file"
    exit
end

BASE_DIR = File.expand_path('..', File.dirname(__FILE__))
ENV['PKG_CONFIG_PATH'] = "#{BASE_DIR}/build:#{ENV['PKG_CONFIG_PATH']}"
Orocos.initialize

# This will kill processes when we quit the block
Orocos.run 'tilt_scan_test' do |p|
    tilt_scan = p.task('tilt_scan')
    tilt_scan.transformer_max_latency = 5.0

    replay = Orocos::Log::Bundle.open( ARGV[0] )

    replay.log.hokuyo.scans.connect_to( tilt_scan.scan_samples, :type => :buffer, :size => 1000 ) do |data|
	data.time = data.time + 2.95 
	data
    end
    replay.log.odometry.odometry_samples.connect_to( tilt_scan.dynamic_transformations, :type => :buffer, :size => 1000 )
    replay.log.dynamixel.lowerDynamixel2UpperDynamixel.connect_to( tilt_scan.dynamic_transformations, :type => :buffer, :size => 1000 )
    replay.log.camera_left.frame.connect_to( tilt_scan.right_frame_in, :type => :buffer, :size => 1000 )
    replay.log.camera_right.frame.connect_to( tilt_scan.left_frame_in, :type => :buffer, :size => 1000 )

    tf = Asguard::Transform.new [:dynamixel]
    tf.setup_filters replay

    tilt_scan.stereo_calibration = ARGV[1]
    tilt_scan.configure
    tilt_scan.start
    tf.configure_task tilt_scan

    replay.log.align( :use_sample_time )

    tilt_scan.environment_debug_path = "/tmp/env"

    if true then
	Orocos.log_all_ports( {:tasks => 'tilt_scan', :log_dir => ARGV[0]} )
	
	#Vizkit.display tilt_scan.left_frame
	#replay.sync_task tilt_scan, 5.0, "transformer"
	replay.run
    else
	Vizkit.control replay.log
	Vizkit.exec
    end

end

