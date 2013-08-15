#! /usr/bin/env ruby

require 'orocos'
require 'orocos/log'
require 'asguard'
require 'vizkit'
include Orocos

if ARGV.size < 1 then 
    puts "usage: process_logs.rb log_dir"
    exit
end

BASE_DIR = File.expand_path('..', File.dirname(__FILE__))
ENV['PKG_CONFIG_PATH'] = "#{BASE_DIR}/build:#{ENV['PKG_CONFIG_PATH']}"
Orocos.initialize

# This will kill processes when we quit the block
Orocos.run 'tilt_scan_test' do |p|
    tilt_scan = p.task('tilt_scan')

    replay = Orocos::Log::Bundle.open( ARGV[0] )

    replay.log.hokuyo.scans.connect_to( tilt_scan.scan_samples, :type => :buffer, :size => 1000 ) 
    replay.log.odometry.odometry_samples.connect_to( tilt_scan.dynamic_transformations, :type => :buffer, :size => 1000 )
    replay.log.dynamixel.lowerDynamixel2UpperDynamixel.connect_to( tilt_scan.dynamic_transformations, :type => :buffer, :size => 1000 )

    tf = Asguard::Transform.new [:dynamixel]
    tf.setup_filters replay

    tilt_scan.configure
    tilt_scan.start
    tf.configure_task tilt_scan

    replay.log.align( :use_sample_time )

    tilt_scan.environment_debug_path = "/tmp/env"

    if true then
	#Orocos.log_all_ports( {:tasks => 'tilt_scan', :log_dir => ARGV[0]} )
	
	replay.sync_task tilt_scan, 1.0, "transformer"
	replay.run
    else
	Vizkit.control replay.log
	Vizkit.exec
    end

end

