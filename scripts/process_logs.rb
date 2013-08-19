#! /usr/bin/env ruby

require 'rock/bundle'
require 'vizkit'
include Orocos

if ARGV.size < 1 then 
    puts "usage: process_logs.rb log_dir"
    exit
end

Bundles.initialize

Bundles.run 'tilt_scan::Task' => 'tilt_scan', 'valgrind' => false  do |p|
    tilt_scan = Orocos::TaskContext.get('tilt_scan')
    tilt_scan.laser_frame = "laser_front"

    log = Orocos::Log::Replay.open( ARGV[0] )

    log.hokuyo.scans.connect_to( tilt_scan.scan_samples, :type => :buffer, :size => 1000 ) 
    log.dynamixel.lowerDynamixel2UpperDynamixel.filter = lambda do |d|
	d.sourceFrame = "laser_tilt_base_front"
	d.targetFrame = "laser_tilt_front"
	d
    end

    log.dynamixel.name = "dynamixel_front"
    ns = Orocos::Local::NameService.new( )
    ns.register( log.odometry )
    ns.register( log.dynamixel, 'dynamixel_front' )
    Orocos.name_service << ns 

    Bundles.transformer.load_conf(Bundles.find_file('config', 'transforms.rb'))
    Bundles.transformer.setup( tilt_scan )

    log.align( :use_sample_time )

    tilt_scan.configure
    tilt_scan.start

    tilt_scan.environment_debug_path = "/tmp/env"

    if false then
	#Orocos.log_all_ports( {:tasks => 'tilt_scan', :log_dir => ARGV[0]} )
	Vizkit.run
    else
	Vizkit.control log
	Vizkit.exec
    end

end

