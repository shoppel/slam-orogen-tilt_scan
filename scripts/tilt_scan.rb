#! /usr/bin/env ruby
require 'rock/bundle'
include Orocos

Bundles.initialize

Orocos.run "servo_dynamixel::Task" => "dynamixel_base", 
	    "tilt_scan::Task" => "tilt_scan",
	    "hokuyo::Task" => "hokuyo" do |p|

    # setup the dynamixel first
    dynamixel = Orocos::TaskContext.get('dynamixel_base')
    Orocos.conf.apply( dynamixel )
    dynamixel.configure
    dynamixel.start

    # the laser scanner
    hokuyo = Orocos::TaskContext.get('hokuyo')
    Orocos.conf.apply( hokuyo )
    hokuyo.configure
    hokuyo.start

    # the the tilt scan module
    tilt_scan = Orocos::TaskContext.get('tilt_scan')
    Orocos.conf.apply( tilt_scan )
    Bundles.transformer.load_conf(Bundles.find_file('config', 'script_transforms.rb'))
    Bundles.transformer.setup( tilt_scan )
    tilt_scan.environment_debug_path = "/tmp/env"
    tilt_scan.configure
    tilt_scan.start

    # do the data connections
    tilt_scan.tilt_cmd.connect_to dynamixel.command
    hokuyo.scans.connect_to tilt_scan.scan_samples

    while true
	tilt_scan.trigger
	sleep 0.1
    end
end

