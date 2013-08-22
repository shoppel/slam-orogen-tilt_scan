#! /usr/bin/env ruby
require 'rock/bundle'
include Orocos

Bundles.initialize

Orocos.run "servo_dynamixel::Task" => "dynamixel_base", "tilt_scan::Task" => "tilt_scan" do |p|
    # setup the dynamixel first
    dynamixel = Orocos::TaskContext.get('dynamixel_base')
    Orocos.conf.apply( dynamixel )
    dynamixel.configure
    dynamixel.start

    tilt_scan = Orocos::TaskContext.get('tilt_scan')
    Orocos.conf.apply( tilt_scan )
    tilt_scan.tilt_cmd.connect_to dynamixel.command
    tilt_scan.configure
    tilt_scan.start

    while true
	tilt_scan.trigger
	sleep 0.1
    end
end

