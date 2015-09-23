#! /usr/bin/env ruby

require 'orocos'

include Orocos
Orocos.initialize

sampler = TaskContext::get 'tilt_scan_front'

sampler.triggerSweep();
