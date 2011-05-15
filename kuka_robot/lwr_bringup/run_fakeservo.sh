#!/bin/bash

rosrun xacro xacro.py lwr_fakeservo.xml.xacro -o example.xml
RTT_COMPONENT_PATH=`rospack find rtt`/install/lib/orocos `rospack find ocl`/bin/deployer-gnulinux -s example.xml
