#!/bin/bash

rosrun xacro xacro.py lwr.xml.xacro -o example.xml
RTT_COMPONENT_PATH=`rospack find rtt`/install/lib/orocos `rospack find ocl`/bin/deployer-gnulinux -s example.xml
