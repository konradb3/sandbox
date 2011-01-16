/*
 * kinematics_node.cpp
 *
 *  Created on: 16-01-2011
 *      Author: konrad
 */

#include <ros/ros.h>

#include <kinematics_utils/kinematics.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinematics_node");

  kinematics_node::Kinematics kin;

  if(kin.initialize())
    ros::spin();

  return 0;
}
