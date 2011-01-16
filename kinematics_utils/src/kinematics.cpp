/*
 * kinematics.cpp
 *
 *  Created on: 16-01-2011
 *      Author: konrad
 */

#include "../include/kinematics_utils/kinematics.h"

namespace kinematics_node
{

static const std::string IK_SERVICE = "get_ik";
static const std::string FK_SERVICE = "get_fk";
static const std::string IK_INFO_SERVICE = "get_ik_solver_info";
static const std::string FK_INFO_SERVICE = "get_fk_solver_info";

Kinematics::Kinematics() :
  nh_("~"), kinematics_loader_("kinematics_base", "kinematics::KinematicsBase")
{

}

Kinematics::~Kinematics()
{

}

bool Kinematics::initialize()
{
  std::string kinematics_name;

  nh_.param(std::string("kinematics_name"), kinematics_name, std::string());

  if (kinematics_name.empty())
  {
    ROS_ERROR("kinematics_name parameter not specyfied, unable to load kinematics plugin");
    return false;
  }

  try
  {
    kin_ = kinematics_loader_.createClassInstance(kinematics_name);
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The kinematics plugin failed to load for some reason. Error: %s", ex.what());
    return false;
  }

  if (!kin_->initialize(""))
    return false;

  fk_solver_info_.joint_names = ik_solver_info_.joint_names = kin_->getJointNames();
  fk_solver_info_.link_names = kin_->getLinkNames();
  ik_solver_info_.link_names.push_back(kin_->getToolFrame());

  fk_service_ = nh_.advertiseService(FK_SERVICE, &Kinematics::getPositionFK, this);
  ik_service_ = nh_.advertiseService(IK_SERVICE, &Kinematics::getPositionIK, this);

  ik_solver_info_service_ = nh_.advertiseService(IK_INFO_SERVICE, &Kinematics::getIKSolverInfo, this);
  fk_solver_info_service_ = nh_.advertiseService(FK_INFO_SERVICE, &Kinematics::getFKSolverInfo, this);

  return true;
}

bool Kinematics::getIKSolverInfo(kinematics_msgs::GetKinematicSolverInfo::Request &request,
                     kinematics_msgs::GetKinematicSolverInfo::Response &response)
{
  response.kinematic_solver_info = ik_solver_info_;
  return true;
}

bool Kinematics::getFKSolverInfo(kinematics_msgs::GetKinematicSolverInfo::Request &request,
                     kinematics_msgs::GetKinematicSolverInfo::Response &response)
{
  response.kinematic_solver_info = fk_solver_info_;
  return true;
}

bool Kinematics::getPositionIK(kinematics_msgs::GetPositionIK::Request &request,
               kinematics_msgs::GetPositionIK::Response &response)
{

}

bool Kinematics::getPositionFK(kinematics_msgs::GetPositionFK::Request &request,
                   kinematics_msgs::GetPositionFK::Response &response)
{

}

}
