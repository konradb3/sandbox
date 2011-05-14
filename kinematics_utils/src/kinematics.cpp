/*
 * kinematics.cpp
 *
 *  Created on: 16-01-2011
 *      Author: konrad
 */

#include <kinematics_utils/kinematics_utils.h>

#include <kinematics_utils/kinematics.h>

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

  dimension_ = ik_solver_info_.joint_names.size();

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
  std::vector<double> joint_seed, solution;

  tf::Stamped<tf::Pose> tf_pose;
  geometry_msgs::PoseStamped pose;

  if (!checkIKService(request, response, ik_solver_info_))
    return true;

  tf::poseStampedMsgToTF(request.ik_request.pose_stamped, tf_pose);
  try
  {
    tf_.transformPose(kin_->getBaseFrame(), tf_pose, tf_pose);
  }
  catch (...)
  {
    ROS_ERROR("Could not transform IK pose to frame: %s",kin_->getBaseFrame().c_str());
    response.error_code.val = response.error_code.FRAME_TRANSFORM_FAILURE;
    return true;
  }
  tf::poseStampedTFToMsg(tf_pose, pose);

  //Do the IK
  joint_seed.resize(dimension_);
  solution.resize(dimension_);
  for(int i=0; i < dimension_; i++)
  {
    int tmp_index = getJointIndex(request.ik_request.ik_seed_state.joint_state.name[i],ik_solver_info_);
    if(tmp_index >=0)
    {
      joint_seed[tmp_index] = request.ik_request.ik_seed_state.joint_state.position[i];
    }
    else
    {
      ROS_ERROR("i: %d, No joint index for %s",i,request.ik_request.ik_seed_state.joint_state.name[i].c_str());
    }
  }

  bool ik_valid = kin_->searchPositionIK(pose.pose, joint_seed, request.timeout.toSec(), solution);

  if(ik_valid)
  {
    response.solution.joint_state.name = ik_solver_info_.joint_names;
    response.solution.joint_state.position.resize(dimension_);
    for(int i=0; i < dimension_; i++)
    {
      response.solution.joint_state.position[i] = solution[i];
      ROS_DEBUG("IK Solution: %s %d: %f",response.solution.joint_state.name[i].c_str(),i,solution[i]);
    }
    response.error_code.val = response.error_code.SUCCESS;
    return true;
  }
  else
  {
    ROS_DEBUG("An IK solution could not be found");
    response.error_code.val = response.error_code.NO_IK_SOLUTION;
    return true;
  }
}

bool Kinematics::getPositionFK(kinematics_msgs::GetPositionFK::Request &request,
                               kinematics_msgs::GetPositionFK::Response &response)
{
  std::vector<double> joint_angles;
  std::vector<geometry_msgs::Pose> poses;
  geometry_msgs::PoseStamped pose;
  tf::Stamped<tf::Pose> tf_pose;

  if (!checkFKService(request, response, fk_solver_info_))
    return true;

  joint_angles.resize(dimension_);
  for (int i = 0; i < dimension_; i++)
  {
    int tmp_index = getJointIndex(request.robot_state.joint_state.name[i], fk_solver_info_);
    if (tmp_index >= 0)
      joint_angles[tmp_index] = request.robot_state.joint_state.position[i];
  }

  poses.resize(request.fk_link_names.size());
  if (!kin_->getPositionFK(request.fk_link_names, joint_angles, poses))
  {
    response.error_code.val = response.error_code.NO_FK_SOLUTION;
    return true;
  }

  response.pose_stamped.resize(request.fk_link_names.size());
  response.fk_link_names.resize(request.fk_link_names.size());

  for (unsigned int i = 0; i < request.fk_link_names.size(); i++)
  {
    tf_pose.frame_id_ = kin_->getBaseFrame();
    tf_pose.stamp_ = ros::Time();
    tf::poseMsgToTF(poses[i], tf_pose);
    try
    {
      tf_.transformPose(request.header.frame_id, tf_pose, tf_pose);
    }
    catch (...)
    {
      ROS_ERROR("Could not transform FK pose to frame: %s",request.header.frame_id.c_str());
      response.error_code.val = response.error_code.FRAME_TRANSFORM_FAILURE;
      return true;
    }
    tf::poseStampedTFToMsg(tf_pose, pose);
    response.pose_stamped[i] = pose;
    response.fk_link_names[i] = request.fk_link_names[i];
    response.error_code.val = response.error_code.SUCCESS;
  }
  return true;
}

}
