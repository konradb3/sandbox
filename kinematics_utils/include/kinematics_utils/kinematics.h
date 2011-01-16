/*
 * kinematics.h
 *
 *  Created on: 16-01-2011
 *      Author: konrad
 */

#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <pluginlib/class_loader.h>

#include <kinematics_base/kinematics_base.h>

#include <kinematics_msgs/GetPositionFK.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>

namespace kinematics_node
{

class Kinematics
{
public:
  Kinematics();
  virtual ~Kinematics();
  bool initialize();

  /**
   * @brief This is the basic IK service method that will compute and return an IK solution.
   * @param A request message. See service definition for GetPositionIK for more information on this message.
   * @param The response message. See service definition for GetPositionIK for more information on this message.
   */
  bool getPositionIK(kinematics_msgs::GetPositionIK::Request &request,
                     kinematics_msgs::GetPositionIK::Response &response);

  /**
   * @brief This is the basic kinematics info service that will return information about the kinematics node.
   * @param A request message. See service definition for GetKinematicSolverInfo for more information on this message.
   * @param The response message. See service definition for GetKinematicSolverInfo for more information on this message.
   */
  bool getIKSolverInfo(kinematics_msgs::GetKinematicSolverInfo::Request &request,
                       kinematics_msgs::GetKinematicSolverInfo::Response &response);

  /**
   * @brief This is the basic kinematics info service that will return information about the kinematics node.
   * @param A request message. See service definition for GetKinematicSolverInfo for more information on this message.
   * @param The response message. See service definition for GetKinematicSolverInfo for more information on this message.
   */
  bool getFKSolverInfo(kinematics_msgs::GetKinematicSolverInfo::Request &request,
                       kinematics_msgs::GetKinematicSolverInfo::Response &response);

  /**
   * @brief This is the basic forward kinematics service that will return information about the kinematics node.
   * @param A request message. See service definition for GetPositionFK for more information on this message.
   * @param The response message. See service definition for GetPositionFK for more information on this message.
   */
  bool getPositionFK(kinematics_msgs::GetPositionFK::Request &request,
                     kinematics_msgs::GetPositionFK::Response &response);

private:
  ros::NodeHandle nh_;

  pluginlib::ClassLoader<kinematics::KinematicsBase> kinematics_loader_;
  kinematics::KinematicsBase *kin_;

  ros::ServiceServer ik_service_, fk_service_, ik_solver_info_service_, fk_solver_info_service_;
  tf::TransformListener tf_;
  kinematics_msgs::KinematicSolverInfo ik_solver_info_, fk_solver_info_;
  int dimisions_;
};

}

#endif /* KINEMATICS_H_ */
