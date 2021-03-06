#! /usr/bin/env python

PKG = 'move_arm'

import roslib; roslib.load_manifest(PKG)
import rospy
import planning_environment_msgs.srv
import sys
import unittest
import actionlib
import actionlib_msgs
import math

import sensor_msgs.msg
import mapping_msgs.msg
from mapping_msgs.msg import CollisionObject
from motion_planning_msgs.msg import CollisionOperation, MotionPlanRequest, DisplayTrajectory
from motion_planning_msgs.srv import GetMotionPlanRequest, GetMotionPlan
from geometric_shapes_msgs.msg import Shape
from geometry_msgs.msg import Pose, PointStamped
from tf import TransformListener
from motion_planning_msgs.msg import JointConstraint

padd_name = "ompl_planning/robot_padd"
extra_buffer = .1

class TestMotionExecutionBuffer(unittest.TestCase):

    def setUp(self):

        rospy.init_node('test_motion_execution_buffer')
        
        self.tf = TransformListener()        

        self.obj_pub = rospy.Publisher('collision_object',CollisionObject,latch=True)
        self.traj_pub = rospy.Publisher('rosie_path',DisplayTrajectory,latch=True)
        
        rospy.wait_for_service("ompl_planning/plan_kinematic_path")
        self.motion_planning_service = rospy.ServiceProxy("ompl_planning/plan_kinematic_path", GetMotionPlan)

        obj1 = CollisionObject()
    
        obj1.header.stamp = rospy.Time.now()
        obj1.header.frame_id = "base_link"
        obj1.id = "obj1";
        obj1.operation.operation = mapping_msgs.msg.CollisionObjectOperation.ADD
        obj1.shapes = [Shape() for _ in range(1)]
        obj1.shapes[0].type = Shape.CYLINDER
        obj1.shapes[0].dimensions = [float() for _ in range(2)]
        obj1.shapes[0].dimensions[0] = .1
        obj1.shapes[0].dimensions[1] = 1.5
        obj1.poses = [Pose() for _ in range(1)]
        obj1.poses[0].position.x = .9
        obj1.poses[0].position.y = -.6
        obj1.poses[0].position.z = .75
        obj1.poses[0].orientation.x = 0
        obj1.poses[0].orientation.y = 0
        obj1.poses[0].orientation.z = 0
        obj1.poses[0].orientation.w = 1
        
        self.obj_pub.publish(obj1)

        rospy.sleep(5.0)
        
    def tearDown(self):
        obj1 = CollisionObject()
        obj1.header.stamp = rospy.Time.now()
        obj1.header.frame_id = "base_link"
        obj1.id = "all";
        obj1.operation.operation = mapping_msgs.msg.CollisionObjectOperation.REMOVE

        #self.obj_pub.publish(obj1)
        
        rospy.sleep(2.0)

    def testMotionExecutionBuffer(self):
        
        global padd_name
        global extra_buffer
        
        #too much trouble to read for now
        allow_padd = .05#rospy.get_param(padd_name)
        
        joint_names = ['right_arm_0_joint',
                       'right_arm_1_joint',
                       'right_arm_2_joint',
                       'right_arm_3_joint',
                       'right_arm_4_joint',
                       'right_arm_5_joint',
                       'right_arm_6_joint']

        motion_plan_request = MotionPlanRequest()

        motion_plan_request.goal_constraints.joint_constraints = [JointConstraint() for i in range(len(joint_names))]

        motion_plan_request.group_name = "right_arm"
        motion_plan_request.num_planning_attempts = 1
        motion_plan_request.allowed_planning_time = rospy.Duration(5.)
        motion_plan_request.planner_id = ""
        planner_service_name = "ompl_planning/plan_kinematic_path"

        motion_plan_request.goal_constraints.joint_constraints = [JointConstraint() for i in range(len(joint_names))]
        for i in range(len(joint_names)):
            motion_plan_request.goal_constraints.joint_constraints[i].joint_name = joint_names[i]
            motion_plan_request.goal_constraints.joint_constraints[i].position = 0.0
            motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.08
            motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.08

        motion_plan_request.goal_constraints.joint_constraints[0].position = 1.5
        motion_plan_request.goal_constraints.joint_constraints[1].position = -1.5
        motion_plan_request.goal_constraints.joint_constraints[5].position = 0.0

        req = GetMotionPlanRequest()
        req.motion_plan_request = motion_plan_request

        res = self.motion_planning_service.call(req)

        path = DisplayTrajectory()
        path.model_id = "right_arm"
        path.trajectory.joint_trajectory = res.trajectory.joint_trajectory
        self.traj_pub.publish(path)

if __name__ == '__main__':

    import rostest
    rostest.unitrun('test_motion_execution_buffer', 'test_motion_execution_buffer', TestMotionExecutionBuffer)


    
