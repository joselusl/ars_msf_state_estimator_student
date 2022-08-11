#!/usr/bin/env python

import numpy as np
from numpy import *

import os




# ROS

import rospy

import rospkg

import std_msgs.msg
from std_msgs.msg import Header

import geometry_msgs.msg
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import QuaternionStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import TwistWithCovarianceStamped



import tf2_ros




#
from ars_msf_state_estimator import *


#
import ars_lib_helpers





class ArsMsfStateEstimatorRos:

  #######

  # Robot frame
  robot_frame = None

  # World frame
  world_frame = None


  # State Estim loop freq 
  # time step
  state_estim_loop_freq = None
  # Timer
  state_estim_loop_timer = None


  # Meas Robot posi subscriber
  meas_robot_posi_sub = None
  # Meas Robot atti subscriber
  meas_robot_atti_sub = None
  # Meas Robot velocity subscriber
  meas_robot_vel_robot_sub = None


  # Estim Robot pose pub
  estim_robot_pose_pub = None
  estim_robot_pose_cov_pub = None
  # Estim Robot velocity pub
  estim_robot_vel_robot_pub = None
  estim_robot_vel_robot_cov_pub = None
  #
  estim_robot_vel_world_pub = None
  estim_robot_vel_world_cov_pub = None


  # tf2 broadcaster
  tf2_broadcaster = None


  # MSF state estimator
  msf_state_estimator = None
  


  #########

  def __init__(self):

    # Robot frame
    self.robot_frame = 'robot_estim_base_link'

    # World frame
    self.world_frame = 'world'

    # State Estim loop freq 
    # time step
    self.state_estim_loop_freq = 50.0

    # Motion controller
    self.msf_state_estimator = ArsMsfStateEstimator()


    # end
    return


  def init(self, node_name='ars_msf_state_estimator_node'):
    #

    # Init ROS
    rospy.init_node(node_name, anonymous=True)

    
    # Package path
    pkg_path = rospkg.RosPack().get_path('ars_msf_state_estimator')
    

    #### READING PARAMETERS ###
    
    # TODO

    ###

    
    # End
    return


  def open(self):

    # Subscribers

    # 
    self.meas_robot_posi_sub = rospy.Subscriber('meas_robot_position', PointStamped, self.measRobotPositionCallback)
    # 
    self.meas_robot_atti_sub = rospy.Subscriber('meas_robot_attitude', QuaternionStamped, self.measRobotAttitudeCallback)
    #
    self.meas_robot_vel_robot_sub = rospy.Subscriber('meas_robot_velocity_robot', TwistStamped, self.measRobotVelRobotCallback)
    


    # Publishers

    # 
    self.estim_robot_pose_pub = rospy.Publisher('estim_robot_pose', PoseStamped, queue_size=1)
    # 
    self.estim_robot_pose_cov_pub = rospy.Publisher('estim_robot_pose_cov', PoseWithCovarianceStamped, queue_size=1)
    #
    self.estim_robot_vel_robot_pub = rospy.Publisher('estim_robot_velocity_robot', TwistStamped, queue_size=1)
    #
    self.estim_robot_vel_robot_cov_pub = rospy.Publisher('estim_robot_velocity_robot_cov', TwistWithCovarianceStamped, queue_size=1)
    #
    self.estim_robot_vel_world_pub = rospy.Publisher('estim_robot_velocity_world', TwistStamped, queue_size=1)
    #
    self.estim_robot_vel_world_cov_pub = rospy.Publisher('estim_robot_velocity_world_cov', TwistWithCovarianceStamped, queue_size=1)


    # Tf2 broadcasters
    self.tf2_broadcaster = tf2_ros.TransformBroadcaster()


    # Timers
    #
    self.state_estim_loop_timer = rospy.Timer(rospy.Duration(1.0/self.state_estim_loop_freq), self.stateEstimLoopTimerCallback)


    # End
    return


  def run(self):

    rospy.spin()

    return


  def measRobotPositionCallback(self, robot_position_msg):

    # Timestamp
    timestamp = robot_position_msg.header.stamp

    # Position
    robot_posi = np.zeros((3,), dtype=float)
    robot_posi[0] = robot_position_msg.point.x
    robot_posi[1] = robot_position_msg.point.y
    robot_posi[2] = robot_position_msg.point.z

    #
    self.msf_state_estimator.setMeasRobotPosition(timestamp, robot_posi)

    # Predict
    #self.msf_state_estimator.predict(timestamp)

    # Update
    #self.msf_state_estimator.update()

    #
    return


  def measRobotAttitudeCallback(self, robot_attitude_msg):

    # Timestamp
    timestamp = robot_attitude_msg.header.stamp

    # Attitude quat simp
    robot_atti_quat = ars_lib_helpers.Quaternion.zerosQuat()
    robot_atti_quat[0] = robot_attitude_msg.quaternion.w
    robot_atti_quat[1] = robot_attitude_msg.quaternion.x
    robot_atti_quat[2] = robot_attitude_msg.quaternion.y
    robot_atti_quat[3] = robot_attitude_msg.quaternion.z

    robot_atti_quat_simp = ars_lib_helpers.Quaternion.getSimplifiedQuatRobotAtti(robot_atti_quat)

    #
    self.msf_state_estimator.setMeasRobotAttitude(timestamp, robot_atti_quat_simp)

    # Predict
    #self.msf_state_estimator.predict(timestamp)

    # Update
    #self.msf_state_estimator.update()

    #
    return


  def measRobotVelRobotCallback(self, robot_vel_msg):

    # Timestamp
    timestamp = robot_vel_msg.header.stamp

    # Linear
    lin_vel_robot = np.zeros((3,), dtype=float)
    lin_vel_robot[0] = robot_vel_msg.twist.linear.x
    lin_vel_robot[1] = robot_vel_msg.twist.linear.y
    lin_vel_robot[2] = robot_vel_msg.twist.linear.z

    # Angular
    ang_vel_robot = np.zeros((1,), dtype=float)
    ang_vel_robot[0] = robot_vel_msg.twist.angular.z

    #
    self.msf_state_estimator.setMeasRobotVelRobot(timestamp, lin_vel_robot, ang_vel_robot)

    # Predict
    #self.msf_state_estimator.predict(timestamp)

    # Update
    #self.msf_state_estimator.update()

    #
    return


  def estimRobotPosePublish(self):

    #
    header_msg = Header()
    header_msg.stamp = self.msf_state_estimator.estim_state_timestamp
    header_msg.frame_id = self.world_frame

    #
    robot_pose_msg = Pose()
    #
    robot_pose_msg.position.x = self.msf_state_estimator.estim_robot_posi[0]
    robot_pose_msg.position.y = self.msf_state_estimator.estim_robot_posi[1]
    robot_pose_msg.position.z = self.msf_state_estimator.estim_robot_posi[2]
    #
    robot_pose_msg.orientation.w = self.msf_state_estimator.estim_robot_atti_quat_simp[0]
    robot_pose_msg.orientation.x = 0.0
    robot_pose_msg.orientation.y = 0.0
    robot_pose_msg.orientation.z = self.msf_state_estimator.estim_robot_atti_quat_simp[1]

    #
    # Covariance
    covariance_pose = np.zeros((6,6), dtype=float)
    # Position - Position
    covariance_pose[0:3, 0:3] = self.msf_state_estimator.estim_state_cov[0:3, 0:3]
    # Position - Attitude
    covariance_pose[0:3, 5] = self.msf_state_estimator.estim_state_cov[0:3, 3]
    # Attitude - Attitude
    covariance_pose[5, 5] = self.msf_state_estimator.estim_state_cov[3, 3]
    # Attitude - Position
    covariance_pose[5, 0:3] = self.msf_state_estimator.estim_state_cov[3, 0:3]

    #
    robot_pose_stamped_msg = PoseStamped()
    #
    robot_pose_stamped_msg.header = header_msg
    robot_pose_stamped_msg.pose = robot_pose_msg

    #
    robot_pose_cov_stamped_msg = PoseWithCovarianceStamped()
    #
    robot_pose_cov_stamped_msg.header = header_msg
    robot_pose_cov_stamped_msg.pose.pose = robot_pose_msg
    robot_pose_cov_stamped_msg.pose.covariance = covariance_pose.reshape((36,))
  
    #
    self.estim_robot_pose_pub.publish(robot_pose_stamped_msg)
    # 
    self.estim_robot_pose_cov_pub.publish(robot_pose_cov_stamped_msg)


    # Tf2
    tf2__msg = geometry_msgs.msg.TransformStamped()

    tf2__msg.header.stamp = self.msf_state_estimator.estim_state_timestamp
    tf2__msg.header.frame_id = self.world_frame
    tf2__msg.child_frame_id = self.robot_frame

    tf2__msg.transform.translation.x = self.msf_state_estimator.estim_robot_posi[0]
    tf2__msg.transform.translation.y = self.msf_state_estimator.estim_robot_posi[1]
    tf2__msg.transform.translation.z = self.msf_state_estimator.estim_robot_posi[2]

    tf2__msg.transform.rotation.w = self.msf_state_estimator.estim_robot_atti_quat_simp[0]
    tf2__msg.transform.rotation.x = 0.0
    tf2__msg.transform.rotation.y = 0.0
    tf2__msg.transform.rotation.z = self.msf_state_estimator.estim_robot_atti_quat_simp[1]

    # Broadcast
    self.tf2_broadcaster.sendTransform(tf2__msg)


    # End
    return


  def estimRobotVelocityPublish(self):

    # TODO Finish!

    #
    header_wrt_world_msg = Header()
    header_wrt_world_msg.stamp = self.msf_state_estimator.estim_state_timestamp
    header_wrt_world_msg.frame_id = self.world_frame

    #
    header_wrt_robot_msg = Header()
    header_wrt_robot_msg.stamp = self.msf_state_estimator.estim_state_timestamp
    header_wrt_robot_msg.frame_id = self.robot_frame

    #
    robot_velocity_world_msg = Twist()
    #
    robot_velocity_world_msg.linear.x = self.msf_state_estimator.estim_robot_velo_lin_world[0]
    robot_velocity_world_msg.linear.y = self.msf_state_estimator.estim_robot_velo_lin_world[1]
    robot_velocity_world_msg.linear.z = self.msf_state_estimator.estim_robot_velo_lin_world[2]
    #
    robot_velocity_world_msg.angular.x = 0.0
    robot_velocity_world_msg.angular.y = 0.0
    robot_velocity_world_msg.angular.z = self.msf_state_estimator.estim_robot_velo_ang_world[0]

    #
    # TODO Cov

    #
    # TODO wrt robot

    #
    robot_velocity_world_stamp_msg = TwistStamped()
    robot_velocity_world_stamp_msg.header = header_wrt_world_msg
    robot_velocity_world_stamp_msg.twist = robot_velocity_world_msg

    #
    robot_velocity_world_cov_stamp_msg = TwistWithCovarianceStamped()
    robot_velocity_world_cov_stamp_msg.header = header_wrt_world_msg
    robot_velocity_world_cov_stamp_msg.twist.twist = robot_velocity_world_msg
    # robot_velocity_world_cov_stamp_msg.twist.covariance

    #
    # TODO
    robot_velocity_robot_stamp_msg = TwistStamped()
    robot_velocity_robot_stamp_msg.header = header_wrt_robot_msg
    #robot_velocity_robot_stamp_msg.twist = robot_velocity_robot_msg

    #
    # TODO
    robot_velocity_robot_cov_stamp_msg = TwistWithCovarianceStamped()
    robot_velocity_robot_cov_stamp_msg.header = header_wrt_robot_msg
    #robot_velocity_robot_cov_stamp_msg.twist.twist = robot_velocity_robot_msg
    # robot_velocity_robot_cov_stamp_msg.twist.covariance



    #
    self.estim_robot_vel_world_pub.publish(robot_velocity_world_stamp_msg)
    # 
    self.estim_robot_vel_world_cov_pub.publish(robot_velocity_world_cov_stamp_msg)

    #
    self.estim_robot_vel_robot_pub.publish(robot_velocity_robot_stamp_msg)
    # 
    self.estim_robot_vel_robot_cov_pub.publish(robot_velocity_robot_cov_stamp_msg)

    #
    return
  

  def stateEstimLoopTimerCallback(self, timer_msg):

    # Get time
    time_stamp_current = rospy.Time.now()

    # Predict
    self.msf_state_estimator.predict(time_stamp_current)

    # Update
    self.msf_state_estimator.update()


    # Publish
    #
    self.estimRobotPosePublish()
    #
    self.estimRobotVelocityPublish()

     
    # End
    return

  