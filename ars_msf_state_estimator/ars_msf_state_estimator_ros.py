#!/usr/bin/env python3

import numpy as np
from numpy import *

import os

# pyyaml - https://pyyaml.org/wiki/PyYAMLDocumentation
import yaml
from yaml.loader import SafeLoader


# ROS
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from ament_index_python.packages import get_package_share_directory

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
from ars_msf_state_estimator.ars_msf_state_estimator import *

#
import ars_lib_helpers.ars_lib_helpers as ars_lib_helpers




class ArsMsfStateEstimatorRos(Node):

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


  #
  config_param = None


  # MSF state estimator
  msf_state_estimator = None
  


  #########

  def __init__(self, node_name='ars_msf_state_estimator_node'):

    # Init ROS
    super().__init__(node_name)

    # Robot frame
    self.robot_frame = 'robot_estim_base_link'
    # World frame
    self.world_frame = 'world'

    # State Estim loop freq 
    self.state_estim_loop_freq = 50.0

    # Motion controller
    self.msf_state_estimator = ArsMsfStateEstimator()

    #
    self.__init(node_name)

    # end
    return


  def __init(self, node_name='ars_msf_state_estimator_node'):

    # Package path
    try:
      pkg_path = get_package_share_directory('ars_msf_state_estimator')
      self.get_logger().info(f"The path to the package is: {pkg_path}")
    except ModuleNotFoundError:
      self.get_logger().info("Package not found")

    
    #### READING PARAMETERS ###
    
    # Config param
    default_config_param_yaml_file_name = os.path.join(pkg_path,'config','config_msf_state_estimator.yaml')
    # Declare the parameter with a default value
    self.declare_parameter('config_param_msf_state_estimator_yaml_file', default_config_param_yaml_file_name)
    # Get the parameter value
    config_param_yaml_file_name_str = self.get_parameter('config_param_msf_state_estimator_yaml_file').get_parameter_value().string_value
    self.get_logger().info(config_param_yaml_file_name_str)
    self.config_param_yaml_file_name = os.path.abspath(config_param_yaml_file_name_str)

    ###

    # Load config param
    with open(self.config_param_yaml_file_name,'r') as file:
        # The FullLoader parameter handles the conversion from YAML
        # scalar values to Python the dictionary format
        self.config_param = yaml.load(file, Loader=SafeLoader)['msf_state_estimator']

    if(self.config_param is None):
      self.get_logger().info("Error loading config param msf state estimator")
    else:
      self.get_logger().info("Config param msf state estimator:")
      self.get_logger().info(str(self.config_param))


    # Parameters
    #
    self.robot_frame = self.config_param['robot_frame']
    self.world_frame = self.config_param['world_frame']
    #
    self.state_estim_loop_freq = self.config_param['state_estim_loop_freq']
    
    #
    self.msf_state_estimator.setConfigParameters(self.config_param['ekf'])

    
    # End
    return


  def open(self):

    # Subscribers

    # 
    self.meas_robot_posi_sub = self.create_subscription(PointStamped, 'meas_robot_position', self.measRobotPositionCallback, qos_profile=10)
    # 
    self.meas_robot_atti_sub = self.create_subscription(QuaternionStamped, 'meas_robot_attitude', self.measRobotAttitudeCallback, qos_profile=10)
    #
    self.meas_robot_vel_robot_sub = self.create_subscription(TwistStamped, 'meas_robot_velocity_robot', self.measRobotVelRobotCallback, qos_profile=10)
    


    # Publishers

    # 
    self.estim_robot_pose_pub = self.create_publisher(PoseStamped, 'estim_robot_pose', qos_profile=10)
    # 
    self.estim_robot_pose_cov_pub = self.create_publisher(PoseWithCovarianceStamped, 'estim_robot_pose_cov', qos_profile=10)
    #
    self.estim_robot_vel_robot_pub = self.create_publisher(TwistStamped, 'estim_robot_velocity_robot', qos_profile=10)
    #
    self.estim_robot_vel_robot_cov_pub = self.create_publisher(TwistWithCovarianceStamped, 'estim_robot_velocity_robot_cov', qos_profile=10)
    #
    self.estim_robot_vel_world_pub = self.create_publisher(TwistStamped, 'estim_robot_velocity_world', qos_profile=10)
    #
    self.estim_robot_vel_world_cov_pub = self.create_publisher(TwistWithCovarianceStamped, 'estim_robot_velocity_world_cov', qos_profile=10)


    # Tf2 broadcasters
    self.tf2_broadcaster = tf2_ros.TransformBroadcaster(self)


    # Timers
    #
    self.state_estim_loop_timer = self.create_timer(1.0/self.state_estim_loop_freq, self.stateEstimLoopTimerCallback)


    # End
    return


  def run(self):

    rclpy.spin(self)

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
    header_msg.stamp = self.msf_state_estimator.estim_state_timestamp.to_msg()
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
    estim_robot_pose_tf2_msg = geometry_msgs.msg.TransformStamped()

    estim_robot_pose_tf2_msg.header.stamp = self.msf_state_estimator.estim_state_timestamp.to_msg()
    estim_robot_pose_tf2_msg.header.frame_id = self.world_frame
    estim_robot_pose_tf2_msg.child_frame_id = self.robot_frame

    estim_robot_pose_tf2_msg.transform.translation.x = self.msf_state_estimator.estim_robot_posi[0]
    estim_robot_pose_tf2_msg.transform.translation.y = self.msf_state_estimator.estim_robot_posi[1]
    estim_robot_pose_tf2_msg.transform.translation.z = self.msf_state_estimator.estim_robot_posi[2]

    estim_robot_pose_tf2_msg.transform.rotation.w = self.msf_state_estimator.estim_robot_atti_quat_simp[0]
    estim_robot_pose_tf2_msg.transform.rotation.x = 0.0
    estim_robot_pose_tf2_msg.transform.rotation.y = 0.0
    estim_robot_pose_tf2_msg.transform.rotation.z = self.msf_state_estimator.estim_robot_atti_quat_simp[1]

    # Broadcast
    self.tf2_broadcaster.sendTransform(estim_robot_pose_tf2_msg)


    # End
    return


  def estimRobotVelocityPublish(self):

    #
    # Robot Velocity Wrt world

    # Header
    header_wrt_world_msg = Header()
    header_wrt_world_msg.stamp = self.msf_state_estimator.estim_state_timestamp.to_msg()
    header_wrt_world_msg.frame_id = self.world_frame

    # Twist
    robot_velocity_world_msg = Twist()
    #
    robot_velocity_world_msg.linear.x = self.msf_state_estimator.estim_robot_velo_lin_world[0]
    robot_velocity_world_msg.linear.y = self.msf_state_estimator.estim_robot_velo_lin_world[1]
    robot_velocity_world_msg.linear.z = self.msf_state_estimator.estim_robot_velo_lin_world[2]
    #
    robot_velocity_world_msg.angular.x = 0.0
    robot_velocity_world_msg.angular.y = 0.0
    robot_velocity_world_msg.angular.z = self.msf_state_estimator.estim_robot_velo_ang_world[0]
    
    # TwistStamped
    robot_velocity_world_stamp_msg = TwistStamped()
    robot_velocity_world_stamp_msg.header = header_wrt_world_msg
    robot_velocity_world_stamp_msg.twist = robot_velocity_world_msg

    # TwistWithCovarianceStamped
    # TODO JL Cov
    robot_velocity_world_cov_stamp_msg = TwistWithCovarianceStamped()
    robot_velocity_world_cov_stamp_msg.header = header_wrt_world_msg
    robot_velocity_world_cov_stamp_msg.twist.twist = robot_velocity_world_msg
    # robot_velocity_world_cov_stamp_msg.twist.covariance


    #
    # Robot velocity wrt robot

    # computation estim robot velocity robot
    estim_robot_vel_lin_robot = ars_lib_helpers.Conversions.convertVelLinFromWorldToRobot(self.msf_state_estimator.estim_robot_velo_lin_world, self.msf_state_estimator.estim_robot_atti_quat_simp, flag_quat_simp=True)
    estim_robot_vel_ang_robot = ars_lib_helpers.Conversions.convertVelAngFromWorldToRobot(self.msf_state_estimator.estim_robot_velo_ang_world, self.msf_state_estimator.estim_robot_atti_quat_simp, flag_quat_simp=True)

    # Header
    header_wrt_robot_msg = Header()
    header_wrt_robot_msg.stamp = self.msf_state_estimator.estim_state_timestamp.to_msg()
    header_wrt_robot_msg.frame_id = self.robot_frame

    # Twist
    robot_velocity_robot_msg = Twist()
    #
    robot_velocity_robot_msg.linear.x = estim_robot_vel_lin_robot[0]
    robot_velocity_robot_msg.linear.y = estim_robot_vel_lin_robot[1]
    robot_velocity_robot_msg.linear.z = estim_robot_vel_lin_robot[2]
    #
    robot_velocity_robot_msg.angular.x = 0.0
    robot_velocity_robot_msg.angular.y = 0.0
    robot_velocity_robot_msg.angular.z = estim_robot_vel_ang_robot[0]

    # TwistStamped
    robot_velocity_robot_stamp_msg = TwistStamped()
    robot_velocity_robot_stamp_msg.header = header_wrt_robot_msg
    robot_velocity_robot_stamp_msg.twist = robot_velocity_robot_msg

    # TwistWithCovarianceStamped
    # TODO JL Cov
    robot_velocity_robot_cov_stamp_msg = TwistWithCovarianceStamped()
    robot_velocity_robot_cov_stamp_msg.header = header_wrt_robot_msg
    robot_velocity_robot_cov_stamp_msg.twist.twist = robot_velocity_robot_msg
    # robot_velocity_robot_cov_stamp_msg.twist.covariance


    # Publish
    #
    self.estim_robot_vel_world_pub.publish(robot_velocity_world_stamp_msg)
    # 
    self.estim_robot_vel_world_cov_pub.publish(robot_velocity_world_cov_stamp_msg)

    #
    self.estim_robot_vel_robot_pub.publish(robot_velocity_robot_stamp_msg)
    # 
    self.estim_robot_vel_robot_cov_pub.publish(robot_velocity_robot_cov_stamp_msg)

    # End
    return

    
  def stateEstimLoopTimerCallback(self):

    # Get time
    time_stamp_current = self.get_clock().now()

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

  