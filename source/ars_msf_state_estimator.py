#!/usr/bin/env python

import numpy as np
from numpy import *

import os

import threading


# ROS

import rospy

import tf_conversions as tf


import nav_msgs.msg
from nav_msgs.msg import Path



#
import ars_lib_helpers






class ArsMsfStateEstimator:

  #######

  # Meas position
  # z_t = [m_posi_x, m_posi_y, m_posi_z]
  # Dim (z_t) = 3
  flag_set_meas_robot_posi = None
  meas_robot_posi_timestamp = None
  meas_robot_posi = None
  # Meas attitude
  # z_a = [m_atti_yaw]
  # Dim (z_a) = 1
  flag_set_meas_robot_atti = None
  meas_robot_atti_timestamp = None
  meas_robot_atti_quat_simp = None
  # Meas velocity
  # z_v = [m_vel_lin_x_robot, m_vel_lin_y_robot, m_vel_lin_z_robot,
  #       m_vel_ang_z_robot]
  # Dim (z_v) = 4
  flag_set_meas_robot_vel_robot = None
  meas_robot_velo_timestamp = None
  meas_robot_velo_lin_robot = None
  meas_robot_velo_ang_robot = None

  #
  lock_meas = None


  # Estimated State: 
  # x = [ posi_x, posi_y, posi_z, 
  #       atti_yaw, 
  #       vel_lin_x_world, vel_lin_y_world, vel_lin_z_world,
  #       vel_ang_z_world ]
  # Dim(x) = 8
  estim_state_timestamp = None
  # Estimated Pose
  estim_robot_posi = None
  estim_robot_atti_quat_simp = None
  # Estimated Velocity
  estim_robot_velo_lin_world = None
  estim_robot_velo_ang_world = None
  # Cov estimated state
  estim_state_cov = None


  # Covariance of the process model
  cov_proc_mod = None

  # Covariance meas position
  cov_meas_posi = None

  # Covariance meas attitude
  cov_meas_atti = None

  # Covariance meas velocity
  cov_meas_velo_lin = None
  cov_meas_velo_ang = None



  #########

  def __init__(self):

    # Meas Position
    self.flag_set_meas_robot_posi = False
    self.meas_robot_posi_timestamp = rospy.Time()
    self.meas_robot_posi = np.zeros((3,), dtype=float)
    # Meas Attitude
    self.flag_set_meas_robot_atti = False
    self.meas_robot_atti_timestamp = rospy.Time()
    self.meas_robot_atti_quat_simp = ars_lib_helpers.Quaternion.zerosQuatSimp()
    # Meas Velocity
    self.flag_set_meas_robot_vel_robot = False
    self.meas_robot_velo_timestamp = rospy.Time()
    self.meas_robot_velo_lin_robot = np.zeros((3,), dtype=float)
    self.meas_robot_velo_ang_robot = np.zeros((1,), dtype=float)

    #
    self.lock_meas = threading.Lock()

    # Estimated State
    self.estim_state_timestamp = rospy.Time()
    # Estmated Pose
    self.estim_robot_posi = np.zeros((3,), dtype=float)
    self.estim_robot_atti_quat_simp = ars_lib_helpers.Quaternion.zerosQuatSimp()
    # Estimated Velocity
    self.estim_robot_velo_lin_world = np.zeros((3,), dtype=float)
    self.estim_robot_velo_ang_world = np.zeros((1,), dtype=float)
    
    # Cov estimated state
    self.estim_state_cov = np.zeros((8,8), dtype=float)
    self.estim_state_cov = np.diag([1.0, 1.0, 1.0, 
                                    1.0, 
                                    1.0, 1.0, 1.0,
                                    1.0])

    # Covariance of the process model
    self.cov_proc_mod = np.zeros((4,4), dtype=float)
    self.cov_proc_mod = np.diag([1.0, 1.0, 1.0, 
                                    1.0])

    # Covariance meas position
    self.cov_meas_posi = np.diag([1.0, 1.0, 1.0])

    # Covariance meas attitude
    self.cov_meas_atti = np.diag([1.0])

    # Covariance meas velocity
    self.cov_meas_velo_lin = np.diag([1.0, 1.0, 1.0])
    self.cov_meas_velo_ang = np.diag([1.0])


    # End
    return


  def setMeasRobotPosition(self, timestamp, robot_posi):

    self.lock_meas.acquire()

    self.flag_set_meas_robot_posi = True

    self.meas_robot_posi_timestamp = timestamp

    self.meas_robot_posi = robot_posi

    self.lock_meas.release()

    return

  def setMeasRobotAttitude(self, timestamp, robot_atti_quat_simp):

    self.lock_meas.acquire()

    self.flag_set_meas_robot_atti = True

    self.meas_robot_atti_timestamp = timestamp

    self.meas_robot_atti_quat_simp = robot_atti_quat_simp

    self.lock_meas.release()

    return

  def setMeasRobotVelRobot(self, timestamp, lin_vel_world, ang_vel_world):

    self.lock_meas.acquire()

    self.flag_set_meas_robot_vel_robot = True

    self.meas_robot_velo_timestamp = timestamp

    self.meas_robot_velo_lin_robot = lin_vel_world
    self.meas_robot_velo_ang_robot = ang_vel_world

    self.lock_meas.release()

    return

  
  def predict(self, timestamp):

    # Delta time
    delta_time = 0.0
    if(self.estim_state_timestamp == rospy.Time()):
      delta_time = 0.0
    else:
      delta_time = (timestamp - self.estim_state_timestamp).to_sec()

    # State
    estim_x_kk_robot_posi = self.estim_robot_posi
    estim_x_kk_robot_atti_quat_simp = self.estim_robot_atti_quat_simp
    estim_x_kk_robot_velo_lin_world = self.estim_robot_velo_lin_world
    estim_x_kk_robot_velo_ang_world = self.estim_robot_velo_ang_world
    # Cov
    estim_P_kk = self.estim_state_cov


    # Process model

    # Position
    # TODO BY STUDENT
    # estim_x_k1k_robot_posi = 

    # Attitude
    delta_robot_atti_ang = delta_time * estim_x_kk_robot_velo_ang_world
    delta_robot_atti_quat_sim = ars_lib_helpers.Quaternion.quatSimpFromAngle(delta_robot_atti_ang)
    estim_x_k1k_robot_atti_quat_simp = ars_lib_helpers.Quaternion.quatSimpProd(estim_x_kk_robot_atti_quat_simp, delta_robot_atti_quat_sim)

    # Velocity Linear
    # Constant
    # TODO BY STUDENT
    # estim_x_k1k_robot_velo_lin_world = 

    # Velocity Angular
    # Constant
    estim_x_k1k_robot_velo_ang_world = estim_x_kk_robot_velo_ang_world


    # Jacobian - Fx
    Fx = np.zeros((8,8), dtype=float)
    # Position k+1 - Position k
    # TODO BY STUDENT
    # Fx[0:3, 0:3] = 
    # Position k+1 - Velocity k
    # Fx[0:3, 4:7] = 
    # Attitude k+1 - Attitude k
    Fx[3, 3] = 1.0
    Fx[3, 7] = delta_time
    # Velocity linear k+1 - Velocity linear k
    # TODO BY STUDENT
    # Fx[4:7, 4:7] = 
    # Velocity angular k+1 - Velocity angular k
    Fx[7, 7] = 1.0


    # Jacobian - Fn
    Fn = np.zeros((8,4), dtype=float)
    # Velocity linear k+1 - Noise Velocity linear
    # TODO BY STUDENT
    # Fn[4:7, 0:3] = 
    # Velocity angular k+1 - Noise Velocity angular
    Fn[7,3] = 1.0


    # Covariance
    # TODO BY STUDENT
    # estim_P_k1k = 
    


    # Prepare for next iteration
    #
    self.estim_state_timestamp = timestamp
    #
    self.estim_robot_posi = estim_x_k1k_robot_posi
    self.estim_robot_atti_quat_simp = estim_x_k1k_robot_atti_quat_simp
    self.estim_robot_velo_lin_world = estim_x_k1k_robot_velo_lin_world
    self.estim_robot_velo_ang_world = estim_x_k1k_robot_velo_ang_world
    #
    self.estim_state_cov = estim_P_k1k

    #
    return


  def update(self):

    # Lock
    self.lock_meas.acquire()

    # Measurements readings - To avoid races
    #
    flag_set_meas_robot_posi = self.flag_set_meas_robot_posi
    if(flag_set_meas_robot_posi):
      meas_z_robot_posi = self.meas_robot_posi
    #
    flag_set_meas_robot_atti = self.flag_set_meas_robot_atti
    if(flag_set_meas_robot_atti):
      meas_z_robot_atti_quat_simp = self.meas_robot_atti_quat_simp
    #
    flag_set_meas_robot_vel_robot = self.flag_set_meas_robot_vel_robot
    if(flag_set_meas_robot_vel_robot):
      meas_z_robot_velo_lin_robot = self.meas_robot_velo_lin_robot
      meas_z_robot_velo_ang_robot = self.meas_robot_velo_ang_robot

    # Put flags measurements down once used
    if(self.flag_set_meas_robot_posi == True):
      self.flag_set_meas_robot_posi = False

    if(self.flag_set_meas_robot_atti == True):
      self.flag_set_meas_robot_atti = False

    if(self.flag_set_meas_robot_vel_robot == True):
      self.flag_set_meas_robot_vel_robot = False

    # Release
    self.lock_meas.release()


    # Dimension of the measurement for update
    # Init
    dim_meas = 0
    #
    if(flag_set_meas_robot_posi == True):
      dim_meas += 3
    #
    if(flag_set_meas_robot_atti == True):
      dim_meas += 1
    #
    if(flag_set_meas_robot_vel_robot == True):
      dim_meas += 4


    # Check that there is at least one measurement
    if(dim_meas == 0):
      return


    # State readings - To avoid races
    estim_x_k1k_robot_posi = self.estim_robot_posi
    estim_x_k1k_robot_atti_quat_simp = self.estim_robot_atti_quat_simp
    estim_x_k1k_robot_velo_lin_world = self.estim_robot_velo_lin_world
    estim_x_k1k_robot_velo_ang_world = self.estim_robot_velo_ang_world
    # Cov
    estim_P_k1k = self.estim_state_cov

    # Rotation matrix robot atti
    estim_x_k1k_robot_atti_rot_mat = np.zeros((3,3), dtype=float)
    estim_x_k1k_robot_atti_rot_mat = ars_lib_helpers.Quaternion.rotMat3dFromQuatSimp(estim_x_k1k_robot_atti_quat_simp)



    # Innovation of the measurement
    innov_meas = np.zeros((dim_meas,), dtype=float)
    innov_meas_idx = 0
    if(flag_set_meas_robot_posi == True):
      # Predicted measurement
      # TODO BY STUDENT
      # pred_z_robot_posi = 
      # Innovation of the measurement
      # TODO BY STUDENT
      # innov_meas_robot_posi = 
      # To the innovation vector
      innov_meas[innov_meas_idx:innov_meas_idx+3] = innov_meas_robot_posi
      innov_meas_idx += 3

    if(flag_set_meas_robot_atti == True):
      # Predicted measurement
      pred_z_robot_atti_quat_simp = estim_x_k1k_robot_atti_quat_simp
      # Innovation of the measurement
      innov_meas_robot_atti_quat_simp = ars_lib_helpers.Quaternion.computeDiffQuatSimp(pred_z_robot_atti_quat_simp, meas_z_robot_atti_quat_simp)
      # Converting to angle
      innov_meas_robot_atti_angle = ars_lib_helpers.Quaternion.angleFromQuatSimp(innov_meas_robot_atti_quat_simp)
      # To the innovation vector
      innov_meas[innov_meas_idx:innov_meas_idx+1] = innov_meas_robot_atti_angle
      innov_meas_idx += 1

    if(flag_set_meas_robot_vel_robot == True):
      # Predicted measurement
      # TODO BY STUDENT
      # pred_z_robot_velo_lin_robot = 
      pred_z_robot_velo_ang_robot = 1.0 * estim_x_k1k_robot_velo_ang_world
      # Innovation of the measurement
      # TODO BY STUDENT
      # innov_meas_robot_velo_lin_robot = 
      innov_meas_robot_velo_ang_robot = pred_z_robot_velo_ang_robot - meas_z_robot_velo_ang_robot
      # To the innovation vector
      innov_meas[innov_meas_idx:innov_meas_idx+3] = innov_meas_robot_velo_lin_robot
      innov_meas_idx += 3
      innov_meas[innov_meas_idx:innov_meas_idx+1] = innov_meas_robot_velo_ang_robot
      innov_meas_idx += 1


    # Covariance of the measurement
    cov_meas = np.zeros((dim_meas,dim_meas), dtype=float)
    cov_meas_idx = 0
    if(flag_set_meas_robot_posi == True):
      cov_meas[cov_meas_idx:cov_meas_idx+3, cov_meas_idx:cov_meas_idx+3] = self.cov_meas_posi
      cov_meas_idx += 3

    if(flag_set_meas_robot_atti == True):
      cov_meas[cov_meas_idx:cov_meas_idx+1, cov_meas_idx:cov_meas_idx+1] = self.cov_meas_atti 
      cov_meas_idx += 1

    if(flag_set_meas_robot_vel_robot == True):
      cov_meas[cov_meas_idx:cov_meas_idx+3, cov_meas_idx:cov_meas_idx+3] = self.cov_meas_velo_lin
      cov_meas_idx += 3
      cov_meas[cov_meas_idx:cov_meas_idx+1, cov_meas_idx:cov_meas_idx+1] = self.cov_meas_velo_ang
      cov_meas_idx += 1


    # Jacobian Hx
    Hx = np.zeros((dim_meas,8), dtype=float)
    Hx_idx = 0
    if(flag_set_meas_robot_posi == True):
      # Meas robot posi - robot posi
      # TODO BY STUDENT
      # Hx[Hx_idx:Hx_idx+3, 0:3] = 
      Hx_idx += 3

    if(flag_set_meas_robot_atti == True):
      # Meas robot atti - robot atti
      Hx[Hx_idx:Hx_idx+1, 3] = 1.0
      Hx_idx += 1

    if(flag_set_meas_robot_vel_robot == True):
      # Meas velo lin - robot atti
      # TODO BY STUDENT
      # Hx[Hx_idx:Hx_idx+3, 3] = 
      # Meas velo lin - robot velo lin
      # TODO BY STUDENT
      # Hx[Hx_idx:Hx_idx+3, 4:7] = 
      Hx_idx += 3
      # Meas velo ang - robot velo ang
      Hx[Hx_idx:Hx_idx+1, 7] = 1.0
      Hx_idx += 1


    # Covariance of the innovation of the measurement
    # TODO BY STUDENT
    # cov_innov_meas = 


    # Kalman Gain
    # TODO BY STUDENT
    # kalman_gain = 


    # Updated state
    delta_x = np.matmul(kalman_gain, innov_meas)

    # Robot posi
    delta_x_robot_posi = delta_x[0:3]
    estim_x_k1k1_robot_posi = estim_x_k1k_robot_posi - delta_x_robot_posi
    # Robot attitude
    delta_x_robot_atti_ang = delta_x[3]
    delta_x_robot_atti_quat_sim = ars_lib_helpers.Quaternion.quatSimpFromAngle(delta_x_robot_atti_ang)
    estim_x_k1k1_robot_atti_quat_simp = ars_lib_helpers.Quaternion.computeDiffQuatSimp(estim_x_k1k_robot_atti_quat_simp, delta_x_robot_atti_quat_sim) 
    # Velocity linear 
    delta_x_robot_velo_lin_world = delta_x[4:7]
    estim_x_k1k1_robot_velo_lin_world = estim_x_k1k_robot_velo_lin_world - delta_x_robot_velo_lin_world
    # Velocity angular
    delta_x_robot_velo_lin_world = delta_x[7]
    estim_x_k1k1_robot_velo_ang_world = estim_x_k1k_robot_velo_ang_world - delta_x_robot_velo_lin_world


    # Updated covariance of state
    # TODO BY STUDENT
    # estim_P_k1k1 = 


    
    # Prepare for next iteration
    #
    self.estim_robot_posi = estim_x_k1k1_robot_posi
    self.estim_robot_atti_quat_simp = estim_x_k1k1_robot_atti_quat_simp
    self.estim_robot_velo_lin_world = estim_x_k1k1_robot_velo_lin_world
    self.estim_robot_velo_ang_world = estim_x_k1k1_robot_velo_ang_world
    #
    self.estim_state_cov = estim_P_k1k1

    
    #
    return
