#!/usr/bin/env python3

import rclpy

from ars_msf_state_estimator.ars_msf_state_estimator_ros import ArsMsfStateEstimatorRos


def main(args=None):

  rclpy.init(args=args)

  ars_msf_state_estimator_ros = ArsMsfStateEstimatorRos()

  ars_msf_state_estimator_ros.open()

  try:
      ars_msf_state_estimator_ros.run()
  except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
      # Graceful shutdown on interruption
      pass
  finally:
    ars_msf_state_estimator_ros.destroy_node()
    rclpy.try_shutdown()

  return 0


''' MAIN '''
if __name__ == '__main__':

  main()