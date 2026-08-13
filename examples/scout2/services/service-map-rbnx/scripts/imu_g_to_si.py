#!/usr/bin/env python3
# Livox MID360 IMU publishes linear_acceleration in G (1.0 at rest), not
# m/s^2. SLAM pipelines that expect SI units (DLIO, KISS-ICP, any
# upstream FAST-LIO) blow up because the bias estimator saturates trying
# to close an 8.8 m/s^2 gap. This node is a thin republisher that scales
# linear_acceleration by g and leaves everything else untouched.
#
# Input  topic (env IMU_IN):  /livox/imu
# Output topic (env IMU_OUT): /livox/imu_si
import os, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

G = 9.80665

class ImuGtoSI(Node):
    def __init__(self):
        super().__init__('imu_g_to_si')
        self.topic_in = os.environ.get('IMU_IN', '/livox/imu')
        self.topic_out = os.environ.get('IMU_OUT', '/livox/imu_si')
        self.pub = self.create_publisher(Imu, self.topic_out, 200)
        self.sub = self.create_subscription(Imu, self.topic_in, self.cb, 200)
        self.get_logger().info(f'IMU G->SI: {self.topic_in} -> {self.topic_out} (x {G})')

    def cb(self, msg: Imu):
        msg.linear_acceleration.x *= G
        msg.linear_acceleration.y *= G
        msg.linear_acceleration.z *= G
        self.pub.publish(msg)

def main():
    rclpy.init()
    n = ImuGtoSI()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally:
        try: n.destroy_node()
        except Exception: pass
        try: rclpy.shutdown()
        except Exception: pass

if __name__ == '__main__':
    main()
