#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2

import numpy as np
import sys
import time

BUFFER_SIZE = 50

def get_statistics(msg_info):
    msg_info = np.array(msg_info)
    latencies = msg_info[:,0]
    report = "\n------------------" +\
    "\nFor topic /k4a/points :" +\
    "\nLatency mean(ms): " + str(round(np.mean(latencies) * 1000, 3)) +\
    "\nLatency minimum(ms): " + str(round(min(latencies) * 1000, 3)) +\
    "\nLatency maximum(ms): " + str(round(max(latencies) * 1000, 3)) +\
    "\nLatency std dev(ms): " + str(round(np.std(latencies) * 1000, 3)) +\
    "\nAverage msg size (bytes): " + str(np.mean(msg_info[:,1])) +\
    "\nThroughput (bytes/sec): " + str(round(np.sum(msg_info[:,1])/(msg_info[-1,2] - msg_info[0,2]), 3)) +\
    "\n------------------"
    return report

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.pointcloud_subscription = self.create_subscription(
            PointCloud2,
            '/k4a/points',
            self.pointcloud_callback,
            10)
        self.pointcloud_buffer = []

    def pointcloud_callback(self, msg):
        if len(self.pointcloud_buffer) == BUFFER_SIZE:
            return

        sec = str(msg.header.stamp.sec)
        nsec = str(msg.header.stamp.nanosec)
        src_time = float(sec + "." + nsec)
        current_time = float(time.time())
        latency = current_time - src_time

        if latency > 0 and len(self.pointcloud_buffer) < BUFFER_SIZE:
            self.pointcloud_buffer.append([latency, sys.getsizeof(msg), current_time])
        
        if latency < 0 :
            self.get_logger().info("Timing mismatch in /k4a/points, dropping msg")

        if len(self.pointcloud_buffer) == BUFFER_SIZE :
            self.get_logger().info(get_statistics((self.pointcloud_buffer)))

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
