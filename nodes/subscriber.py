#!/usr/bin/env python3

from collections import defaultdict
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import CameraInfo

from collections import defaultdict
import numpy as np
import sys
import time

BUFFER_SIZE = 100
THROUGHPUT = 0

def get_statistics(msg_info, msg_size, topic):
    global THROUGHPUT
    msg_info = np.array(msg_info)
    # Ignore the first entry, as we serialize the msg to get its size when the callback is run the first time.
    # This makes the callback run much longer.
    latencies = msg_info[1:,0]
    current_throughput = round((msg_size * len(latencies))/((msg_info[-1,1] - msg_info[1,1]) * 1024 *1024), 3)
    THROUGHPUT += current_throughput

    report = "\n------------------" +\
    "\nFor topic " + topic +\
    "\nLatency mean(ms): " + str(round(np.mean(latencies) * 1000, 3)) +\
    "\nLatency minimum(ms): " + str(round(min(latencies) * 1000, 3)) +\
    "\nLatency maximum(ms): " + str(round(max(latencies) * 1000, 3)) +\
    "\nLatency std dev(ms): " + str(round(np.std(latencies) * 1000, 3)) +\
    "\nMsg size (bytes): " + str(msg_size) +\
    "\nThroughput (MB/sec): " + str(current_throughput) +\
    "\nCumulative throughput (MB/sec): " + str(THROUGHPUT) +\
    "\n------------------"
    return report

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('subscriber_py')

        self.data_buffer = defaultdict(list)
        self.msg_sizes = defaultdict(int)
        topics_and_types = [('/k4a/points', PointCloud2), ('/k4a/color/image', Image), ('/k4a/depth/image', Image),
                            ('/k4a/color/camera_info', CameraInfo), ('/k4a/depth/camera_info', CameraInfo)]
        self.subscribers = []

        for topic, type in topics_and_types :
            self.subscribers.append(self.create_subscription(
                type,
                topic,
                self.msg_callback_generator(topic),
                10))

    def msg_callback_generator(self, topic):
        def msg_callback(msg):
            if len(self.data_buffer[topic]) == BUFFER_SIZE:
                return

            src_time = float(str(msg.header.stamp.sec) + "." + str(msg.header.stamp.nanosec))
            current_time = float(time.time())
            latency = current_time - src_time

            if self.msg_sizes[topic] == 0:
                bytes = serialize_message(msg)
                self.msg_sizes[topic] = sys.getsizeof(bytes)

            if latency > 0 and len(self.data_buffer[topic]) < BUFFER_SIZE:
                self.data_buffer[topic].append([latency, current_time])
 
            if latency < 0 :
                self.get_logger().info("Timing mismatch in " + topic + " , dropping msg")

            if len(self.data_buffer[topic]) == BUFFER_SIZE :
                self.get_logger().info(get_statistics((self.data_buffer[topic]), self.msg_sizes[topic] , topic))

        return msg_callback

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
