# performance-testing

This repo is meant to serve as a testing rig for benchmarking network performance of the k4a publisher.

## Basic usage
The setup here includes a k4a camera publisher, and 2 subscriber nodes, one written using rclpy, and the other using rclcpp.
Effectively both of them do the same job, they subscriber to the pointcloud2 and image topics and record their size and timestamps.
The launch scripts below use the python publisher at 30 Hz and one of the subscribers.

### python publisher + python subscriber
After sourcing ros2 rolling installation, run :

```
git clone https://github.com/adityapande-1995/performance-testing.git
cd performance-testing
ros2 launch launch/pubsub_py.py
```
I got a throughput of around 270 MB/s (approx 15.82 GB/min) using the above launch file. You should get an output like this : 
```
subscriber.py-2] ------------------
[subscriber.py-2] For topic /k4a/depth/image
[subscriber.py-2] Latency mean(ms): 17.271
[subscriber.py-2] Latency minimum(ms): 5.18
[subscriber.py-2] Latency maximum(ms): 38.983
[subscriber.py-2] Latency std dev(ms): 7.337
[subscriber.py-2] Msg size (bytes): 737381
[subscriber.py-2] Throughput (MB/sec): 19.541
[subscriber.py-2] Cumulative throughput (MB/sec): 19.541
[subscriber.py-2] ------------------
[subscriber.py-2] [INFO] [1686600972.826939693] [subscriber_py]: 
[subscriber.py-2] ------------------
[subscriber.py-2] For topic /k4a/color/camera_info
[subscriber.py-2] Latency mean(ms): 3.063
[subscriber.py-2] Latency minimum(ms): 1.159
[subscriber.py-2] Latency maximum(ms): 39.322
[subscriber.py-2] Latency std dev(ms): 5.906
[subscriber.py-2] Msg size (bytes): 406
[subscriber.py-2] Throughput (MB/sec): 0.011
[subscriber.py-2] Cumulative throughput (MB/sec): 19.552
[subscriber.py-2] ------------------
[subscriber.py-2] [INFO] [1686600972.837119874] [subscriber_py]: 
[subscriber.py-2] ------------------
[subscriber.py-2] For topic /k4a/color/image
[subscriber.py-2] Latency mean(ms): 14.711
[subscriber.py-2] Latency minimum(ms): 6.026
[subscriber.py-2] Latency maximum(ms): 38.141
[subscriber.py-2] Latency std dev(ms): 7.269
[subscriber.py-2] Msg size (bytes): 3686501
[subscriber.py-2] Throughput (MB/sec): 96.958
[subscriber.py-2] Cumulative throughput (MB/sec): 116.50999999999999
[subscriber.py-2] ------------------
[subscriber.py-2] [INFO] [1686600972.861404797] [subscriber_py]: 
[subscriber.py-2] ------------------
[subscriber.py-2] For topic /k4a/depth/camera_info
[subscriber.py-2] Latency mean(ms): 3.677
[subscriber.py-2] Latency minimum(ms): 1.389
[subscriber.py-2] Latency maximum(ms): 39.511
[subscriber.py-2] Latency std dev(ms): 5.803
[subscriber.py-2] Msg size (bytes): 406
[subscriber.py-2] Throughput (MB/sec): 0.011
[subscriber.py-2] Cumulative throughput (MB/sec): 116.52099999999999
[subscriber.py-2] ------------------
[subscriber.py-2] [INFO] [1686600972.888072416] [subscriber_py]: 
[subscriber.py-2] ------------------
[subscriber.py-2] For topic /k4a/points
[subscriber.py-2] Latency mean(ms): 21.721
[subscriber.py-2] Latency minimum(ms): 14.591
[subscriber.py-2] Latency maximum(ms): 39.087
[subscriber.py-2] Latency std dev(ms): 5.078
[subscriber.py-2] Msg size (bytes): 5898422
[subscriber.py-2] Throughput (MB/sec): 156.076
[subscriber.py-2] Cumulative throughput (MB/sec): 272.597
[subscriber.py-2] ------------------
```

### python publisher + cpp subscriber
After sourcing the ros2 rolling installation, build the subscriber cpp node :
```
cd nodes/subscriber_cpp/build
cmake ..
make
```
Then use the launch script :

```
ros2 launch launch/pubsub_cpp.py
```
With this, I got a throughput of 287 MB/s (approx 16.81 GB/min). You should get an output something like this :
```
[subscriber-2] --------------
[subscriber-2] For topic: /k4a/depth/camera_info
[subscriber-2] Msg size (bytes): 373
[subscriber-2] Throughput (MB/s): 0.000000
[subscriber-2] Cumulative throughput (MB/s): 0.000000
[subscriber-2] --------------
[subscriber-2] [INFO] [1686601107.395284237] [subscriber_cpp]: 
[subscriber-2] --------------
[subscriber-2] For topic: /k4a/color/camera_info
[subscriber-2] Msg size (bytes): 373
[subscriber-2] Throughput (MB/s): 0.000000
[subscriber-2] Cumulative throughput (MB/s): 0.000000
[subscriber-2] --------------
[subscriber-2] [INFO] [1686601107.403397357] [subscriber_cpp]: 
[subscriber-2] --------------
[subscriber-2] For topic: /k4a/color/image
[subscriber-2] Msg size (bytes): 3686468
[subscriber-2] Throughput (MB/s): 95.879223
[subscriber-2] Cumulative throughput (MB/s): 95.879223
[subscriber-2] --------------
[subscriber-2] [INFO] [1686601107.412250985] [subscriber_cpp]: 
[subscriber-2] --------------
[subscriber-2] For topic: /k4a/depth/image
[subscriber-2] Msg size (bytes): 737348
[subscriber-2] Throughput (MB/s): 19.326374
[subscriber-2] Cumulative throughput (MB/s): 115.205597
[subscriber-2] --------------
[subscriber-2] [INFO] [1686601107.492810639] [subscriber_cpp]: 
[subscriber-2] --------------
[subscriber-2] For topic: /k4a/points
[subscriber-2] Msg size (bytes): 5898389
[subscriber-2] Throughput (MB/s): 169.896649
[subscriber-2] Cumulative throughput (MB/s): 285.102246
[subscriber-2] --------------
```

## Testing over a simulated network using mininet

[Mininet](https://github.com/mininet/mininet) is a network simulator.

We can start mininet in a basic setup. Open a new terminal and run: 
```
sudo mn --link tc,bw=1000,delay=1ms
```
The bandwidth has to be supplied in Mbit/s. This starts a minimal simulated network between 2 hosts h1 and h2.
We can run the publisher on one host and the subscriber on the other.

In the mininet prompt, run :
```
mininet> h1 . /opt/ros/rolling/setup.bash ; python3 ~/performance-testing/nodes/publisher.py &
mininet> h2 . /opt/ros/rolling/setup.bash ; python3 ~/performance-testing/nodes/subscriber.py
```
