# performance-testing

This repo is meant to serve as a testing rig for benchmarking network performance of the k4a publisher.

## Usage
### Python subscriber
After sourcing ros2 rolling installation, run :

```
ros2 launch launch/pubsub_py.py
```
I got a throughput of around 270 MB/s using the above launch file.

This should output the following result:
```
[subscriber.py-2] ------------------
[subscriber.py-2] For topic /k4a/color/camera_info
[subscriber.py-2] Latency mean(ms): 2.384
[subscriber.py-2] Latency minimum(ms): 1.118
[subscriber.py-2] Latency maximum(ms): 31.657
[subscriber.py-2] Latency std dev(ms): 4.056
[subscriber.py-2] Msg size (bytes): 406
[subscriber.py-2] Throughput (MB/sec): 0.009
[subscriber.py-2] ------------------
[subscriber.py-2] [INFO] [1686098955.507461132] [minimal_subscriber]: 
[subscriber.py-2] ------------------
[subscriber.py-2] For topic /k4a/depth/camera_info
[subscriber.py-2] Latency mean(ms): 2.98
[subscriber.py-2] Latency minimum(ms): 1.395
[subscriber.py-2] Latency maximum(ms): 31.846
[subscriber.py-2] Latency std dev(ms): 4.079
[subscriber.py-2] Msg size (bytes): 406
[subscriber.py-2] Throughput (MB/sec): 0.009
[subscriber.py-2] ------------------
[subscriber.py-2] [INFO] [1686098955.519252154] [minimal_subscriber]: 
[subscriber.py-2] ------------------
[subscriber.py-2] For topic /k4a/depth/image
[subscriber.py-2] Latency mean(ms): 20.698
[subscriber.py-2] Latency minimum(ms): 4.829
[subscriber.py-2] Latency maximum(ms): 309.19
[subscriber.py-2] Latency std dev(ms): 29.585
[subscriber.py-2] Msg size (bytes): 737381
[subscriber.py-2] Throughput (MB/sec): 17.521
[subscriber.py-2] ------------------
[subscriber.py-2] [INFO] [1686098955.575008506] [minimal_subscriber]: 
[subscriber.py-2] ------------------
[subscriber.py-2] For topic /k4a/color/image
[subscriber.py-2] Latency mean(ms): 16.545
[subscriber.py-2] Latency minimum(ms): 5.549
[subscriber.py-2] Latency maximum(ms): 279.369
[subscriber.py-2] Latency std dev(ms): 27.316
[subscriber.py-2] Msg size (bytes): 3686501
[subscriber.py-2] Throughput (MB/sec): 85.765
[subscriber.py-2] ------------------
[subscriber.py-2] [INFO] [1686098955.587713500] [minimal_subscriber]: 
[subscriber.py-2] ------------------
[subscriber.py-2] For topic /k4a/points
[subscriber.py-2] Latency mean(ms): 21.45
[subscriber.py-2] Latency minimum(ms): 15.271
[subscriber.py-2] Latency maximum(ms): 37.582
[subscriber.py-2] Latency std dev(ms): 4.71
[subscriber.py-2] Msg size (bytes): 5898422
[subscriber.py-2] Throughput (MB/sec): 152.461
[subscriber.py-2] ------------------
```
