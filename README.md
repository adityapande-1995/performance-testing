# performance-testing

This repo is meant to serve as a testing rig for benchmarking network performance of the k4a publisher.

## Usage
### python publisher + python subscriber
After sourcing ros2 rolling installation, run :

```
ros2 launch launch/pubsub_py.py
```
I got a throughput of around 270 MB/s (approx 15.82 GB/s) using the above launch file.

### python publisher + cpp subscriber
After sourcing ros2 rolling installation, build the subscriber cpp node :
```
cd nodes/subscriber_cpp/build
cmake ..
make
```
Then use the launch script :

```
ros2 launch launch/pubsub_cpp.py
```
With this, I got a throughput of 287 MB/s (approx 16.81 GB/s).
