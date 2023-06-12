# performance-testing

This repo is meant to serve as a testing rig for benchmarking network performance of the k4a publisher.

## Usage
### python publisher + python subscriber
After sourcing ros2 rolling installation, run :

```
git clone https://github.com/adityapande-1995/performance-testing.git
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

## Using mininet

[Mininet](https://github.com/mininet/mininet) is a network simulator.

We can start mininet in a basic setup. One a new terminal and run: 
```
sudo mn --link tc,bw=2048000,delay=1ms
```
The bandwidth has to be supplied in Mbit/s.

Then in the mininet prompt, run :
```
mininet> h1 . /opt/ros/rolling/setup.bash ; python3 ~/performance-testing/nodes/publisher.py &
mininet> h2 . /opt/ros/rolling/setup.bash ; python3 ~/performance-testing/nodes/subscriber.py &
```

With these parameters, I was able to get a cumulative throughput of 15 MB/s, and it took 149s to complete, probably due to high packet drop.
