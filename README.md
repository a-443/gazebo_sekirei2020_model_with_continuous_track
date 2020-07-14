# gazebo_sekirei2020_model_with_continuous_track

## About this package
This package includes the robot model for gazebo. The model has 6 continuous tracks and 2 RGB cameras. Continuous tracks are simulated on the plugin shown following. Also this package was developed referencing the example by plugin developer.

* plugin: [gazebo_continuous_track][1]
* example by plugin developer: [gazebo_continuous_track_example][2]

[1]:https://github.com/yoshito-n-students/gazebo_continuous_track
[2]:https://github.com/yoshito-n-students/gazebo_continuous_track_example


## How to get
This model can work on only [the plugin][1] shown above, so confirm those packages at first.

And then, type following commands on the terminal:
```
cd <your ROS workspace>/src
git clone https://github.com/a-443/gazebo_sekirei2020_model_with_continuous_track.git
cd ..
catkin_make
```


## How to run example
1. launch gazebo and spawn the model
```
roslaunch gazebo_sekirei2020_model_with_continuous_track demo.launch
```
2. send a velocity command to all crawlers
```
rostopic pub -1 /cmd_vel std_msgs/Float64 "data: 0.5"
```
3. send a position command to joint between main crawler and flipper
```
rostopic pub -1 /cmd_pos std_msgs/Float64 "data: 0.5"