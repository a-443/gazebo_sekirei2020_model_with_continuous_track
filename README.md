# gazebo_sekirei2020_model_with_continuous_track

## description
This package includes the robot model for gazebo. The model has 6 continuous tracks and 2 RGB cameras. Continuous tracks are simulated on the plugin shown following. Also this package was developed referencing the example developed by plugin developer.

* plugin: [gazebo_continuous_track][1]
* example developed by plugin developer: [gazebo_continuous_track_example][2]

[1]:https://github.com/yoshito-n-students/gazebo_continuous_track
[2]:https://github.com/yoshito-n-students/gazebo_continuous_track_example


### How to get
This model can work on only [the plugin][1] shown above, so confirm those packages at first.

And then, type following commands on the terminal:
```
cd <your ROS workspace>/src
git clone
cd ..
catkin_make
```


## How to run
1. launch gazebo and spawn the model
```
roslaunch gazebo_sekirei2020_model_with_continuous_track demo.launch
```
2. send a velocity command
```
rostopic pub -1 /cmd_vel std_msgs/Float64 "data: 0.5"
```

## Related packages
* GUI
* control system


## Note
* I don't maintain this package.
* This is the first time I've published something on github, so let me know if there are any problem.