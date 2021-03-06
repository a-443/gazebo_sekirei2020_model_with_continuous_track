# gazebo_sekirei2020_model_with_continuous_track

![image](https://user-images.githubusercontent.com/34676615/89340159-43fba000-d6da-11ea-8af0-af709ff91d04.png)


This package includes the robot model for gazebo. The model has 6 continuous tracks and 2 RGB cameras. Continuous tracks are simulated on the plugin shown following. Also this package was designed referring to the example by plugin developer.

* plugin: [gazebo_continuous_track][1]
* example by plugin developer: [gazebo_continuous_track_example][2]

[1]:https://github.com/yoshito-n-students/gazebo_continuous_track
[2]:https://github.com/yoshito-n-students/gazebo_continuous_track_example


## How to get
This model can work on only [the plugin][1] shown above, so confirm those packages at first.

And then, type following commands on the terminal:
```
cd <your ROS workspace>/src
git clone git@github.com:a-443/gazebo_sekirei2020_model_with_continuous_track.git
cd ..
catkin_make
```


## How to run example
1. launch gazebo and spawn the model
```
roslaunch gazebo_sekirei2020_model_with_continuous_track demo.launch
```
2. press the play button

3. send a velocity command to all crawlers
```
rostopic pub -1 /cmd_vel std_msgs/Float64 "data: 0.5"
```
4. send a position command to joint between main crawler and flipper
```
rostopic pub -1 /cmd_pos std_msgs/Float64 "data: 0.5"
```


## Topics
### Subscribed topics
* /desired_left_crawler_cmd (std_msgs/Float64)

  All left crawlers(1 main crawler and 2 sub crawlers) move at the value of this topic.

* /desired_right_crawler_cmd (std_msgs/Float64)

  All right crawlers(1 main crawler and 2 sub crawlers) move at the value of this topic.

* /sekirei2020/front_left_flipper_position_controller/command (std_msgs/Float64)

  The front left flipper rotates by the value of this topic (radian).

* /sekirei2020/front_right_flipper_position_controller/command (std_msgs/Float64)

  The front right flipper rotates by the value of this topic (radian).

* /sekirei2020/rear_left_flipper_position_controller/command (std_msgs/Float64)

  The rear left flipper rotates by the value of this topic (radian).

* /sekirei2020/rear_right_flipper_position_controller/command (std_msgs/Float64)

  The rear right flipper rotates by the value of this topic (radian).