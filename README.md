# insight_ros_driver

## Build

ROS1:
```bash
./build.sh ROS1
```

ROS2:
```bash
./build.sh ROS2
```

## Run

ROS1:
```bash
source devel/setup.bash
roslaunch insight_ros_driver insight9_ros_driver.launch rviz:=true
```

ROS2:
```bash
source install/setup.bash
ros2 launch insight_ros_driver insight9_ros_driver.launch.py
```
