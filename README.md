# insight_ros_driver

## Build

```bash
cd /home/right/insight_ws/src/insight9_ros_driver
chmod +x build.sh
./build.sh ROS1
```

## Run

ROS1:
```bash
source devel/setup.bash
roslaunch insight_ros_driver insight9_ros_driver.launch rviz:=true
```
