# hdt moveit demo

## pick & place demo

```bash
roslaunch hdt_angler_bringup angler_a1_bringup.launch
roslaunch realsense2_camera rs_camera.launch
roslaunch hdt_calibration single.launch
roslaunch hdt_calibration static_transform_publisher.launch

rosrun hdt_moveit_demo hdt_pick&place_demo.py
```

![demo](video/hdt_moveit_pick&place_demo.gif)