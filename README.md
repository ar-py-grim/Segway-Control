# Segway
Segway motion using ROS2 Jazzy

to view urdf file use
```
ros2 launch urdf_launch display.launch.py urdf_package:=segway urdf_package_path:=urdf/robot/segway.urdf
```
create sdf file from urdf file
```
gz sdf -p segway.urdf > segway.sdf
```
To view all gazebo topics
```
gz topic -l
```
To view for all available options
```
gz topic -h
```
```
xacro segway.urdf.xacro > segway.urdf
```

```
gz service -s /world/default/set_pose/blocking \
  --reqtype gz.msgs.Pose \
  --reptype gz.msgs.Boolean \
  --timeout 2000 \
  --req "name: 'segway' position: {x: 9.0 y: 0.0 z: 0.05} orientation: {x: 0.0 y: 0.0 z: 0.707 w: 0.707}"
```
