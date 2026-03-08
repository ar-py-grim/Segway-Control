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
