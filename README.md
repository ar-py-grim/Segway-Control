Packages used
pynput == 1.7.6

To run the simulation use
```
ros2 launch segway segway_sim.launch.py 
```
For Gazebo Harmonic on system which has hybrid gpus add below lines in .bashrc
```
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
```
