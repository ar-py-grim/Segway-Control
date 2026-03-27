Packages used </br>
1. pynput == 1.7.6
2. deap == 1.3.1
3. trimesh == 4.11.3

To run the simulation to get PID coefficients
```
ros2 launch segway segway_sim.launch.py train:=true 
```

To run the simulation use
```
ros2 launch segway segway_sim.launch.py 
```
<img width="3072" height="1524" alt="Screenshot from 2026-03-22 22-55-04" src="https://github.com/user-attachments/assets/76abb70c-2ec3-4490-b50f-8830d3063a9e" />

For Gazebo Harmonic on system which has hybrid gpus add below lines in .bashrc
```
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
```
### References
**Gazebo Harmonic:** https://gazebosim.org/docs/harmonic/troubleshooting/ </br>
**Demo link:** https://youtu.be/wS8NtGUcq1I
