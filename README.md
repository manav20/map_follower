# map_follower
The `map_follower` is a ROS package for autonomous navigation and control of an ackermann steered race car. The software takes 2D lidar data and IMU to localise itself on a given map and race along a predefined trajectory. </br></br>
The software is tested on UBUNTU 16.04 with ROS Kinetic.
## License

## Compiling
The software is a standard catkin package. Make sure the package is on `ROS_PACKAGE_PATH` after cloning the package to your workspace.
```
cd your_work_space
catkin_make
```
## Example 1 - Hallway
### configure map_server
Upload hallmap.pgm and hallmap.yaml in the resource folder. </br>
Update hallmap.launch with the address of hallmap.yaml file.
<!-- insert picture -->
### mark waypoints
Run the waypoint.py for waypoint generation.
```
rosrun map_follower waypoint.py
```
This script allows the user to use rviz for selecting waypoints, visualize the path and save the waypoints in a CSV file.
### run the code
In a new terminal execute the run.py file
```
rosrun map_follower run.py
```

