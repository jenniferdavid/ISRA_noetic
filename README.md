# ISRA - noetic

This is the modified code for ISRA - Internal State-based Risk Assessment for navigation of mobile robots. It has two new layers for layered costmaps - temperature and radiation layers. The code works with ROS Noetic. The original code is written by [Tom Bridgwater] and [Andy West] which can be found in [here]. It works on a Neobotix mp-700 robot with ROS Navigation stack.

First add the "my_ground_plane" to your /home/user/.gazebo/models folder.

Download the other 5 folders to your catkin workspace and catkin_make it.
In case, the catkin_make is not built due to missing header files. It is due to the cfg files in the folders /temperature_layer_isra/cfg/ or /radiation_layer_isra/cfg/  did not generate the required .h files because the python scripts in the folders /temperature_layer_isra/scripts/ and /radiation_layer_isra/scripts/ are in read-only mode.

So, go to the folders /temperature_layer_isra/scripts/ and /radiation_layer_isra/scripts/ and make the .py files into executable ones using:
```
sudo chmod +x filename
```

Now the catkin_make should be successful.
The simulation can now be run by:
```
roslaunch neo_simulation simulation.launch
```
The above should launch the simulation with the radiation and temperature layers.
From here you can teleoperate, using the robot ROS teleop commands or just autonomous navigation using 2D goal.
It will create a costmap of the arena. 
You can then launch the dynamic reconfigure and state updater by using the command 
```
roslaunch neo_simulation reconfigure.launch
```
[Tom Bridgwater]: https://github.com/Tom1693
[Andy West]: https://github.com/DrAndyWest 
[here]: https://github.com/jenniferdavid/ISRA
