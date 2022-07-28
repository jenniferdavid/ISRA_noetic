# ISRA - noetic

This is the modified code for ISRA - Internal State-based Risk Assessment for navigation of mobile robots. It has two new layers for layered costmaps - temperature and radiation layers. The code works with ROS Noetic. The original code is written by [Tom Bridgwater] and [Andy West] which can be found in [here]. It works on a Neobotix mp-700 robot with ROS Navigation stack.

First add the "my_ground_plane" to your /home/user/.gazebo/models folder.

Download the other 5 folders to your catkin workspace and catkin_make it.
In case, the cfg files did not generate .h files then the scripts might be in read-only mode.
So, make into an executable file using:
```
sudo chmod +x filename
```

After catkin_make,
The simulation can be run by:
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
