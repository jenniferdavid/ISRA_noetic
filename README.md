# ISRA

This package has two new layers of layered costmaps - temperature and radiation layer developed by [Tom Bridgwater] and [Andy West].
It works on a Neobotix mp-700 robot.

First add the my_ground_plane to your /home/user/.gazebo/models folder

Download the other 4 folders to your catkin workspace and catkin_make it.
The cfg files will not generate .h files because the script might be in read-only mode.
Make into an executable file using:
```
sudo chmod +x filename
```

After catkin_make,
The simulation can be run by:
```
roslaunch neo_simulation simulation.launch
```
The above should launch the simulation with the radiation and temperature layers.
From here you can teleoperate, using the robot ROS teleop commands.
It will create a costmap of the arena. 
You can then launch the dynamic reconfigure and state updater by using the command 
```
roslaunch neo_simulation reconfigure.launch
```
[Tom Bridgwater]: https://github.com/Tom1693
[Andy West]: https://github.com/DrAndyWest 
