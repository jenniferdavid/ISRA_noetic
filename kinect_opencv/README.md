# mpo_700_kinect
This contains the nodes to run alongside the neobotix simulation that subscribe to the kinect sensors and provide temperature and radiation data.

To run a point cloud publish that publishes a PointCloud2 topic for each of the four kinect sensors, which represent temperature sensors, run the  	sensorNodeLauncher.launch file with roslaunch. This must be run in tandem with the Neobotix simulation package.

This will publish the points in the map that correspond to red pixels.