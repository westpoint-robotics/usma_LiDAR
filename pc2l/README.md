#pc2l Pointcloud to Laserscan Package

This package is to be used in conjunction with the pointcloud_to_laserscan (`sudo apt-get install ros-melodic-pointcloud-to-laserscan`) package to convert pointcloud
ROS messages to laser scan messages as required by Google Cartographer.  There are separate launch files for each supported sensor.

1. pc2l_ouster for the OS1 series Ousters.
2. pc2l_velodyne for the 32 and 16 laser Velodyne Pucks.
3. pc2l_realsense for the Intel RealSense D435i depth camera.

### TODO:
- [ ] Parameterize the launch file to accept a specification of which sensor, and set relevant message names accordingly.
