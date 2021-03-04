# usma_LiDAR

This repository contains all LiDAR packages (usma_ouster and usma_velodyne as of 4 March 2021).  These have been configured with urdfs launch files, and 
README instructions for visualizing relevant LiDARs using vendor applications (e.g. Ouster Studio), and or ROS.  Also included is the package, "pc2l", which,
in conjunction with pointcloud_to_laserscan (see package README's for details) will let the user map LiDAR pointcloud data to a 2D laserscan published as a
message in accordance with the requirements of Google Cartographer.
