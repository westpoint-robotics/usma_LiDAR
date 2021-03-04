# usma_velodyne
## Velodyne Setup Guide

### Assumptions
This document assumes you are working with a computer running Ubuntu 18.04 with ROS melodic installed.  It is also assumed that you have a Velodyne LiDAR (e.g. VLP16 Puck).

### Overview
This document will take you through the following steps of setting up your Velodyne Puck.

1. Connecting Hardware
2. Install Dependencies and Packages
3. Network Setup
4. Verfiying Functionality, and Network Setup
5. ROS Pointcloud Visualization


### 1. Connecting Hardware
Chapter 4 of the [Velodyne User Manual](https://greenvalleyintl.com/wp-content/uploads/2019/02/Velodyne-LiDAR-VLP-16-User-Manual.pdf) guides you through hardware setup.  Instead of the power adapter that comes with the Velodyne, which is useful for benchwork, you will need regulated power that provides 9V - 18V.  Refer to the [datasheet](https://pdf.directindustry.com/pdf/velodynelidar/vlp-16-datasheets/182407-676097.html) for further details.

### 2. Install Dependencies and Packages
1. System space packages (substitute kinetic, or noetic for melodic if you are using Ubuntu 16.04, or 20.04 respectively)
   `sudo apt-get install ros-melodic-pointcloud-to-laserscan`

2. User space (catkin_ws) packages
   If you do not have a catkin workspace, you can follow the instructions in [ROS wiki tutorials](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) for how to create one.  Afterward, install the Velodyne packages as instructed below.  If you name your workspace something other than, "catkin_ws" (e.g. my_awesome_ws), just replace "catkin_ws" with the name you used.
   `cd ~/catkin_ws/src`
   `git clone https://github.com/ros-drivers/velodyne.git`
   `cd ~/catkin_ws` or `cd ..`
   `catkin_make`
   

### 3. Network Setup 
1. # In the project root directory the network settings are saved in a file called velodyneVLP16 and veldoyne32E.
2. `sudo chown root:root velodyneVLP16`
3. `sudo chown root:root velodyne32E`
4. `sudo chmod 600 velodyneVLP16`
5. `sudo chmod 600 velodyne32E`
6. `sudo cp velodyneVLP16 /etc/NetworkManager/system-connections/`
7. `sudo cp velodyne32E /etc/NetworkManager/system-connections/`
8. `sudo service network-manager restart`
9. # In the network-manager gui, connect to velodyneVLP16 or velodyne32E

### 3. Network Setup (Alternate)
1. Make sure to **plug directly into the PCI port on the computer that will be running Velodyne related code/ROS nodes.**
2. Create a **manual configuration in Network Manager** as depicted below to connect to the Velodyne.
    1. In the upper right of your screen, select the down arrow, and tools button.
    2. Select the Network section from the choices on the left of the tools window.
    3. Click the "+", or "ADD" button for the "Wired" section.
    4. On the "Identity" tab, change the Name the one below.  In the MAC Address field, set the address to the one corresponding to the "eno1" port (it may be called something different; wherever ifconfig tells you the Velodyne is plugged in).  Setting the MAC address will force connections be directly through the PCI Ethernet port, meaning if anyone connects to a USB Ethernet adapter, the Velodyne connection will not even show up.
       
       [INSERT PICTURE HERE??]
       

    5. On the "IPv4" tab, select "IPv4 Method" to be "manual", and fill in the "Addresses" section like this:
      
	IPV4
	Method:Manual
	Addresses
	Address:192.168.3.100
	Netmask:255.255.255.0
	Gateway 0.0.0.0

    6. Security should be off; leave IPv6 at default values.

### 4. Verfiying Functionality, and Network Setup

    `roslaunch velodyne_pointcloud  VLP16_points.launch`
or

    `roslaunch velodyne_pointcloud  32e_points.launch`

To see if the velodyne is working you can run: `sudo tcpdump -i eth0` (This may be eno1, or something else instead of eth0; use `ifconfig` to determine what port your LiDAR is plugged into.)  But tcpdump doesn't guarantee that ROS can see the data. If the route is not set up properly you will see data streaming in tcpdump but ROS will not be able to access to it.

In addition to tcpdump, you can check to see if ROS is recieving the Pointcloud: 

`rostopic echo /velodyne_points`


### 5. ROS Pointcloud Visualization
1. `roscd usma_velodyne`
2. `rviz -d launch/velodyne_bare.rviz`


### TODO:
- [ ] Add images


