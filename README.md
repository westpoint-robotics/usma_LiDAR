# usma_velodyne

#### 1. Install the velodyne laser packages from github.
1. `cd ~/catkin_ws/src`
2. `git clone https://github.com/ros-drivers/velodyne.git`
3. `cd ~/catkin_make`
4. `catkin_make`

#### 2. Make a network connection for the velodyne VLP16.
1. # In the project root directory the network settings are saved in a file called velodyneVLP16 and veldoyne32E.
2. `sudo chown root:root velodyneVLP16`
3. `sudo chown root:root velodyne32E`
4. `sudo chmod 600 velodyneVLP16`
5. `sudo chmod 600 velodyneV32E`
6. `sudo cp velodyneVLP16 /etc/NetworkManager/system-connections/`
7. `sudo cp velodyne32E /etc/NetworkManager/system-connections/`
8. `sudo service network-manager restart`
9. # In the network-manager gui, connect to velodyneVLP16 or velodyne32E

#### 3. Test connection by running 

    roslaunch velodyne_pointcloud  VLP16_points.launch
or

    roslaunch velodyne_pointcloud  32e_points.launch

To see if the velodyne is working you can run: `sudo tcpdump -i eth0` But tcpdump doesn't garuntee that ROS can see the data. If the route is not set up properly you will see data streaming in tcpdump but ROS will not be able to access to it.

#### Alternative method with manual settings below here.
This needs updated !!!

How to configure Velodyne32 to give
Edit Ethernet connections
IPV4
Method:Manual
Addresses
Address:192.168.3.100
Netmask:255.255.255.0
Gateway 0.0.0.0

Routes
Address 192.168.17.215
netmask 255.255.255.255

SAVE

In terminal,
sudo route add 192.168.17.215 dev eth1 (changes depends on the port you use)


USE 
sudo tcpdump -i eth1
this sees whether the velodyne is givin data on that line should stream lots of data

Run the roslaunch file for the velodyne

Check to see if ROS is recieving the Pointcloud 

rostopic echo /velodyne_points




