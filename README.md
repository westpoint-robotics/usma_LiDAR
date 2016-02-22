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




