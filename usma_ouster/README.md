# usma_ouster
## Ouster OS1 Setup Guide (Gen1)

### Assumptions
This document assumes you are working with a computer running Ubuntu 18.04 with ROS melodic installed.  It is also assumed that you have an Ouster LiDAR, of the Generation 1 series (e.g. OS1-64).

### Overview
Following the [hardware and software user guides](https://ouster.com/downloads/) to get started with an Ouster LiDAR can be useful, but has limitations due to numerous combinations of available hardware and firmware versions.  Their download page lets you select each of these (e.g. Gen1, v1.13), but the PDFs you download may still not provide the appropriate documentation for certain aspects of the setup.  The best way to get appropriate documentation is to enter the SN of the Ouster you have into a web browser as described below.  This is also a good way to ensure your sensor is working correctly.

Before taking that step, however, you will need to connect your Ouster to a computer (e.g. robot companion, or Linux development computer), and set up some network settings on that computer.  This document will take you through the following steps of setting up your Ouster.

1. Connecting Hardware
2. Network Setup
3. Verifying Functionality, Network Settings, and Documentation
4. Ouster Studio (GUI Visualization)
5. ROS Pointcloud Demo
6. ROS Pointcloud Visualization
7. PTP Server Setup for Clock Synchronization

### 1. Connecting Hardware
Each Ouster comes with an Interface Box, Cat 5 cable, and 24V 1.5A power adapter.  The [hardware guide](https://data.ouster.io/downloads/hardware-user-manual/hardware-user-manual-gen1-os1.pdf) should be consulted for a  more complete understanding of specifications and proper setup, particularly with respect to the pigtail connector, but the images below show a system completely set up for benchtop testing (robot power should be used in lieu of the wall adapter in experiments and missions involving locomotion; power tolerances can be found in the [datasheet](https://data.ouster.io/downloads/datasheets/datasheet-gen1-v1p13-os1.pdf).

<p align="center">
  <img width="302" height="446" src="https://github.com/doubleonick/Shazam/blob/master/Ouster_Full_Setup_labeled.png">
</p>

**a) Ouster LiDAR, b) Pigtail, c) CAT 5 cable, d) Interface Box, e) 24V 1.5A power adapter**


Once your Ouster is set up as shown, you should hear the laser array spin up, and continue its steady whir; once you get this, you may proceed to set up your network.  NOTE: While the lasers should be safe for human eyes (e.g. if you are being scanned in part of a scene), do not stare into the array during operation.

### 2. Network Setup
1. Do **not** use USB ethernet adapters for the Ouster, the ones we have do not support PTP for time synchronization.  Make sure to **plug directly into the PCI port on the computer that will be running Ouster related code/ROS nodes.**
2.  By default the Ouster has an ip address of: 169.254.156.128. If reset the Ouster this is the IP address it has.
3.  The [software guide](https://data.ouster.io/downloads/software-user-manual/software-user-manual-v1p13.pdf) mentions using a 10.5.5.0/24 subnet. This only works if you successfully follow their instructions to run a DHCP server on the computer. This is time consuming and unnecessary. Instead, create a **manual configuration in Network Manager** as depicted below to connect to the Ouster.
    1. In the upper right of your screen, select the down arrow, and tools button.
    2. Select the Network section from the choices on the left of the tools window.
    3. Click the "+", or "ADD" button for the "Wired" section.
    4. On the "Identity" tab, change the Name the one below.  In the MAC Address field, set the address to the one corresponding to the "eno1" port (it may be called something different; wherever ifconfig tells you the Ouster is plugged in).  Setting the MAC address will force connections be directly through the PCI Ethernet port, meaning if anyone connects to a USB Ethernet adapter, the Ouster connection will not even show up.
       
       ![Ouster Identity](https://github.com/doubleonick/Shazam/blob/master/OUSTER_OS1-64_Identity.png)  
       

    5. On the "IPv4" tab, select "IPv4 Method" to be "manual", and fill in the "Addresses" section like this:
      ![Ouster Identity](https://github.com/doubleonick/Shazam/blob/master/OUSTER_OS1-64_IPv4.png)  
      You may be used to setting the Netmask differently, but using 255.255.0.0, leaves the last two octets free to be changed by the Ouster.  
    6. Security should be off; leave IPv6 at default values.
    


### 3. Verfiying Functionality, Network Settings, and Documentation

This step requires you to determine your Ouster's Serial Number (SN), which will be on the top of the sensor, and be of the form, "99xxxxxxxxxx":

![Ouster SN](https://github.com/doubleonick/Shazam/blob/master/Ouster_SN.png)

When you have the Ouster powered and connected to the computer as described above, you can open a web browser, and navigate to:

[http://os1-991947000828.local](http://os1-991947000828.local)

or 

[http://os1-992007000208.local](http://os1-992007000208.local)

depending on SN of the Ouster.  If your SN is different from this, make the substitution.  If you are successful, and your sensor is working properly, you will see a page like this:

![Ouster Dashboard](https://github.com/doubleonick/Shazam/blob/master/Ouster_Browser_Dashboard.png)

From here, you can see that it is possible to update the firmware as needed (though not recommended without guidance by an instructor), get the **appropriate documentation**, and more.

To find network settings visit:
[http://os1-991947000828.local/api/v1/system/network](http://os1-991947000828.local/api/v1/system/network)

To examine other settings look at the web api in the Ouster documentation.

### 4. Ouster Studio (GUI Visualization)

Downlad [Ouster Studio](https://www.paraview.org/ousterstudio/) from ParaView, and make sure to get the lattest version (3.6.0-777 as the writing of this document).  For Linux, you can download the .tar.gz file, and Extract it to somewhere convenient.  Once extracted, go to the /bin folder within the Ouster Studio directory, and scroll down to the bottom to find the "OusterStudio" application.  Launch the application and select the black Ouster icon at the top left.  This will let you configure the sensor.  

![Ouster Configure](https://github.com/doubleonick/Shazam/blob/master/Ouster_Studio_Configure.png)

Do a new search for addresses, and when one shows up, click the dropdown, and select the IPv6 address.  

![Ouster Sensor](https://github.com/doubleonick/Shazam/blob/master/Ouster_Studio_Sensor.png)

Then select the "Configure Sensor" button, before clicking, "OK".  After you click, "OK", close out of the window that pops up, and after a few seconds, you should start to see the pointcloud being displayed.  

![Ouster Studio Pointcloud](https://github.com/doubleonick/Shazam/blob/master/Ouster_Studio_Pointcloud.png)

You can use a mouse(pad) to zoom in and out and rotate the POV to better inspect the image.

### 5. ROS Pointcloud Demo

ROS Code to use:

Do not use the code from apt-get, it is outdated and does not work well with firmware version 1.13 and later.

Use the code found at:
`git clone https://github.com/ouster-lidar/ouster_example.git`
clone this into your local catkin_ws and run 
`catkin_make`
Here is the cli (command-line interface) to run this:
`roslaunch ouster_ros ouster.launch sensor_hostname:=os1-992007000208.local udp_dest:=169.254.156.200 viz:=false timestamp_mode:=TIME_FROM_PTP_1588`

`roslaunch ouster_ros ouster.launch sensor_hostname:=os1-991947000828.local udp_dest:=169.254.156.200 viz:=false timestamp_mode:=TIME_FROM_PTP_1588`

The Ouster IP address may change with out warining. So for hostname always use:os1-99xxxxxxxxxx.local and not the IP address.

NOTE: It may take a few minutes for the Ouster to initialize

### 6. ROS Pointcloud Visualization

To vizualize in rviz open a terminal and type 
`rviz`
![Ouster Rviz default](https://github.com/doubleonick/Shazam/blob/master/Ouster_rviz_default.png)

Once rviz starts choose "Add"
Choose "By Topic"
Choose "Pointcloud2" as the topic.

![Ouster Rviz AddByTopic](https://github.com/doubleonick/Shazam/blob/master/Ouster_rviz_addbytopic.png)

This will often generate a frame error in rviz depending on your default rviz file.
To fix this change the reference frame:
Under Displays window, choose "Global Frames", "Fixed Frame" and scroll down until you find the Ouster frame.

After this you should see a visualization of the point cloud.

![Ouster ROS pointcloud](https://github.com/doubleonick/Shazam/blob/master/Ouster_ROS_Rviz_Pointcloud.png)


### 7. PTP Server Setup for Clock Synchronization

PTP (Precision Time Protocol) is a system used to sychroize clocks in a computer network.  The Ouster has its own clock, and that needs to be synchronized with the clock on the computer running any Ouster ROS nodes for the system to function correctly.

To setup the PTP server to synch the clocks between the Ouster and computer follow the instructions in the Ouster manual appendix part 1.4.  (See the Documentation tab on the browser page visited in the "Verfiying Functionality, Network Settings, and Documentation" section of this document.)  **Do not do section 1.4.7**, this refers to synching to another master clock and we do not need or want that.

### TODO:
- [ ] Add parameters to launch file (ouster.launch), instead of passing arguments in cli.

