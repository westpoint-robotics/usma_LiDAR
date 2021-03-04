
#ifndef GAZEBO_ROS_VELODYNE_HH
#define GAZEBO_ROS_VELODYNE_HH

#include <string>
#include <queue>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/thread/locks.hpp>

#include <ros/ros.h>
#include <ros/advertise_options.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include "laser_geometry/laser_geometry.h"


#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/plugins/GpuRayPlugin.hh>

#include <sdf/sdf.hh>

#include <gazebo_plugins/PubQueue.h>

namespace gazebo
{
  class GazeboRosVelodyne : public GpuRayPlugin
  {
     ///  Constructor
    public: GazeboRosVelodyne();

     ///  Destructor
    public: ~GazeboRosVelodyne();

     ///  Load the plugin
     /// \param take in SDF root element
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);


    private:
    ///  Keep track of number of connctions
    int laser_connect_count_;
    void LaserConnect();
    void LaserDisconnect();
    void VelodyneConnect();
    void VelodyneDisconnect();


    /// Pointer to the model
    std::string world_name_;
    physics::WorldPtr world_;
    ///  The sensor
    sensors::GpuRaySensorPtr ray_sensor_;

    // The parent entity
    physics::EntityPtr spindle_entity_;
    physics::EntityPtr base_entity_;

    ///  pointer to ros node
    ros::NodeHandle* rosnode_;
    ros::Publisher pointcloud_pub_;
    PubQueue<sensor_msgs::PointCloud2>::Ptr pointcloud_pub_queue_;
    std::queue<math::Pose> base_pose_queue_; // a queue for the OnScan to push into and the assembler thread to pop out of
    std::queue<math::Pose> spindle_pose_queue_; // a queue for the OnScan to push into and the assembler thread to pop out of

    std::queue<sensor_msgs::LaserScan> laserscan_queue_; // a queue for the OnScan to push into and the assembler thread to pop out of

    laser_geometry::LaserProjection projector_; // reprojects into a pointcloud

    boost::condition_variable scan_in_queue_;
    boost::mutex scan_queue_mutex_;

    boost::thread assembler_thread_; // a thread to assemble the laserscans into a full 360 point cloud
    void assemblerThread();

    boost::thread spindle_thread_;
    void UpdateChild();

    ///  topic name
    std::string topic_name_;

    ///  frame transform name, should match link name
    std::string spin_frame_name_;

    std::string static_frame_name_;
   
    ///  tf prefix
    std::string tf_prefix_;

    ///  for setting ROS name space
    std::string robot_namespace_;

    // deferred load in case ros is blocking
    sdf::ElementPtr sdf;
    void LoadThread();
    boost::thread deferred_load_thread_;
    unsigned int seed;

    gazebo::transport::NodePtr gazebo_node_;
    gazebo::transport::SubscriberPtr laser_scan_sub_;
    void OnScan(ConstLaserScanStampedPtr &_msg);

    ///  prevents blocking
    PubMultiQueue pmq;

    void quatToEuler( double &qx, double &qy, double &qz, double &qw, double &ex, double &ey, double &ez);
    void laserscanToPointCloud();

  };
}
#endif
