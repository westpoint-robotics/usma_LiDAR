/*
 * Copyright 2013 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/*
   Desc: GazeboRosGpuLaser plugin for simulating ray sensors in Gazebo
   Author: Mihai Emanuel Dolha
   Date: 29 March 2012
 */

#include <algorithm>
#include <string>
#include <queue>
#include <assert.h>
#include <math.h>

#include <sdf/sdf.hh>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/sensors/GpuRaySensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/transport/transport.hh>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include "usma_velodyne/gazebo_ros_gpu_velodyne.h"
#include <gazebo_plugins/gazebo_ros_utils.h>

namespace gazebo
{
  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(GazeboRosVelodyne);


  ////////////////////////////////////////////////////////////////////////////////
  // Constructor
  GazeboRosVelodyne::GazeboRosVelodyne()
  {
    this->seed = 0;
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Destructor
  GazeboRosVelodyne::~GazeboRosVelodyne()
  {
    ROS_DEBUG_STREAM_NAMED("gpu_laser","Shutting down GPU Laser");
    this->rosnode_->shutdown();
    delete this->rosnode_;
    ROS_DEBUG_STREAM_NAMED("gpu_laser","Unloaded");
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Load the controller
  void GazeboRosVelodyne::Load(sensors::SensorPtr sensor, sdf::ElementPtr _sdf)
  {
    // load plugin
    GpuRayPlugin::Load(sensor, this->sdf);
    // Get the world name.
    std::string worldName = sensor->GetWorldName();
    this->world_ = physics::get_world( worldName );
    this->spindle_entity_ = this->world_->GetEntity( sensor->GetParentName() );
    this->base_entity_ = this->world_->GetEntity( this->spindle_entity_->GetParent()->GetName() );
    // save pointers
    this->sdf = _sdf;

    this->ray_sensor_ =
      boost::dynamic_pointer_cast<sensors::GpuRaySensor>(sensor);

    if (!this->ray_sensor_)
      gzthrow("GazeboRosVelodyne controller requires a Ray Sensor as its parent");

    this->robot_namespace_ =  GetRobotNamespace(sensor, _sdf, "Laser");

    if (!this->sdf->HasElement("spinFrameName"))
    {
      ROS_INFO("GazeboRosVelodyne plugin missing <frameName>, defaults to /world");
      this->spin_frame_name_ = "/world";
    }
    else
      this->spin_frame_name_ = this->sdf->Get<std::string>("spinFrameName");

    if (!this->sdf->HasElement("staticFrameName"))
    {
      ROS_INFO("GazeboRosVelodyne plugin missing <frameName>, defaults to /world");
      this->static_frame_name_ = "/world";
    }
    else
      this->static_frame_name_ = this->sdf->Get<std::string>("staticFrameName");

    if (!this->sdf->HasElement("topicName"))
    {
      ROS_INFO("GazeboRosVelodyne plugin missing <topicName>, defaults to /world");
      this->topic_name_ = "/world";
    }
    else
      this->topic_name_ = this->sdf->Get<std::string>("topicName");

    this->laser_connect_count_ = 0;


    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
          << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    ROS_INFO ( "Starting GazeboRosVelodyne Plugin (ns = %s)!", this->robot_namespace_.c_str() );
    // ros callback queue for processing subscription
    this->deferred_load_thread_ = boost::thread(
        boost::bind(&GazeboRosVelodyne::LoadThread, this));

    this->assembler_thread_ = boost::thread( &GazeboRosVelodyne::assemblerThread, this );

  }

  ////////////////////////////////////////////////////////////////////////////////
  // Load the controller
  void GazeboRosVelodyne::LoadThread()
  {
    ROS_WARN("loading");
    this->gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    this->gazebo_node_->Init(this->world_name_);

    this->pmq.startServiceThread();

    this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

    this->tf_prefix_ = tf::getPrefixParam(*this->rosnode_);
    if(this->tf_prefix_.empty()) {
      this->tf_prefix_ = this->robot_namespace_;
      boost::trim_right_if(this->tf_prefix_,boost::is_any_of("/"));
    }
    ROS_INFO("GPU Laser Plugin (ns = %s) <tf_prefix_>, set to \"%s\"",
        this->robot_namespace_.c_str(), this->tf_prefix_.c_str());

    // resolve tf prefix
    this->spin_frame_name_ = tf::resolve(this->tf_prefix_, this->spin_frame_name_);
    this->static_frame_name_ = tf::resolve(this->tf_prefix_, this->static_frame_name_);

    ROS_INFO( "[GazeboRosVelodyne] spin frame name::: %s", this->spin_frame_name_.c_str() );
    ROS_INFO( "[GazeboRosVelodyne] static frame name::: %s", this->static_frame_name_.c_str() );

    if (this->topic_name_ != "")
    {
      ros::AdvertiseOptions ao =
        ros::AdvertiseOptions::create<sensor_msgs::PointCloud2>(
            this->topic_name_, 1,
            boost::bind(&GazeboRosVelodyne::VelodyneConnect, this),
            boost::bind(&GazeboRosVelodyne::VelodyneDisconnect, this),
            ros::VoidPtr(), NULL);
      this->pointcloud_pub_ = this->rosnode_->advertise(ao);
      this->pointcloud_pub_queue_ = this->pmq.addPub<sensor_msgs::PointCloud2>();
    }

    // Initialize the controller

    // sensor generation off by default
    this->ray_sensor_->SetActive(false);
    //  this->ray_sensor_->SetParent("robot_description::base_link", 18);

    ROS_INFO_STREAM_NAMED("gpu_laser","LoadThread function completed");
  }

  void GazeboRosVelodyne::VelodyneConnect()
  {
    this->LaserConnect();
  }

  void GazeboRosVelodyne::VelodyneDisconnect()
  {
    this->LaserDisconnect();
  }

  ////////////////////////////////////////////////////////////////////////////////
  void GazeboRosVelodyne::LaserConnect()
  {
    ROS_WARN("[GazeboRosVelodyne]: LASER CONNECTED");
    this->laser_connect_count_++;
    if (this->laser_connect_count_ == 1)
      this->laser_scan_sub_ =
        this->gazebo_node_->Subscribe(this->ray_sensor_->GetTopic(),
            &GazeboRosVelodyne::OnScan, this);
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Decrement count
  void GazeboRosVelodyne::LaserDisconnect()
  {
    this->laser_connect_count_--;
    if (this->laser_connect_count_ == 0)
      this->laser_scan_sub_.reset();
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Convert new Gazebo message to ROS message and publish it
  void GazeboRosVelodyne::OnScan(ConstLaserScanStampedPtr &_msg)
  {
    // We got a new message from the Gazebo sensor.  Stuff a
    // corresponding ROS message and publish it.
    sensor_msgs::LaserScan laser_msg;
    laser_msg.header.stamp = ros::Time(_msg->time().sec(), _msg->time().nsec());
    laser_msg.header.frame_id = this->spin_frame_name_;
    laser_msg.angle_min = _msg->scan().angle_min();
    laser_msg.angle_max = _msg->scan().angle_max();
    laser_msg.angle_increment = _msg->scan().angle_step();
    laser_msg.time_increment = 0;  // instantaneous simulator scan
    laser_msg.scan_time = 0;  // not sure whether this is correct
    laser_msg.range_min = _msg->scan().range_min();
    laser_msg.range_max = _msg->scan().range_max();
    laser_msg.ranges.resize(_msg->scan().ranges_size());
    std::copy(_msg->scan().ranges().begin(),
        _msg->scan().ranges().end(),
        laser_msg.ranges.begin());
    laser_msg.intensities.resize(_msg->scan().intensities_size());
    std::copy(_msg->scan().intensities().begin(),
        _msg->scan().intensities().end(),
        laser_msg.intensities.begin());

    boost::lock_guard<boost::mutex> lock( this->scan_queue_mutex_ );
    this->laserscan_queue_.push(laser_msg);
    //  this->laserscan_pub_queue_->push(laser_msg, this->laserscan_pub_);

    this->spindle_pose_queue_.push( this->spindle_entity_->GetWorldPose() );//this->ray_sensor_->GetPose() + this->spindle_entity_->GetRelativePose() );
    this->base_pose_queue_.push( this->base_entity_->GetWorldPose() );
    this->scan_in_queue_.notify_one();
  }

  void GazeboRosVelodyne::laserscanToPointCloud()
  {
  }

  void GazeboRosVelodyne::quatToEuler(double &qx, double &qy, double &qz, double &qw, double &ex, double &ey, double &ez)
  {
    double sqw = qw*qw;
    double sqx = qx*qx;
    double sqy = qy*qy;
    double sqz = qz*qz;

    ey = asin(2.0 * (qw*qy - qx*qz));
    if (M_PI /2.0 - fabs(ey) > FLT_EPSILON) {
      ez = atan2(2.0 * (qx*qy + qw*qz),
          sqx - sqy - sqz + sqw);
      ex = atan2(2.0 * (qw*qx + qy*qz),
          sqw - sqx - sqy + sqz);
    } else {
      // compute heading from local 'down' vector
      ez = atan2(2*qy*qz - 2*qx*qw,
          2*qx*qz + 2*qy*qw);
      ex = 0.0;

      // If facing down, reverse yaw
      if (ey < 0)
        ez = M_PI - ez;
    }      
  }

  void GazeboRosVelodyne::assemblerThread()
  {
    ROS_WARN("[GazeboRosVelodyne]: ASSEMBLER THREAD RUNNING");
    sensor_msgs::PointCloud2 assembled_ros_cloud;

    pcl::PointCloud<pcl::PointXYZ> single_scan_cloud;
    pcl::PointCloud<pcl::PointXYZ> assembled_pcl_cloud;

    math::Pose frameStartWorldPose;

    double eulerx, eulery, eulerz, prev_eulerz, prev_angle_change;

    while (ros::ok()) {
      while ( !this->laserscan_queue_.empty() ) {
        // project the laser scan into a cloud then reorient the cloud

        unsigned int length = this->laserscan_queue_.front().ranges.size();
        double angle_min = this->laserscan_queue_.front().angle_min;
        double angle_increment = this->laserscan_queue_.front().angle_increment;

        boost::numeric::ublas::matrix<double> ranges(2, length);
        boost::numeric::ublas::matrix<double> unit_vectors(2,length);

        // Fill the ranges matrix and unit vectors
        for (unsigned int i = 0; i < length; i++)
        {
          ranges(0,i) = (double) this->laserscan_queue_.front().ranges[i];
          ranges(1,i) = (double) this->laserscan_queue_.front().ranges[i];
          unit_vectors(0,i) = cos(angle_min + (double) i * angle_increment);
          unit_vectors(1,i) = sin(angle_min + (double) i * angle_increment);
        }

        boost::numeric::ublas::matrix<double> single_scan_matrix = element_prod(ranges, unit_vectors);

        single_scan_cloud.clear();

        for ( unsigned int i = 0; i < length; i++ ) {
          single_scan_cloud.push_back( pcl::PointXYZ(single_scan_matrix(0,i), single_scan_matrix(1,i), 0.0) );
        }

        math::Pose gzPose( this->spindle_pose_queue_.front() );

        // quick conversion to Euler angles
        GazeboRosVelodyne::quatToEuler( gzPose.rot.x, gzPose.rot.y, gzPose.rot.z, gzPose.rot.w, eulerx, eulery, eulerz );

        pcl::transformPointCloud( single_scan_cloud, single_scan_cloud, pcl::getTransformation( gzPose.pos.x, gzPose.pos.y, gzPose.pos.z, eulerx, eulery, eulerz ) );

        assembled_pcl_cloud += single_scan_cloud;

        // check for wrap around on the scans
        if (  fabs( fabs( eulerz - prev_eulerz ) - fabs( prev_angle_change ) ) > M_PI )
        {
          gzPose = this->base_pose_queue_.front().GetInverse();

          double ex, ey, ez;

          GazeboRosVelodyne::quatToEuler( gzPose.rot.x, gzPose.rot.y, gzPose.rot.z, gzPose.rot.w, ex, ey, ez );

          pcl::transformPointCloud( assembled_pcl_cloud, assembled_pcl_cloud, pcl::getTransformation( gzPose.pos.x, gzPose.pos.y, gzPose.pos.z, ex, ey, ez ) );

          pcl::toROSMsg( assembled_pcl_cloud, assembled_ros_cloud );
          assembled_ros_cloud.header = this->laserscan_queue_.front().header;
          assembled_ros_cloud.header.frame_id = this->static_frame_name_;
          this->pointcloud_pub_queue_->push(assembled_ros_cloud, this->pointcloud_pub_);
          assembled_pcl_cloud.clear();
          frameStartWorldPose = this->spindle_pose_queue_.front();
        }
        else
        {
          prev_angle_change = eulerz - prev_eulerz;
        }

        prev_eulerz = eulerz;
        boost::lock_guard<boost::mutex> lock( this->scan_queue_mutex_ );
        this->spindle_pose_queue_.pop();
        this->base_pose_queue_.pop();
        this->laserscan_queue_.pop();

      }
      boost::unique_lock<boost::mutex> lock( this->scan_queue_mutex_ );
      this->scan_in_queue_.wait( lock );
    }
  }
}
