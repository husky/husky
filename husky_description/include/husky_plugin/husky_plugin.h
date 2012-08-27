#ifndef GAZEBO_ROS_CREATE_H
#define GAZEBO_ROS_CREATE_H

#include "physics/physics.h"
#include "physics/PhysicsTypes.hh"
#include "sensors/SensorTypes.hh"
#include "transport/TransportTypes.hh"
#include "common/Time.hh"
#include "common/Plugin.hh"
#include "common/Events.hh"

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/PoseWithCovariance.h>

#include <tf/transform_broadcaster.h>
#include <ros/ros.h>


/*
 * Desc: Gazebo 1.x plugin for a Clearpath Robotics Husky A200
 * Adapted from the TurtleBot plugin
 * Author: Ryan Gariepy
 */ 

namespace gazebo
{
  class HuskyPlugin : public ModelPlugin
  {
    public: 
      HuskyPlugin();
      virtual ~HuskyPlugin();
          
      virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf );

      virtual void UpdateChild();
  
    private:

      void OnContact(const std::string &name, const physics::Contact &contact);
      void OnCmdVel( const geometry_msgs::TwistConstPtr &msg);


      /// Parameters
      std::string node_namespace_;
      std::string bl_joint_name_;
      std::string br_joint_name_;
      std::string fl_joint_name_;
      std::string fr_joint_name_;
      std::string base_geom_name_;

      /// Separation between the wheels
      float wheel_sep_;

      /// Diameter of the wheels
      float wheel_diam_;

      ///Torque applied to the wheels
      float torque_;

      ros::NodeHandle *rosnode_;
  
      ros::Publisher sensor_state_pub_;
      ros::Publisher odom_pub_;
      ros::Publisher joint_state_pub_;
  
      ros::Subscriber cmd_vel_sub_;

      physics::WorldPtr world_;
      physics::ModelPtr model_;
      sensors::SensorPtr parent_sensor_;

      /// Speeds of the wheels
      float *wheel_speed_;

      // Simulation time of the last update
      common::Time prev_update_time_;
      common::Time last_cmd_vel_time_;

      float odom_pose_[3];
      float odom_vel_[3];

      bool set_joints_[4];
      physics::JointPtr joints_[4];
      physics::CollisionPtr base_geom_;

      tf::TransformBroadcaster transform_broadcaster_;
      sensor_msgs::JointState js_;

      void spin();
      boost::thread *spinner_thread_;

      event::ConnectionPtr contact_event_;

      // Pointer to the update event connection
      event::ConnectionPtr updateConnection;
  };
}
#endif
