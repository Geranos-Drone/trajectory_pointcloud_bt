#ifndef TRAJECTORY_PC_H
#define TRAJECTORY_PC_H

#include <ros/ros.h>
#include <chrono>
#include <sensor_msgs/PointCloud2.h>
#include <omav_local_planner/ExecuteTrajectory.h>

// Transform listener
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include "tf/transform_datatypes.h"

//Service
#include <std_srvs/Empty.h>
#include <cstdlib>
#include <ros/service_traits.h>
#include "trajectory_pointcloud_bt/NewViewPoint.h"
#include "bachelor_thesis/PoleFound.h"

//msgs
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <fstream>

//eigen
#include <Eigen/Eigen>
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/emitter.h>


class TRAJECTORY_PC
{

public:

  //functions
  void Callback(const nav_msgs::OdometryConstPtr& odometry_msg);
  bool getTrajectory_height(const Eigen::Vector3d &current_position, const Eigen::Vector3d &current_rotation, const Eigen::Vector3d &end_position);
  bool getTrajectory_pole(const Eigen::Vector3d &current_position_eigen, const Eigen::Vector3d &pole_position_eigen, const Eigen::Vector3d &current_rotation_eigen);
  bool getTrajectory_yaw(const Eigen::Vector3d &current_position, const Eigen::Vector3d &current_rotation, const Eigen::Vector3d &end_rotation);
  bool Rec_Fus_PC(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  bool Service(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  void tf_transformer_();

  //Parameters
  double length_pole_;
  double yaw_end_;
  Eigen::Vector3d pole_com_;
  Eigen::Vector3d position_end_;
  int durchlaeufe_;
  int durchlaeufe2_;
  int response_pole_found;

  //Services and ros subscribers
  Eigen::Affine3d T_B_world_1;
  geometry_msgs::Pose pose_;
  ros::ServiceClient execute_traj;
  ros::ServiceClient record_pointcloud;
  ros::ServiceServer go_to_detected_pole;
  ros::ServiceServer pointcloud_trajectory;

  //tf
  tf::TransformListener tf_listener_;

};

#endif