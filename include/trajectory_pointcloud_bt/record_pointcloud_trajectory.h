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
#include "trajectory_pointcloud_bt/GoToPole.h"
#include "trajectory_pointcloud_bt/NewViewPoint.h"
#include "bachelor_thesis/PoleFound.h"

//msgs
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
//#include <mav_msgs/conversions.h>
//#include <mav_msgs/default_topics.h>
//#include <mav_msgs/common.h>
//#include <mav_msgs/eigen_mav_msgs.h>
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
  //void Callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_pcd);
  void Callback(const nav_msgs::OdometryConstPtr& odometry_msg);
  bool Rec_Fus_PC(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  bool Service2(trajectory_pointcloud_bt::NewViewPoint::Request& request, trajectory_pointcloud_bt::NewViewPoint::Response& response);
  bool getTrajectory_height(const Eigen::Vector3d &current_position, const Eigen::Vector3d &current_rotation, const Eigen::Vector3d &end_position);
  bool getTrajectory_yaw(const Eigen::Vector3d &current_position, const Eigen::Vector3d &current_rotation, const Eigen::Vector3d &end_rotation);
  bool getTrajectory_pole(const Eigen::Vector3d &current_position_eigen, const Eigen::Vector3d &pole_position_eigen, const Eigen::Vector3d &current_rotation_eigen);
  bool getTrajectory3(const Eigen::Vector3d &current_position_eigen, const Eigen::Vector3d &end_position_eigen, const Eigen::Vector3d &current_rotation_eigen, const Eigen::Vector3d &end_rotation_eigen);
  bool Trajectory_and_Recording(double &angle);
  void tf_transformer_();
  bool Service(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  int durchlaeufe_;
  int durchlaeufe2_;
  int response_pole_found;

  //Services and ros subscribers
  geometry_msgs::Pose pose_;
  ros::ServiceClient record_pointcloud;
  ros::ServiceServer pointcloud_trajectory;
  ros::ServiceServer go_to_detected_pole;
  ros::ServiceServer new_view_point;
  ros::ServiceClient execute_traj;
  Eigen::Affine3d T_B_world_1;

  Eigen::Vector3d position_end_;
  Eigen::Vector3d pole_com_;
  double yaw_end_;
  double length_pole_;


  //odometry
  //mav_msgs::EigenOdometry current_odometry_;

  //tf
  ros::NodeHandle nh_;
  tf::TransformListener tf_listener_;
  Eigen::Affine3d T_B_color_;
  //Eigen::Affine3d T_B_body_world_;


};

#endif