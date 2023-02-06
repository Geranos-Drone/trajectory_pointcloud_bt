#include <trajectory_pointcloud_bt/record_pointcloud_trajectory.h>
#include <math.h> 
using namespace std::chrono;

void TRAJECTORY_PC::Callback (const nav_msgs::OdometryConstPtr& odometry_msg) {                             
  ROS_INFO_ONCE("Record_PC received first odometry!");
  pose_ = odometry_msg->pose.pose;
  //current_yaw_W_B_ = mav_msgs::yawFromQuaternion(current_odometry_.orientation_W_B);
}

void TRAJECTORY_PC::tf_transformer_() {
  tf::StampedTransform transform_2;

  try {
    tf_listener_.waitForTransform("world", "geranos/base_link", ros::Time(0), ros::Duration(5.0));  
    tf_listener_.lookupTransform("world", "geranos/base_link", ros::Time(0), transform_2);      
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
  }
  tf::transformTFToEigen(transform_2, T_B_world_1);
}

bool TRAJECTORY_PC::Service2(trajectory_pointcloud_bt::NewViewPoint::Request& request, trajectory_pointcloud_bt::NewViewPoint::Response& response) {
  std::cerr << "Coordinates of pole are: " << request.x << " " << request.y << " " << request.z << " " << request.yaw << std::endl;
  //trajectory if camera view has to be changed

  Eigen::Vector3d current_position;
  tf_transformer_();
  current_position = T_B_world_1.translation();
  Eigen::Vector3d current_rotation;
  Eigen::Matrix3d rotational_matrix = T_B_world_1.rotation();
  double yaw = atan2(rotational_matrix(1,0),rotational_matrix(0,0));
  current_rotation << yaw,0,0;

  Eigen::Vector3d end_position; 
  Eigen::Vector3d end_rotation;
  end_position << request.x, request.y, request.z;
  end_rotation << 0, 0, request.yaw;

  
  if(getTrajectory3(current_position,end_position,current_rotation,end_rotation)) {
    //trajectory durchführen
    omav_local_planner::ExecuteTrajectory srv;
    srv.request.waypoint_filename = "/home/henriette/catkinws/src/mav_ui/omav_local_planner/resource/go_to_detected_pole.yaml";
    if (!execute_traj.call(srv))
      ROS_ERROR_STREAM("Was not able to call execute_trajectory from go_to_detected_pole service!");
  } 

  return true;
}


bool TRAJECTORY_PC::getTrajectory3(const Eigen::Vector3d &current_position_eigen, const Eigen::Vector3d &end_position_eigen, const Eigen::Vector3d &current_rotation_eigen, const Eigen::Vector3d &end_rotation_eigen) {

  double time1 = 0.75;
  double time2 = 3.3;
  std::vector<double> current_position(&current_position_eigen[0], current_position_eigen.data()+current_position_eigen.cols()*current_position_eigen.rows());
  std::vector<double> end_position(&end_position_eigen[0], end_position_eigen.data()+end_position_eigen.cols()*end_position_eigen.rows());
  std::vector<double> current_rotation(&current_rotation_eigen[0], current_rotation_eigen.data()+current_rotation_eigen.cols()*current_rotation_eigen.rows());
  std::vector<double> end_rotation(&end_rotation_eigen[0], end_rotation_eigen.data()+end_rotation_eigen.cols()*end_rotation_eigen.rows());
  std::vector<double> force = {0.0, 0.0, 0.0};

  YAML::Emitter emitter;

  emitter << YAML::BeginMap;
  emitter << YAML::Key << "order_rpy";
  emitter << YAML::Value << 0; // 0 for yaw-pitch-roll, 1 for roll-pitch-yaw
  emitter << YAML::Key << "forces";
  emitter << YAML::Value << true;
  emitter << YAML::Key << "torques";
  emitter << YAML::Value << false;
  emitter << YAML::Key << "times";
  emitter << YAML::Value << true;
  emitter << YAML::Key << "velocity_constraints";
  emitter << YAML::Value << true;
  emitter << YAML::Key << "constant_velocity";
  emitter << YAML::Value << false;
  emitter << YAML::Key << "points";

  YAML::Node point_list = YAML::Node(YAML::NodeType::Sequence);

  YAML::Node yaml_point = YAML::Node(YAML::NodeType::Map);
  yaml_point["pos"] = current_position;
  yaml_point["att"] = current_rotation;
  yaml_point["force"] = force;
  yaml_point["stop"] = true;
  yaml_point["time"] = time1;   

  YAML::Node yaml_point0 = YAML::Node(YAML::NodeType::Map);
  yaml_point0["pos"] = end_position;
  yaml_point0["att"] = end_rotation;
  yaml_point0["force"] = force;
  yaml_point0["stop"] = true;
  yaml_point0["time"] = time1;    

  point_list.push_back(yaml_point);
  point_list.push_back(yaml_point0);  
  
  emitter << YAML::Value << point_list;
  emitter << YAML::EndMap;

  return true;
/*  ROS_INFO_STREAM("writeYamlFile");
  std::string filename = "/home/henriette/catkinws/src/mav_ui/omav_local_planner/resource/go_to_detected_pole.yaml";
  try
  {
    std::ofstream fout;
    fout.open(filename.c_str());
    fout << emitter.c_str();
    fout.close();
    return true;
  }
  catch (...)
  {
    ROS_ERROR_STREAM("FAILED to write Yaml File!");
    return false;
  } */
}


// Main ROS method
int main(int argc, char **argv) {
    
  // Initialize the node and set the name
  ros::init(argc, argv, "record_pointcloud");
  
  TRAJECTORY_PC trajec_pc;
  ros::NodeHandle nh;
  // Create the service and advertise it to the ROS computational network
  ros::Subscriber sub_2 = nh.subscribe("/odometry", 1, &TRAJECTORY_PC::Callback , &trajec_pc);  
  trajec_pc.new_view_point = nh.advertiseService("/trajectory_pointcloud_bt/new_view_point", &TRAJECTORY_PC::Service2, &trajec_pc);

  // Keep processing information over and over again
  ros::spin();
 
  // Program completed successfully
  return 0;
}