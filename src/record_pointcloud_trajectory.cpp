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


bool TRAJECTORY_PC::Rec_Fus_PC(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  auto start = high_resolution_clock::now();

  durchlaeufe_= 0;
  durchlaeufe2_=0;
  double angle = 45;

  Eigen::Vector3d current_position;
  Eigen::Vector3d current_rotation;
  Eigen::Vector3d end_position;
  Eigen::Vector3d end_rotation;
  double yaw;

  while (angle*durchlaeufe2_ < 360 || response_pole_found == 1) //durchlaeufe2_ < 1) //
  {
    //Trajectory
    //Current yaw + 45deg (360->0)
    if(durchlaeufe2_ == 0){
      tf_transformer_();      
      current_position = T_B_world_1.translation();
      Eigen::Matrix3d rotational_matrix = T_B_world_1.rotation();
      yaw = atan2(rotational_matrix(1,0),rotational_matrix(0,0));
      current_rotation << yaw,0,0;
      end_position << current_position[0], current_position[1], 1; //take-off to 1m
      if(getTrajectory_height(current_position,current_rotation,end_position)) {
        //trajectory durchführen
        omav_local_planner::ExecuteTrajectory srv;
        srv.request.waypoint_filename = "/home/henriette/catkinws/src/mav_ui/omav_local_planner/resource/pointcloud_trajectory.yaml";
        if (!execute_traj.call(srv))
          ROS_ERROR_STREAM("Was not able to call execute_trajectory from record_pointcloud_trajectory_node service!");
      }
      ros::Duration(3).sleep(); // sleep while trajectory is taking place
    }
    else if (response_pole_found == 0) { //no pole
      tf_transformer_();      
      current_position = T_B_world_1.translation();
      Eigen::Matrix3d rotational_matrix = T_B_world_1.rotation();
      yaw = atan2(rotational_matrix(1,0),rotational_matrix(0,0));
      current_rotation << yaw,0,0;
      end_rotation << yaw_end_,0,0;
      if(getTrajectory_yaw(current_position,current_rotation,end_rotation)) {
        //trajectory durchführen
        omav_local_planner::ExecuteTrajectory srv;
        srv.request.waypoint_filename = "/home/henriette/catkinws/src/mav_ui/omav_local_planner/resource/pointcloud_trajectory.yaml";
        if (!execute_traj.call(srv))
          ROS_ERROR_STREAM("Was not able to call execute_trajectory from record_pointcloud_trajectory_node service!");
      }
      ros::Duration(3).sleep(); // sleep while trajectory is taking place
    }
    else if (response_pole_found == 1) { //pole found
      pole_com_ = position_end_;
      length_pole_ = yaw_end_;
      response_pole_found = 1;
      break;
    }
    else if (response_pole_found == 2) { //bottom of pole not in view
      tf_transformer_();      
      current_position = T_B_world_1.translation();
      Eigen::Matrix3d rotational_matrix = T_B_world_1.rotation();
      yaw = atan2(rotational_matrix(1,0),rotational_matrix(0,0));
      current_rotation << yaw,0,0;
      end_position = position_end_;
      if(getTrajectory_height(current_position,current_rotation,end_position)) {
        //trajectory durchführen
        omav_local_planner::ExecuteTrajectory srv;
        srv.request.waypoint_filename = "/home/henriette/catkinws/src/mav_ui/omav_local_planner/resource/pointcloud_trajectory.yaml";
        if (!execute_traj.call(srv))
          ROS_ERROR_STREAM("Was not able to call execute_trajectory from record_pointcloud_trajectory_node service!");
      }
      ros::Duration(2.5).sleep(); // sleep while trajectory is taking place
    }
    else if (response_pole_found == 3) { //top of pole not in view
      tf_transformer_();      
      current_position = T_B_world_1.translation();
      Eigen::Matrix3d rotational_matrix = T_B_world_1.rotation();
      yaw = atan2(rotational_matrix(1,0),rotational_matrix(0,0));
      current_rotation << yaw,0,0;
      end_position = position_end_;
      if(getTrajectory_height(current_position,current_rotation,end_position)) {
        //trajectory durchführen
        omav_local_planner::ExecuteTrajectory srv;
        srv.request.waypoint_filename = "/home/henriette/catkinws/src/mav_ui/omav_local_planner/resource/pointcloud_trajectory.yaml";
        if (!execute_traj.call(srv))
          ROS_ERROR_STREAM("Was not able to call execute_trajectory from record_pointcloud_trajectory_node service!");
      }
      ros::Duration(5).sleep(); // sleep while trajectory is taking place
    }
    else if (response_pole_found == 4) { //pole in left edge
      tf_transformer_();      
      current_position = T_B_world_1.translation();
      Eigen::Matrix3d rotational_matrix = T_B_world_1.rotation();
      yaw = atan2(rotational_matrix(1,0),rotational_matrix(0,0));
      current_rotation << yaw,0,0;
      end_rotation << yaw_end_,0,0;
      if(getTrajectory_yaw(current_position,current_rotation,end_rotation)) {
        //trajectory durchführen
        omav_local_planner::ExecuteTrajectory srv;
        srv.request.waypoint_filename = "/home/henriette/catkinws/src/mav_ui/omav_local_planner/resource/pointcloud_trajectory.yaml";
        if (!execute_traj.call(srv))
          ROS_ERROR_STREAM("Was not able to call execute_trajectory from record_pointcloud_trajectory_node service!");
      }
      ros::Duration(5).sleep(); // sleep while trajectory is taking place
    }
    else if (response_pole_found == 5) { //pole in right edge
      tf_transformer_();      
      current_position = T_B_world_1.translation();
      Eigen::Matrix3d rotational_matrix = T_B_world_1.rotation();
      yaw = atan2(rotational_matrix(1,0),rotational_matrix(0,0));
      current_rotation << yaw,0,0;
      end_rotation << yaw_end_,0,0;
      if(getTrajectory_yaw(current_position,current_rotation,end_rotation)) {
        //trajectory durchführen
        omav_local_planner::ExecuteTrajectory srv;
        srv.request.waypoint_filename = "/home/henriette/catkinws/src/mav_ui/omav_local_planner/resource/pointcloud_trajectory.yaml";
        if (!execute_traj.call(srv))
          ROS_ERROR_STREAM("Was not able to call execute_trajectory from record_pointcloud_trajectory_node service!");
      }
      ros::Duration(5).sleep(); // sleep while trajectory is taking place
    }

    //Service call to record and fuse point cloud
    bachelor_thesis::PoleFound srv;
    if (record_pointcloud.call(srv)) { //No pole was found
      std::cerr << "Service record pointcloud worked!" << std::endl;
      response_pole_found = srv.response.pole_found;
      position_end_ << srv.response.x,srv.response.y,srv.response.z;
      yaw_end_ =  srv.response.yaw;
      length_pole_ = yaw_end_;
    }
    else
      ROS_ERROR_STREAM("Was not able to call pole detection service!");

    if(srv.response.pole_found == 0){
      ROS_ERROR_STREAM("0!");
      ++durchlaeufe2_;  
    }
    if(srv.response.pole_found == 1){
      ROS_ERROR_STREAM("1");
      response_pole_found = 1;
      pole_com_ = position_end_;
      break;
    }
    if(srv.response.pole_found == 2){
      ROS_ERROR_STREAM("2");
    }
    if(srv.response.pole_found == 3){
      ROS_ERROR_STREAM("3");
    }
    if(srv.response.pole_found == 4){
      ROS_ERROR_STREAM("4");
      ++durchlaeufe2_;  
    }
    if(srv.response.pole_found == 5) {
      ROS_ERROR_STREAM("5!");
      ++durchlaeufe2_;  
    }
    if(response_pole_found == 1) {
      return true;
    }
    ++durchlaeufe_;  
  }

  auto stop = high_resolution_clock::now();
  auto duration = duration_cast<microseconds>(stop - start);
  std::cerr << "Execution Time: " << duration.count() / 1000 << std::endl;

  if (response_pole_found == 1)
    return true;
  else
    return false;
}


bool TRAJECTORY_PC::getTrajectory_height(const Eigen::Vector3d &current_position_eigen, const Eigen::Vector3d &current_rotation_eigen, const Eigen::Vector3d &end_position_eigen) {

  double time = 0.5;
  double time1 = 1.5;

  std::vector<double> current_position(&current_position_eigen[0], current_position_eigen.data()+current_position_eigen.cols()*current_position_eigen.rows());
  std::vector<double> current_rotation(&current_rotation_eigen[0], current_rotation_eigen.data()+current_rotation_eigen.cols()*current_rotation_eigen.rows());
  std::vector<double> end_position(&end_position_eigen[0], end_position_eigen.data()+end_position_eigen.cols()*end_position_eigen.rows());
  //std::vector<double> force = {0.0, 0.0, 0.0};
  std::vector<double> position_traj = {current_position[0], current_position[1], 1}; //before: 1.3m

  YAML::Emitter emitter;

  emitter << YAML::BeginMap;
  emitter << YAML::Key << "order_rpy";
  emitter << YAML::Value << 0; // 0 for yaw-pitch-roll, 1 for roll-pitch-yaw
  emitter << YAML::Key << "forces";
  emitter << YAML::Value << false;
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


  YAML::Node yaml_point1 = YAML::Node(YAML::NodeType::Map);
  yaml_point1["pos"] = current_position;
  yaml_point1["att"] = current_rotation;
  yaml_point1["stop"] = true;
  yaml_point1["time"] = time;
  
  YAML::Node yaml_point2 = YAML::Node(YAML::NodeType::Map);
  yaml_point2["pos"] = end_position;
  yaml_point2["att"] = current_rotation;
  yaml_point2["stop"] = true;
  yaml_point2["time"] = time1;

  point_list.push_back(yaml_point1);
  point_list.push_back(yaml_point2);
  
  emitter << YAML::Value << point_list;
  emitter << YAML::EndMap;

  ROS_INFO_STREAM("writeYamlFile");
  std::string filename = "/home/henriette/catkinws/src/mav_ui/omav_local_planner/resource/pointcloud_trajectory.yaml";
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
  }
}



bool TRAJECTORY_PC::getTrajectory_yaw(const Eigen::Vector3d &current_position_eigen, const Eigen::Vector3d &current_rotation_eigen, const Eigen::Vector3d &end_rotation_eigen) {
  double time = 2.0;

  std::vector<double> current_position(&current_position_eigen[0], current_position_eigen.data()+current_position_eigen.cols()*current_position_eigen.rows());
  std::vector<double> current_rotation(&current_rotation_eigen[0], current_rotation_eigen.data()+current_rotation_eigen.cols()*current_rotation_eigen.rows());
  std::vector<double> end_rotation(&end_rotation_eigen[0], end_rotation_eigen.data()+end_rotation_eigen.cols()*end_rotation_eigen.rows());
  std::vector<double> position_traj = {current_position[0], current_position[1], 1}; //before: 1.3m

  YAML::Emitter emitter;

  emitter << YAML::BeginMap;
  emitter << YAML::Key << "order_rpy";
  emitter << YAML::Value << 0; // 0 for yaw-pitch-roll, 1 for roll-pitch-yaw
  emitter << YAML::Key << "forces";
  emitter << YAML::Value << false;
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

  YAML::Node yaml_point1 = YAML::Node(YAML::NodeType::Map);
  yaml_point1["pos"] = current_position;
  yaml_point1["att"] = current_rotation;
  yaml_point1["stop"] = true;
  yaml_point1["time"] = time;
  
  current_position[2] = 1;

  YAML::Node yaml_point2 = YAML::Node(YAML::NodeType::Map);
  yaml_point2["pos"] = current_position;
  yaml_point2["att"] = end_rotation;
  yaml_point2["stop"] = true;
  yaml_point2["time"] = time;

  point_list.push_back(yaml_point1);
  point_list.push_back(yaml_point2);
  
  emitter << YAML::Value << point_list;
  emitter << YAML::EndMap;

  ROS_INFO_STREAM("writeYamlFile");
  std::string filename = "/home/henriette/catkinws/src/mav_ui/omav_local_planner/resource/pointcloud_trajectory.yaml";
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
  }
}

bool TRAJECTORY_PC::Service(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  std::cerr << "Coordinates of pole are: " << pole_com_ << std::endl;

  //Pole position
  Eigen::Vector3d pole_position;
  pole_position = pole_com_;
  
  //Drone position
  Eigen::Vector3d current_position;
  Eigen::Vector3d current_rotation;
  tf_transformer_();
  current_position = T_B_world_1.translation();
  Eigen::Matrix3d rotational_matrix = T_B_world_1.rotation();
  double yaw = atan2(rotational_matrix(1,0),rotational_matrix(0,0));
  current_rotation << yaw,0,0;

  if(getTrajectory_pole(current_position,pole_position,current_rotation)) {
    //trajectory durchführen
    omav_local_planner::ExecuteTrajectory srv;
    srv.request.waypoint_filename = "/home/henriette/catkinws/src/mav_ui/omav_local_planner/resource/go_to_detected_pole.yaml";
    if (!execute_traj.call(srv))
      ROS_ERROR_STREAM("Was not able to call execute_trajectory from go_to_detected_pole service!");
  } 
  return true;
}


bool TRAJECTORY_PC::getTrajectory_pole(const Eigen::Vector3d &current_position_eigen, const Eigen::Vector3d &pole_position_eigen, const Eigen::Vector3d &current_rotation_eigen) {
  double time1 = 0.75;
  double time2 = 3.3;
  std::vector<double> current_position(&current_position_eigen[0], current_position_eigen.data()+current_position_eigen.cols()*current_position_eigen.rows());
  std::vector<double> pole_postion(&pole_position_eigen[0], pole_position_eigen.data()+pole_position_eigen.cols()*pole_position_eigen.rows());
  std::vector<double> current_rotation(&current_rotation_eigen[0], current_rotation_eigen.data()+current_rotation_eigen.cols()*current_rotation_eigen.rows());
  std::vector<double> over_pole_postion;
  std::vector<double> position_height;
  position_height = current_position;
  over_pole_postion = pole_postion;
  position_height[2] = pole_postion[2] + length_pole_/2+0.5;
  over_pole_postion[2] = pole_postion[2] + length_pole_/2+0.5;

  std::cerr << "position_height[2]: " << position_height[2] << std::endl;

  YAML::Emitter emitter;

  emitter << YAML::BeginMap;
  emitter << YAML::Key << "order_rpy";
  emitter << YAML::Value << 0; // 0 for yaw-pitch-roll, 1 for roll-pitch-yaw
  emitter << YAML::Key << "forces";
  emitter << YAML::Value << false;
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
  yaml_point["stop"] = true;
  yaml_point["time"] = time1;   

  YAML::Node yaml_point1 = YAML::Node(YAML::NodeType::Map);
  yaml_point1["pos"] = position_height;
  yaml_point1["att"] = current_rotation;
  yaml_point1["stop"] = true;
  yaml_point1["time"] = time1;   

  YAML::Node yaml_point2 = YAML::Node(YAML::NodeType::Map);
  yaml_point2["pos"] = over_pole_postion;
  yaml_point2["att"] = current_rotation;
  yaml_point2["stop"] = true;
  yaml_point2["time"] = time2;    

  YAML::Node yaml_point3 = YAML::Node(YAML::NodeType::Map);
  yaml_point3["pos"] = pole_postion;
  yaml_point3["att"] = current_rotation;
  yaml_point3["stop"] = true;
  yaml_point3["time"] = time1;


  point_list.push_back(yaml_point);
  point_list.push_back(yaml_point1);
  point_list.push_back(yaml_point1);
  point_list.push_back(yaml_point2);
  point_list.push_back(yaml_point2);
  point_list.push_back(yaml_point3);

  emitter << YAML::Value << point_list;
  emitter << YAML::EndMap;

  ROS_INFO_STREAM("writeYamlFile");
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
  }
}


// Main ROS method
int main(int argc, char **argv) {
    
  // Initialize the node and set the name
  ros::init(argc, argv, "record_pointcloud");
  
  TRAJECTORY_PC trajec_pc;
  ros::NodeHandle nh;
  // Create the service and advertise it to the ROS computational network
  ros::Subscriber sub_2 = nh.subscribe("/odometry", 1, &TRAJECTORY_PC::Callback , &trajec_pc);
  trajec_pc.pointcloud_trajectory = nh.advertiseService("record_and_fusing_pointcloud", &TRAJECTORY_PC::Rec_Fus_PC, &trajec_pc);
  trajec_pc.record_pointcloud = nh.serviceClient<bachelor_thesis::PoleFound>("bachelor_thesis/record_pointcloud");
  trajec_pc.execute_traj = nh.serviceClient<omav_local_planner::ExecuteTrajectory>("/geranos/execute_trajectory");
  trajec_pc.go_to_detected_pole = nh.advertiseService("/trajectory_pointcloud_bt/go_to_detected_pole", &TRAJECTORY_PC::Service, &trajec_pc);

  // Keep processing information over and over again
  ros::spin();
 
  // Program completed successfully
  return 0;
}