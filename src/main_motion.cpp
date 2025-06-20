#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "std_msgs/msg/string.hpp"

#include "Motionplanning_arms.hpp"
#include "RobotTaskStatus.hpp"

#include <rclcpp_action/rclcpp_action.hpp>
#include <play_motion2_msgs/action/play_motion2.hpp>
#include <control_msgs/action/point_head.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <yaml-cpp/yaml.h>
#include <memory>
#include <chrono>
#include <vector>
#include <iostream>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Create a shared pointer to your node
  auto node = std::make_shared<icr_Motionplanning_arms>();

  node->GripperControl("OPEN");
  // You can spin in a separate thread or just call your function directly
  try {
    double lift_value = 0.0;   // Example lift value within limits
    // node->TorsoControl(lift_value);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Exception in TorsoControl: %s ", e.what());
  }
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_publisher_;
  planning_scene_publisher_ = node->create_publisher<moveit_msgs::msg::PlanningScene>(
    "/planning_scene", rclcpp::QoS(1));

// Add obj table in pose 0.4 0.0 0.5
  // geometry_msgs::msg::PoseStamped obstacle_pose;
  // obstacle_pose.header.frame_id = "base_footprint";
  // obstacle_pose.pose.position.x = 0.4;
  // obstacle_pose.pose.position.y = 0.0;
  // obstacle_pose.pose.position.z = 0.5;
  // obstacle_pose.pose.orientation.x = 0.0;
  // obstacle_pose.pose.orientation.y = 0.0;
  // obstacle_pose.pose.orientation.z = 0.0;
  // obstacle_pose.pose.orientation.w = 1.0;
  // moveit_msgs::msg::PlanningScene planning_scene_msg =
  //   node->Add_Obstacle(obstacle_pose, "Table");
  // planning_scene_publisher_->publish(planning_scene_msg);
  vector<float> marker = {0.1,0.2,0.4};
  float gripper= 0.235;
  float layer = 0.015
  vector<float> goal_y = {0.025, -0.025, 0, 0} + marker[1];
  vector<float> goal_x = {0.0, 0.0, 0.025, -0.025}+ marker[0];
  vector<float> goal_z = {0.0, 0.0, layer, layer}+marker[2]+gripper;
  vector<float> orizontal = {-0.5, -0.5, 0.5, -0.5};
  vector<float> vertical = {0.0, 0.7071068, 0.0, 0.7071068};


  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = goal_x[0]; 
  target_pose.position.y = goal_y[0];  
  target_pose.position.z = goal_z[0];  

  // Simple orientation (quaternion), fastd::cing forward
  target_pose.orientation.x = vertical[0];
  target_pose.orientation.y = vertical[1] ;  
  target_pose.orientation.z = vertical[2];
  target_pose.orientation.w = verttical[3];
  // geometry_msgs::msg::Pose target_pose2;
  // target_pose2.position.x = 0.4;
  // target_pose2.position.y = 0.0;
  // target_pose2.position.z = 0.5;

  // // Simple orientation (quaternion), fastd::cing forward
  // target_pose2.orientation.x = 0.0;
  // target_pose2.orientation.y = 0.7071068;  
  // target_pose2.orientation.z = 0.0;
  // target_pose2.orientation.w = 0.7071068;
  
  // geometry_msgs::msg::Pose target_pose3;
  // target_pose3.position.x = 0.4;
  // target_pose3.position.y = 0.0;
  // target_pose3.position.z = 0.5;

  // // Simple orientation (quaternion), fastd::cing forward
  // target_pose3.orientation.x = 0.5;
  // target_pose3.orientation.y = 0.5;  
  // target_pose3.orientation.z = 0.5;
  // target_pose3.orientation.w = 0.5;

  // geometry_msgs::msg::Pose target_pose4;
  // target_pose4.position.x = 0.4;
  // target_pose4.position.y = 0.0;
  // target_pose4.position.z = 0.5;

  // // Simple orientation (quaternion), fastd::cing forward
  // target_pose4.orientation.x = 0.5;
  // target_pose4.orientation.y = 0.5;  
  // target_pose4.orientation.z = 0.5;
  // target_pose4.orientation.w = 0.5;


  try {
    node->motion_planning_control(target_pose, RobotTaskStatus::Arm::ARM_torso);  // or "arm" depending on your group name
    RCLCPP_INFO(node->get_logger(), "Motion planning succeeded.");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Motion planning failed: %s", e.what());
  }
  char input = 'a';

  while (input != 'g')
  {
      std::cout << "Press Space to close the gripper: ";
      std::cin >> input;
  }
  vector<int> v = {0, 1, 2};
  int state = 0;
  node->GripperControl("CLOSE");

  try {
    node->motion_planning_control(target_pose2, RobotTaskStatus::Arm::ARM_torso);  // or "arm" depending on your group name
    RCLCPP_INFO(node->get_logger(), "Motion planning succeeded.");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Motion planning failed: %s", e.what());
  }

  while (input != ' ')
  {
      std::cout << "Press Space to close the gripper: ";
      std::cin >> input;
  }
  node->GripperControl("OPEN"); 
  // vector<int> v = {0, 1, 2};
  // int state = 0;
  // while (true)
  // {

  //   if (input == ' ') {
  //       std::cout << "Hai premuto spazio." << endl;
  //   } else if (input == 'y') {
  //       std::cout << "Hai premuto y." << endl;
  //   } else if (input == 'n') {
  //       std::cout << "Hai premuto n." << endl;
  //   } else {
  //       std::cout << "Hai premuto: " << input << endl;
  //   }
  // }


  rclcpp::shutdown();
  return 0;
}
