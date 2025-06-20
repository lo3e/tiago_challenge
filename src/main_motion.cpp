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
#include <string>
#include <thread>
#include <chrono>
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Create a shared pointer to your node
  auto node = std::make_shared<icr_Motionplanning_arms>();
  RobotTaskStatus robot_status;
  robot_status.setStatus(RobotTaskStatus::Status::QUEUED);

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


  geometry_msgs::msg::PoseStamped obstacle_pose;
  obstacle_pose.header.frame_id = "base_footprint";
  obstacle_pose.pose.position.x = 0.3;
  obstacle_pose.pose.position.y = 0.0;
  obstacle_pose.pose.position.z = 0.52;
  obstacle_pose.pose.orientation.x = 0.0;
  obstacle_pose.pose.orientation.y = 0.0;
  obstacle_pose.pose.orientation.z = 0.0;
  obstacle_pose.pose.orientation.w = 1.0;
  moveit_msgs::msg::PlanningScene planning_scene_msg =
    node->Add_Obstacle(obstacle_pose, "Table");
  planning_scene_publisher_->publish(planning_scene_msg);


  std::vector<float> marker = {0.4, 0.0, 0.52};
  float gripper = 0.235;
  float layer = 0.015;
  float tavolo = 0.0;
  std::vector<float> base_x = {0.0, 0.0, 0.025, -0.025};
  std::vector<float> base_y = {0.025, -0.025, 0, 0};
  std::vector<float> base_z = {0.0, 0.0, layer, layer};
  std::vector<float> orizontal = {-0.5, -0.5, 0.5, -0.5};
  std::vector<float> vertical = {0.0, 0.7071068, 0.0, 0.7071068};
  std::vector<float> goal_x, goal_y, goal_z;

  for (size_t i = 0; i < base_x.size(); ++i) {
      goal_x.push_back(base_x[i] + marker[0]);
      goal_y.push_back(base_y[i] + marker[1]);
      goal_z.push_back(base_z[i] + marker[2] + gripper);
  }


  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = goal_x[0]; 
  target_pose.position.y = goal_y[0];  
  target_pose.position.z = goal_z[0]+0.10;  

  // Simple orientation (quaternion), fastd::cing forward
  target_pose.orientation.x = vertical[0];
  target_pose.orientation.y = vertical[1];  
  target_pose.orientation.z = vertical[2];
  target_pose.orientation.w = vertical[3];

  char input = 'a';
  int i = 0 ;
  rclcpp::Rate loop_rate(1);
  while (rclcpp::ok())
  {
    RobotTaskStatus::Status current_status=robot_status.getStatus();
    switch(current_status){
      case RobotTaskStatus::Status::QUEUED:
        target_pose.position.x = goal_x[i]; 
        target_pose.position.y = goal_y[i];  
        target_pose.position.z = goal_z[i]+0.10;  

        // Simple orientation (quaternion), fastd::cing forward
        if(i<2)
        {
          target_pose.orientation.x = vertical[0];
          target_pose.orientation.y = vertical[1];  
          target_pose.orientation.z = vertical[2];
          target_pose.orientation.w = vertical[3];
        }
        else
        {
          target_pose.orientation.x = orizontal[0];
          target_pose.orientation.y = orizontal[1];  
          target_pose.orientation.z = orizontal[2];
          target_pose.orientation.w = orizontal[3];
        }
        try {
          node->motion_planning_control(target_pose, RobotTaskStatus::Arm::ARM_torso);  // or "arm" depending on your group name
          RCLCPP_INFO(node->get_logger(), "Motion planning succeeded.");
          robot_status.setStatus(RobotTaskStatus::Status::WAITING_BRICK);
          std::this_thread::sleep_for(std::chrono::seconds(3));
        } catch (const std::exception & e) {
          RCLCPP_ERROR(node->get_logger(), "Motion planning failed: %s", e.what());
        }
      break;
      case RobotTaskStatus::Status::WAITING_BRICK:
            input='a';
            std::cout << "Press g to close the gripper: ";
            std::cin >> input;
            if (input == 'g') {
              std::this_thread::sleep_for(std::chrono::seconds(1));
              node->GripperControl("CLOSE");
              robot_status.setStatus(RobotTaskStatus::Status::GRASPED_BRICK);
            }
      break;
      case RobotTaskStatus::Status::GRASPED_BRICK:
        input='a';
        std::cout << "Press y to confirm otherwise press n : ";
        std::cin >> input;
        if(input=='y')
        {
            target_pose.position.x = goal_x[i]; 
            target_pose.position.y = goal_y[i];  
            target_pose.position.z = goal_z[i];  

            // Simple orientation (quaternion), fastd::cing forward
            if(i<=1)
            {
              target_pose.orientation.x = vertical[0];
              target_pose.orientation.y = vertical[1];  
              target_pose.orientation.z = vertical[2];
              target_pose.orientation.w = vertical[3];
            }
            else
            {
              target_pose.orientation.x = orizontal[0];
              target_pose.orientation.y = orizontal[1];  
              target_pose.orientation.z = orizontal[2];
              target_pose.orientation.w = orizontal[3];
            }
            robot_status.setStatus(RobotTaskStatus::Status::ONGOING);
        }
        else if (input == 'n')
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            node->GripperControl("OPEN");
            robot_status.setStatus(RobotTaskStatus::Status::WAITING_BRICK);
        }
        break;
      case RobotTaskStatus::Status::ONGOING:
        try {
          node->motion_planning_control(target_pose, RobotTaskStatus::Arm::ARM_torso);  // or "arm" depending on your group name
          RCLCPP_INFO(node->get_logger(), "Motion planning succeeded.");
          robot_status.setStatus(RobotTaskStatus::Status::LEAVE_BRICK);
        } catch (const std::exception & e) {
          RCLCPP_ERROR(node->get_logger(), "Motion planning failed: %s", e.what());
        }
        break;
      case RobotTaskStatus::Status::LEAVE_BRICK:
        std::this_thread::sleep_for(std::chrono::seconds(3));
        node->GripperControl("OPEN");
        std::this_thread::sleep_for(std::chrono::seconds(3));
        std::cout << "Gripper opened, waiting for 5 seconds..." << std::endl;
        // std::this_thread::sleep_for(std::chrono::seconds(5));
        std::cout << "Gripper opened, ended sleep for 5 seconds..." << std::endl;
        if (i==0){
          for (size_t i = 0; i < goal_z.size(); ++i) {
            goal_z[i]=goal_z[i]+layer;
          }
        }
        robot_status.setStatus(RobotTaskStatus::Status::DONE);
        break;
      case RobotTaskStatus::Status::DONE:
        target_pose.position.x = goal_x[i]; 
        target_pose.position.y = goal_y[i];  
        target_pose.position.z = goal_z[i]+0.10;  

        // Simple orientation (quaternion), fastd::cing forward
        if(i<2)
        {
          target_pose.orientation.x = vertical[0];
          target_pose.orientation.y = vertical[1];  
          target_pose.orientation.z = vertical[2];
          target_pose.orientation.w = vertical[3];
        }
        else
        {
          target_pose.orientation.x = orizontal[0];
          target_pose.orientation.y = orizontal[1];  
          target_pose.orientation.z = orizontal[2];
          target_pose.orientation.w = orizontal[3];
        }
        try {
          node->motion_planning_control(target_pose, RobotTaskStatus::Arm::ARM_torso);  // or "arm" depending on your group name
          RCLCPP_INFO(node->get_logger(), "Motion planning succeeded.");
          std::this_thread::sleep_for(std::chrono::seconds(3));
          robot_status.setStatus(RobotTaskStatus::Status::QUEUED);

        } catch (const std::exception & e) {
          RCLCPP_ERROR(node->get_logger(), "Motion planning failed: %s", e.what());
        }
        if(i==1 || i==3)
        {
          obstacle_pose.header.frame_id = "base_footprint";
          obstacle_pose.pose.position.x = 0.3;
          obstacle_pose.pose.position.y = 0.0;
          obstacle_pose.pose.position.z = obstacle_pose.pose.position.z + layer ;
          obstacle_pose.pose.orientation.x = 0.0;
          obstacle_pose.pose.orientation.y = 0.0;
          obstacle_pose.pose.orientation.z = 0.0;
          obstacle_pose.pose.orientation.w = 1.0;
          planning_scene_msg =
            node->Add_Obstacle(obstacle_pose, "Table");
          planning_scene_publisher_->publish(planning_scene_msg);
        }
        i=(i+1)%4;
        break;
    }
  }


  rclcpp::shutdown();
  return 0;
}
