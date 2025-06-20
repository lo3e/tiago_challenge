// -----------------------------------------------------------------------------
// main_state_machine.cpp
// -----------------------------------------------------------------------------
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

//-------------- ----
#include <chrono>
#include <array>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include "your_pkg/your_node.hpp"        // ← sostituisci col nome reale

using namespace std::chrono_literals;

static const Eigen::Matrix<double, 7, 4> BlockMatrix{ {
        /* col0 */ { 0.00,  0.025, 0.00,   0.0, 0.7071068, 0.0, 0.7071068 },
        /* col1 */ { 0.00,  -0.025, 0.00,   0.0, 0.7071068, 0.0, 0.7071068 },
        /* col2 */ { 0.025,  0.00, 0.015,   -0.5, -0.5, 0.5, -0.5},
        /* col3 */ { -0.025,  0.00, 0.015,   -0.5, -0.5, 0.5, -0.5}
    } };

size_t i_col = 0;          
double P_off = 0.0;        // incremental Z (+0.03 each 4 cycles)

/* Marker Pose  */
static const Eigen::Vector3d P_target(0.40, 0.00, 0.00);   // x,y,z in m

enum class State
{
    INIT_OPEN_GRIPPER,   // passo 0
    MOVE_START,          // passo 1 + 2
    KEYBOARD_LOOP,       // passo 3
    MOVE_DOWN,           // passo 4
    OPEN_GRIPPER,        // passo 5
    MOVE_UP,             // passo 6
    UPDATE_OFFSETS       // passo 7
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<YourNode>();   // ← sostituisci con il tuo
    State state = State::INIT_OPEN_GRIPPER;

    while (rclcpp::ok())
    {
        /* ---------- Estrae dalla colonna corrente px,py,pz, qw,qx,qy,qz ---- */
        Eigen::Vector3d P_xy = BlockMatrix.block<3, 1>(0, i_col);   // righe 0-2
        Eigen::Quaterniond O_i(
            BlockMatrix(3, i_col),   // qw
            BlockMatrix(4, i_col),   // qx
            BlockMatrix(5, i_col),   // qy
            BlockMatrix(6, i_col));  // qz

        /* ------------ Pose utili ------------------------------------------- */
        auto make_pose = [](const Eigen::Vector3d& p, const Eigen::Quaterniond& q, const Eigen::Matrix& BlockMatrix )
            {
                geometry_msgs::msg::Pose pose;
                pose.position.x = p.x();
                pose.position.y = p.y();
                pose.position.z = p.z();
                
                pose.orientation.x = BlockMatrix(3, i_col);
                pose.orientation.y = BlockMatrix(4, i_col);
                pose.orientation.z = BlockMatrix(5, i_col);
                pose.orientation.w = BlockMatrix(6, i_col);
                // pose.orientation.w = q.w();
                // pose.orientation.x = q.x();
                // pose.orientation.y = q.y();
                // pose.orientation.z = q.z();
                return pose;
            };

        Eigen::Vector3d P_start = P_target + P_xy + Eigen::Vector3d(0, 0, P_off + 0.335);   // punto alto
        Eigen::Vector3d P_down = P_target + P_xy + Eigen::Vector3d(0, 0, P_off + 0.235);   // punto basso

        switch (state)
        {
            /* ------------------------------------------------------------------ */
        case State::INIT_OPEN_GRIPPER:
            RCLCPP_INFO(node->get_logger(), "INIT → apro il gripper");
            node->GripperControl("OPEN");
            state = State::MOVE_START;
            break;

            /* ------------------------------------------------------------------ */
        case State::MOVE_START:   // (P_target + P_xy_i + P_off + 0.335) con orient. O_i
        {
            RCLCPP_INFO(node->get_logger(), "MOVE_START: colonna %zu", i_col + 1);
            auto pose_start = make_pose(P_start, O_i);
            node->motion_planning_control(pose_start, RobotTaskStatus::Arm::ARM_torso);
            state = State::KEYBOARD_LOOP;
            break;
        }

        /* ------------------------------------------------------------------ */
        case State::KEYBOARD_LOOP:   // resta finché non si preme <SPAZIO>
        {
            char key = read_key_non_blocking();   // ↔ implementa a tua scelta
            if (key == 'C' || key == 'c')
            {
                RCLCPP_INFO(node->get_logger(), "→ chiudo gripper");
                node->GripperControl("CLOSE");
            }
            else if (key == 'O' || key == 'o')
            {
                RCLCPP_INFO(node->get_logger(), "→ apro gripper");
                node->GripperControl("OPEN");
            }
            else if (key == ' ')
            {
                RCLCPP_INFO(node->get_logger(), "<SPACE> premuto → scendo");
                state = State::MOVE_DOWN;
            }
            /* altrimenti rimango qui */
            break;
        }

        /* ------------------------------------------------------------------ */
        case State::MOVE_DOWN:     // (P_target + P_xy_i + P_off + 0.235)
        {
            auto pose_down = make_pose(P_down, O_i);
            node->motion_planning_control(pose_down, RobotTaskStatus::Arm::ARM_torso);
            state = State::OPEN_GRIPPER;
            break;
        }

        /* ------------------------------------------------------------------ */
        case State::OPEN_GRIPPER:  // passo 5
            node->GripperControl("OPEN");
            state = State::MOVE_UP;
            break;

            /* ------------------------------------------------------------------ */
        case State::MOVE_UP:       // torna a +0.335
        {
            auto pose_up = make_pose(P_start, O_i);
            node->motion_planning_control(pose_up, RobotTaskStatus::Arm::ARM_torso);
            state = State::UPDATE_OFFSETS;
            break;
        }

        /* ------------------------------------------------------------------ */
        case State::UPDATE_OFFSETS:
        {
            /* Aggiorna colonna e offset Z ogni 4 cicli -------------------- */
            i_col = (i_col + 1) % 4;        // 0→1→2→3→0
            if (i_col == 0)                 // abbiamo completato un “layer”
            {
                P_off += 0.03;              // +3 cm
                RCLCPP_INFO(node->get_logger(),
                    "Nuovo layer: P_off = %.3f m", P_off);
            }
            state = State::MOVE_START;      // ricomincia il ciclo
            break;
        }
        } // switch
        rclcpp::spin_some(node);
    } // while

    rclcpp::shutdown();
    return 0;
}
