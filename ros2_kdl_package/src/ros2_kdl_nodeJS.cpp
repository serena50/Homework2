#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"
 
using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

class Iiwa_pub_sub : public rclcpp::Node
{
    public:
        Iiwa_pub_sub()
        : Node("ros2_kdl_node"), 
        node_handle_(std::shared_ptr<Iiwa_pub_sub>(this))
        {
            // Initialize variables
            iteration_ = 0;
            t_ = 0;
            total_time_ = 1.5;  
            joint_state_available_ = false;
            acc_duration = 0.5;
            radius = 0.2;

            // Declare path type and velocity profile parameters
            declare_parameter("path_type", "linear");
            declare_parameter("vel_profile", "trapezoidal");
            declare_parameter("Kp", 150.0); // Default Kp
            declare_parameter("Kd", 15.0);  // Default Kd

            std::string path_type, vel_profile;
            get_parameter("path_type", path_type);
            get_parameter("vel_profile", vel_profile);

            KDLPlanner::PathType pathType;
            if (path_type == "linear") {
                pathType = KDLPlanner::PathType::LINEAR;
            } else if (path_type == "circular") {
                pathType = KDLPlanner::PathType::CIRCULAR;
            } else {
                RCLCPP_ERROR(get_logger(), "Invalid path type. Using linear.");
                pathType = KDLPlanner::PathType::LINEAR;
            }

            KDLPlanner::VelProfileType velProfileType;
            if (vel_profile == "trapezoidal") {
                velProfileType = KDLPlanner::VelProfileType::TRAPEZOIDAL;
            } else if (vel_profile == "cubic") {
                velProfileType = KDLPlanner::VelProfileType::CUBIC;
            } else {
                RCLCPP_ERROR(get_logger(), "Invalid velocity profile. Using trapezoidal.");
                velProfileType = KDLPlanner::VelProfileType::TRAPEZOIDAL;
            }

            // Create robot
            auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(
                node_handle_, "robot_state_publisher");
            while (!parameters_client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(get_logger(), "Interrupted while waiting for service. Exiting.");
                    rclcpp::shutdown();
                }
                RCLCPP_INFO(get_logger(), "Service not available, waiting...");
            }
            auto parameter = parameters_client->get_parameters({"robot_description"});

            KDL::Tree robot_tree;
            if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)){
                RCLCPP_ERROR(get_logger(), "Failed to construct robot tree");
                return;
            }
            robot_ = std::make_shared<KDLRobot>(robot_tree);  
            
            // Set joint limits
            unsigned int nj = robot_->getNrJnts();
            KDL::JntArray q_min(nj), q_max(nj);
            q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-3.05;
            q_max.data <<  2.96,2.09,2.96,2.09,2.96,2.09,3.05;
            robot_->setJntLimits(q_min,q_max);            
            joint_positions_.resize(nj); 
            joint_velocities_.resize(nj); 

            // Subscribe to joint states
            jointSubscriber_ = create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, 
                std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1));

            // Wait for joint states
            while(!joint_state_available_){
                RCLCPP_INFO(get_logger(), "Waiting for joint states...");
                rclcpp::spin_some(node_handle_);
            }

            // Initialize robot state
            robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
            robot_->addEE(KDL::Frame::Identity());
            robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));

            // Get initial pose
            init_cart_pose_ = robot_->getEEFrame();
            Eigen::Vector3d init_position(init_cart_pose_.p.data);

            // Set trajectory parameters
            Eigen::Vector3d end_position = init_position;
            if (pathType == KDLPlanner::PathType::LINEAR) {
                end_position[0] = init_position[0];
                end_position[1] = -init_position[1];
                end_position[2] = init_position[2];
            }

            // Create planner
            planner_ = KDLPlanner(total_time_, acc_duration, init_position, end_position, 
                                radius, pathType, velProfileType);

            // Create publishers
            effortPublisher_ = create_publisher<FloatArray>("/effort_controller/commands", 10);

            // Create timer
            timer_ = create_wall_timer(10ms, std::bind(&Iiwa_pub_sub::cmd_publisher, this));

            RCLCPP_INFO(get_logger(), "Node initialized successfully");
        }

    private:
        void cmd_publisher()
        {
            iteration_++;
            double dt = 0.01;
            t_ += dt;

            // Get trajectory point
            trajectory_point p;
            if (t_ <= total_time_) {
                p = planner_.compute_trajectory(t_);
                
                // Store final desired position for holding
                if (t_ >= total_time_ - dt) {
                    final_desired_pos_ = p.pos;
                }
            } else {
                // Hold final desired position
                p.pos = final_desired_pos_;
                p.vel = Eigen::Vector3d::Zero();
            }

            // Compute inverse kinematics
            KDL::Frame desFrame = robot_->getEEFrame();
            desFrame.p = KDL::Vector(p.pos[0], p.pos[1], p.pos[2]);
            KDL::JntArray qd(robot_->getNrJnts());
            qd.data = joint_positions_.data;
            robot_->getInverseKinematics(desFrame, qd);

            KDL::JntArray dqd(robot_->getNrJnts());
            KDL::Jacobian J = robot_->getEEJacobian();
            Eigen::VectorXd cart_vel(6);
            cart_vel << p.vel[0], p.vel[1], p.vel[2], 0, 0, 0;
            dqd.data = pseudoinverse(J.data) * cart_vel;

            KDL::JntArray ddqd(robot_->getNrJnts());
            ddqd.data.setZero();

            double Kp = 150.0; 
            double Kd = 15;
            
            KDLController controller(*robot_);
            Eigen::VectorXd tau = controller.idCntr(qd, dqd, ddqd, Kp, Kd);

            // Publish effort commands
            auto effort_msg = std::make_unique<FloatArray>();
            effort_msg->data.resize(robot_->getNrJnts());
            for (size_t i = 0; i < robot_->getNrJnts(); i++) {
                effort_msg->data[i] = tau[i];
            }
            effortPublisher_->publish(std::move(effort_msg));

            // Debug info
            if (iteration_ % 10 == 0) {
                Eigen::Vector3d curr_pos(robot_->getEEFrame().p.data);
                Eigen::Vector3d des_pos(p.pos);
                double pos_error = (des_pos - curr_pos).norm();
                RCLCPP_INFO(get_logger(), 
                            "t: %.2f s\nError: %.3f m\nDes:  [%.3f, %.3f, %.3f]\nCurr: [%.3f, %.3f, %.3f]",
                            t_, pos_error,
                            des_pos[0], des_pos[1], des_pos[2],
                            curr_pos[0], curr_pos[1], curr_pos[2]);
            }
        }

        void joint_state_subscriber(const sensor_msgs::msg::JointState& msg)
        {
            joint_state_available_ = true;
            for (size_t i = 0; i < msg.position.size(); i++)
            {
                joint_positions_.data[i] = msg.position[i];
                joint_velocities_.data[i] = msg.velocity[i];
            }
            robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
        }
private:
        std::shared_ptr<rclcpp::Node> node_handle_;
        rclcpp::Publisher<FloatArray>::SharedPtr effortPublisher_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
        rclcpp::TimerBase::SharedPtr timer_;

        Eigen::Vector3d final_desired_pos_;
        KDLPlanner planner_;
        KDL::Frame init_cart_pose_;
        std::shared_ptr<KDLRobot> robot_;
        KDL::JntArray joint_positions_, joint_velocities_;
        bool joint_state_available_;
        double t_, total_time_;
        double acc_duration;
        double radius;
        int iteration_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Iiwa_pub_sub>());
    rclcpp::shutdown();
    return 0;
}

