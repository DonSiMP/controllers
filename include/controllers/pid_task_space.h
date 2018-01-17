#ifndef PID_TASK_SPACE_H
#define PID_TASK_SPACE_H

#include <rtt_ros_tools/rtt_ros_tools.h>

#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>
#include <rtt_rosclock/rtt_rosclock.h>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include <eigen_conversions/eigen_kdl.h>
#include <tf_conversions/tf_kdl.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

#include <Eigen/Core>
#include <memory>

class PIDtaskSpace : public RTT::TaskContext{

public:

   PIDtaskSpace(const std::string& name);

   bool configureHook();
   bool startHook();
   void updateHook();

   virtual ~PIDtaskSpace(){}

protected:

   RTT::InputPort<geometry_msgs::Pose> inPort_DesiredTaskPos;
   RTT::InputPort<geometry_msgs::Twist> inPort_DesiredTaskVel;
   RTT::InputPort<sensor_msgs::JointState> inPort_CurrentJointStates;

   RTT::OutputPort<sensor_msgs::JointState> outPort_TorqueCommands;
   RTT::OutputPort<geometry_msgs::PoseArray> outPort_TrajectoryGenerator;

   KDL::Chain kdl_chain;
   std::string robot_description, root_link, tip_link, ee_frame_name;
   int ee_frame_index;
   geometry_msgs::Pose ee_start_pose_msg;
   geometry_msgs::PoseArray ee_start_pose_to_port;
   std::vector<double> Kp, Ki, Kd;
   sensor_msgs::JointState joint_states_msg;
   KDL::JntArray joint_start_pos;
   KDL::JntArrayVel joint_states;
   geometry_msgs::Pose current_joint_pos_msg, desired_task_pos_msg;
   geometry_msgs::Twist current_joint_vel_msg, desired_task_vel_msg;
   KDL::Frame current_task_pos, desired_task_pos, ee_start_pose_frame;
   KDL::Twist current_task_vel, desired_task_vel, pose_error, PID_commands;
   KDL::FrameVel current_task_vel_solver;
   std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver;
   std::unique_ptr<KDL::ChainFkSolverVel_recursive> fk_vel_solver;
   std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_solver;
   KDL::Jacobian jacobian;
   Eigen::MatrixXd jacobian_transpose;
   Eigen::VectorXd torque_commands;
   sensor_msgs::JointState torque_commands_msg;
   Eigen::Matrix<double,6,1> pid_commands;
};

ORO_CREATE_COMPONENT_LIBRARY()
ORO_LIST_COMPONENT_TYPE(PIDtaskSpace)

#endif // PID_TASK_SPACE_H
