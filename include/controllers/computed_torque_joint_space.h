#ifndef COMPUTED_TORQUE_JOINT_SPACE_H
#define COMPUTED_TORQUE_JOINT_SPACE_H

#include <rtt_ros_tools/rtt_ros_tools.h>
#include <trajectory_generators/joint_space_velocity_profiles.h>

#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>

#include <kdl/jntarray.hpp>
#include <kdl/jntarrayacc.hpp>
#include <kdl/frames.hpp>
#include <kdl/chaindynparam.hpp>

#include <eigen_conversions/eigen_kdl.h>
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>

#include <Eigen/Core>
#include <memory>

class ComputedTorqueJointSpace : public RTT::TaskContext{

public:

   ComputedTorqueJointSpace(const std::string& name);

   bool configureHook();
   bool startHook();
   void updateHook();

   virtual ~ComputedTorqueJointSpace(){}

protected:
   
   RTT::InputPort<geometry_msgs::PoseStamped> inPort_TrajectoryGoalPose;
   RTT::InputPort<sensor_msgs::JointState> inPort_JointStates;

   RTT::OutputPort<sensor_msgs::JointState> outPort_TorqueCommands;

   bool use_joint_vel_lims, use_joint_acc_lims, transform_points;
   double current_time, duration, max_solve_time, error;
   std::vector<double> Kp, Kd, joint_vel_lims, joint_acc_lims;
   KDL::Vector grav_vec;
   sensor_msgs::JointState joint_states_msg;
   KDL::JntArrayVel joint_states;
   KDL::JntArrayAcc desired_joint_states;
   KDL::JntArray joint_start_pos, joint_final_pos;
   KDL::JntArray coriolis, gravity, dynam_commands, PID_commands;
   KDL::Chain kdl_chain;
   std::unique_ptr<KDL::ChainDynParam> chain_dynamic_params;
   KDL::JntSpaceInertiaMatrix mass_mat;
   geometry_msgs::PoseStamped trajectory_goal_pose;
   KDL::Frame trajectory_goal_frame;
   std::unique_ptr<joint_space_vel_prof::velocityProfile> vel_prof;
   std::unique_ptr<TRAC_IK::TRAC_IK> ik_solver;
   std::string robot_description, root_link, tip_link, frame_id;
   std::string vel_prof_name, joint_space_velocity_profile;
   sensor_msgs::JointState torque_commands_msg;
   Eigen::VectorXd torque_commands;
   tf::TransformListener tf_listener;
};

ORO_LIST_COMPONENT_TYPE(ComputedTorqueJointSpace)

#endif 
