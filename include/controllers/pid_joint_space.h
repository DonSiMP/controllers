#ifndef PID_JOINT_SPACE_H
#define PID_JOINT_SPACE_H

#include <rtt_ros_tools/rtt_ros_tools.h>
#include <trajectory_generators/joint_space_velocity_profiles.h>

#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>

#include <kdl/jntarrayacc.hpp>
#include <kdl/frames.hpp>
#include <kdl/chain.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>

#include <tf_conversions/tf_kdl.h>
#include <Eigen/Core>
#include <memory>

class PIDjointSpace : public RTT::TaskContext{

public:

   PIDjointSpace(const std::string& name);

   bool configureHook();
   bool startHook();
   void updateHook();

   virtual ~PIDjointSpace(){}

protected:

   RTT::InputPort<sensor_msgs::JointState> inPort_JointStates;
   RTT::InputPort<geometry_msgs::PoseStamped> inPort_TrajectoryGoalPose;

   RTT::OutputPort<sensor_msgs::JointState> outPort_TorqueCommands;

   bool use_joint_vel_lims, use_joint_acc_lims, transform_points;
   std::vector<double> Kp, Ki, Kd, joint_vel_lims, joint_acc_lims;
   KDL::JntArray joint_start_pose, joint_final_pose;
   KDL::JntArrayVel joint_states, desired_joint_states;
   sensor_msgs::JointState joint_states_msg;
   double current_time, duration, max_solve_time, error;
   KDL::Chain kdl_chain;
   KDL::Frame trajectory_goal_frame;
   geometry_msgs::PoseStamped trajectory_goal_pose;
   std::unique_ptr<joint_space_vel_prof::velocityProfile> vel_prof;
   std::unique_ptr<TRAC_IK::TRAC_IK> ik_solver;
   std::string robot_description, root_link, tip_link, joint_space_velocity_profile, frame_id;
   sensor_msgs::JointState torque_commands;
   tf::TransformListener tf_listener;
};

ORO_LIST_COMPONENT_TYPE(PIDjointSpace)

#endif // PID_JOINT_SPACE_H
