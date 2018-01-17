#include <controllers/pid_joint_space.h>

// Create a new instance of PIDjointSpace. The PreOperational argument forces you to call configureHook().
PIDjointSpace::PIDjointSpace(const std::string& name) : RTT::TaskContext(name, PreOperational)
{
   // Add RTT ports.
   this->addPort("joint_states", inPort_JointStates);
   this->addPort("trajectory_goal_pose", inPort_TrajectoryGoalPose);
   this->addPort("torque_commands", outPort_TorqueCommands);

   // Add RTT properties.
   this->addProperty("joint_P_gains", Kp).doc("propertional gains");
   this->addProperty("joint_I_gains", Ki).doc("integral gains");
   this->addProperty("joint_D_gains", Kd).doc("derivative gians");
   this->addProperty("robot_description", robot_description).doc("URDF or SDF");
   this->addProperty("root_link", root_link).doc("first link in KDL chain");
   this->addProperty("tip_link", tip_link).doc("last link in KDL chain");
   this->addProperty("joint_start_pose", joint_start_pose).doc("array of joint start positions");
   this->addProperty("joint_space_velocity_profile", joint_space_velocity_profile).doc("selected joint space velocity profile");
   this->addProperty("max_solve_time", max_solve_time).doc("max allowable solve time for IK solver.");
   this->addProperty("error", error).doc("max allowable error in IK solution.");
   this->addProperty("use_joint_vel_lims", use_joint_vel_lims).doc("use velocity limits to calculate velocity profile");
   this->addProperty("use_joint_acc_lims", use_joint_acc_lims).doc("use acceleration limits to calculate velocity profile");
   this->addProperty("joint_vel_lims", joint_vel_lims).doc("maximum joint velocities");
   this->addProperty("joint_accel_lims", joint_acc_lims).doc("maximum joint accelerations");
   this->addProperty("frame_id", frame_id).doc("reference frame for trajectory waypoints");
   this->addProperty("tranform_waypoints_to_new_frame", transform_points).doc("transform waypoints to new reference frame");
}

// Configure this component.
bool PIDjointSpace::configureHook()
{
   // Abort if any ports are not connected.
   if(!inPort_JointStates.connected()){
      RTT::log(RTT::Error) << "inPort_JointStates is not connected!" << RTT::endlog();
      return false;
   }
   if(!inPort_TrajectoryGoalPose.connected()){
      RTT::log(RTT::Error) << "inPort_TrajectoryGoalPose is not connected!" << RTT::endlog();
      return false;
   }
   if(!outPort_TorqueCommands.connected()){
      RTT::log(RTT::Error) << "outPort_TorqueCommands is not connected!" << RTT::endlog();
      return false;
   }
   // Get KDL chain
   rtt_ros_tools::getChainFromURDF(kdl_chain, this);

   // Resize.
   Kp.resize(kdl_chain.getNrOfJoints());
   Ki.resize(kdl_chain.getNrOfJoints());
   Kd.resize(kdl_chain.getNrOfJoints());
   joint_vel_lims.resize(kdl_chain.getNrOfJoints());
   joint_acc_lims.resize(kdl_chain.getNrOfJoints());
   joint_start_pose.resize(kdl_chain.getNrOfJoints());
   joint_final_pose.resize(kdl_chain.getNrOfJoints());
   joint_states.q.resize(kdl_chain.getNrOfJoints());
   joint_states.qdot.resize(kdl_chain.getNrOfJoints());
   desired_joint_states.q.resize(kdl_chain.getNrOfJoints());
   desired_joint_states.qdot.resize(kdl_chain.getNrOfJoints());
   joint_states_msg.position.resize(kdl_chain.getNrOfJoints()); 
   joint_states_msg.velocity.resize(kdl_chain.getNrOfJoints());
   torque_commands.effort.resize(kdl_chain.getNrOfJoints());
   
   // Parse parameters.
   rtt_ros_tools::getJointSpaceVelProfileName(joint_space_velocity_profile, this);
   joint_space_vel_prof::getJointSpaceVelProfile(joint_space_velocity_profile, kdl_chain, vel_prof, this);
   rtt_ros_tools::getJointSpacePgains(kdl_chain, Kp, this);
   rtt_ros_tools::getJointSpaceIgains(kdl_chain, Ki, this);
   rtt_ros_tools::getJointSpaceDgains(kdl_chain, Kd, this);
   rtt_ros_tools::getIKsolver(ik_solver, this);
   rtt_ros_tools::getJointStartPose(kdl_chain, joint_start_pose);
   rtt_ros_tools::transformWaypointsToNewFrame(transform_points, this);
   if(transform_points) rtt_ros_tools::getFrameID(frame_id, this);

   return true;
}

// Start this component.
bool PIDjointSpace::startHook()
{
   current_time = 0.0;
   duration = 0.0;
   return true;
}

// Run update loop.
void PIDjointSpace::updateHook()
{
   // Read current joint states.
   inPort_JointStates.read(joint_states_msg);
   
   // Convert joint state message to KDL JntArrayVel.
   joint_states.q.data = Eigen::VectorXd::Map(joint_states_msg.position.data(), joint_states_msg.position.size());
   joint_states.qdot.data = Eigen::VectorXd::Map(joint_states_msg.velocity.data(), joint_states_msg.velocity.size());
   
   // If the current trajectory duration is over, check for new goal poses.
   if(current_time >= duration){
      if(inPort_TrajectoryGoalPose.read(trajectory_goal_pose) == RTT::NewData){
	 
	 if(transform_points)
	    tf_listener.transformPose(frame_id, trajectory_goal_pose, trajectory_goal_pose);
	 
	 // Convert pose message to KDL frame.
	 tf::poseMsgToKDL(trajectory_goal_pose.pose, trajectory_goal_frame);
	 
	 // Find an IK solution.
	 ik_solver->CartToJnt(joint_states.q, trajectory_goal_frame, joint_final_pose);
      }
      // Set to start position if there are no new goal poses.
      else joint_final_pose.data = joint_start_pose.data;
      
      // Set up velocity profile.
      vel_prof->solve(joint_states.q, joint_final_pose, duration);
      
      // Reset time before beginning new trajectory.
      current_time = 0.0;
   }
   // Get desired joint positions.
   vel_prof->get_desired_joint_pos(desired_joint_states.q, current_time);
   
   // Get desired joint velocities.
   vel_prof->get_desired_joint_vel(desired_joint_states.qdot, current_time);
   
   // Increase timer.
   current_time += this->getPeriod();

   // Create torque commands message.
   for(int i=0; i<joint_states.q.rows(); ++i){
      torque_commands.effort[i] = (desired_joint_states.q(i) - joint_states.q(i))*Kp[i]
      + (desired_joint_states.qdot(i) - joint_states.qdot(i))*Kd[i]
      + (desired_joint_states.q(i) - joint_states.q(i))*Ki[i];
   }
   // Write torque commands.
   outPort_TorqueCommands.write(torque_commands);
}
