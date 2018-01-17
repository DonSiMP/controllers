#include <controllers/computed_torque_joint_space.h>

// Create a new instance of ComputedTorqueJointSpace. The PreOperational argument forces you to call configureHook().
ComputedTorqueJointSpace::ComputedTorqueJointSpace(const string &name) : RTT::TaskContext(name, PreOperational)
{
   // Add RTT ports.
   this->addPort("joint_states", inPort_JointStates);
   this->addPort("trajectory_goal_pose", inPort_TrajectoryGoalPose);
   this->addPort("torque_commands", outPort_TorqueCommands);

   // Add RTT properties.
   this->addProperty("joint_P_gains", Kp).doc("propertional gains");
   this->addProperty("joint_D_gains", Kd).doc("derivative gians");
   this->addProperty("robot_description", robot_description).doc("URDF or SDF");
   this->addProperty("root_link", root_link).doc("first link in KDL chain");
   this->addProperty("tip_link", tip_link).doc("last link in KDL chain");
   this->addProperty("joint_start_pose", joint_start_pos).doc("array of joint start positions");
   this->addProperty("joint_space_velocity_profile", joint_space_velocity_profile).doc("selected joint space velocity profile");
   this->addProperty("max_solve_time", max_solve_time).doc("max allowable solve time for IK solution");
   this->addProperty("error", error).doc("max allowable cartesian error in IK solution");
   this->addProperty("use_joint_vel_lims", use_joint_vel_lims).doc("use velocity limits to calculate velocity profile");
   this->addProperty("use_joint_acc_lims", use_joint_acc_lims).doc("use acceleration limits to calculate velocity profile");
   this->addProperty("joint_vel_lims", joint_vel_lims).doc("maximum joint velocities");
   this->addProperty("joint_accel_lims", joint_acc_lims).doc("maximum joint accelerations");
   this->addProperty("frame_id", frame_id).doc("reference frame for trajectory waypoints");
   this->addProperty("tranform_waypoints_to_new_frame", transform_points).doc("transform waypoints to new reference frame");
}

// Configure this component.
bool ComputedTorqueJointSpace::configureHook()
{
   // Abort if any ports are not connected.
   if(!inPort_TrajectoryGoalPose.connected()){
      RTT::log(RTT::Error) << "inPort_TrajectoryGoalPose is not connected!" << RTT::endlog();
      return false;
   }
   if(!inPort_JointStates.connected()){
      RTT::log(RTT::Error) << "inPort_JointStates is not connected!" << RTT::endlog();
      return false;
   }
   if(!outPort_TorqueCommands.connected()){
      RTT::log(RTT::Error) << "outPort_TorqueCommands is not connected!" << RTT::endlog();
      return false;
   }
   // Get KDL chain.
   rtt_ros_tools::getChainFromURDF(kdl_chain, this);
   
   // Set gravity.
   grav_vec.Zero();
   grav_vec(2)=-9.81;
   
   // Resize.
   Kp.resize(kdl_chain.getNrOfJoints());
   Kd.resize(kdl_chain.getNrOfJoints());
   joint_vel_lims.resize(kdl_chain.getNrOfJoints());
   joint_acc_lims.resize(kdl_chain.getNrOfJoints());
   joint_states_msg.position.resize(kdl_chain.getNrOfJoints());
   joint_states_msg.velocity.resize(kdl_chain.getNrOfJoints());
   joint_start_pos.resize(kdl_chain.getNrOfJoints());
   joint_final_pos.resize(kdl_chain.getNrOfJoints());
   joint_states.q.resize(kdl_chain.getNrOfJoints());
   joint_states.qdot.resize(kdl_chain.getNrOfJoints());
   desired_joint_states.q.resize(kdl_chain.getNrOfJoints());
   desired_joint_states.qdot.resize(kdl_chain.getNrOfJoints());
   desired_joint_states.qdotdot.resize(kdl_chain.getNrOfJoints());
   torque_commands.resize(kdl_chain.getNrOfJoints());
   PID_commands.resize(kdl_chain.getNrOfJoints());
   dynam_commands.resize(kdl_chain.getNrOfJoints());
   mass_mat.resize(kdl_chain.getNrOfJoints());
   gravity.resize(kdl_chain.getNrOfJoints());
   coriolis.resize(kdl_chain.getNrOfJoints());
   torque_commands_msg.effort.resize(kdl_chain.getNrOfJoints());
   torque_commands.resize(kdl_chain.getNrOfJoints());

   // Parse parameters. 
   rtt_ros_tools::getJointSpacePgains(kdl_chain, Kp, this);
   rtt_ros_tools::getJointSpaceDgains(kdl_chain, Kd, this);
   rtt_ros_tools::getIKsolver(ik_solver, this);
   rtt_ros_tools::getJointStartPose(kdl_chain, joint_start_pos);
   rtt_ros_tools::getJointSpaceVelProfileName(vel_prof_name, this);
   joint_space_vel_prof::getJointSpaceVelProfile(vel_prof_name, kdl_chain, vel_prof, this);
   rtt_ros_tools::transformWaypointsToNewFrame(transform_points, this);
   if(transform_points) rtt_ros_tools::getFrameID(frame_id, this);
   
   // Create dynamics solver.
   chain_dynamic_params.reset(new KDL::ChainDynParam(kdl_chain, grav_vec));

   return true;
}

// Start this component.
bool ComputedTorqueJointSpace::startHook()
{
   current_time = 0.0;
   duration = 0.0;
   return true;
}

// Run update loop.
void ComputedTorqueJointSpace::updateHook()
{
   //Read current joint states.
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
	 ik_solver->CartToJnt(joint_states.q, trajectory_goal_frame, joint_final_pos);
      }
      // Set to start position if there are no new goal poses.
      else joint_final_pos.data = joint_start_pos.data;
      
      // Set up velocity profile.
      vel_prof->solve(joint_states.q, joint_final_pos, duration);
      
      // Reset time before beginning new trajectory.
      current_time = 0.0;
   }
   // Get mass matrix
   chain_dynamic_params->JntToMass(joint_states.q, mass_mat);
   
   // Get gravity forces.
   chain_dynamic_params->JntToGravity(joint_states.q, gravity);
   
   // Get coriolis forces.
   chain_dynamic_params->JntToCoriolis(joint_states.q, joint_states.qdot, coriolis);

   // Get desired joint positions.
   vel_prof->get_desired_joint_pos(desired_joint_states.q, current_time);
   
   // Get desired joint velocities.
   vel_prof->get_desired_joint_vel(desired_joint_states.qdot, current_time);
   
   // Get desired joint accelerations.
   vel_prof->get_desired_joint_acc(desired_joint_states.qdotdot, current_time);
   
   // Increase timer.
   current_time += this->getPeriod();

   // Solve for torque commands.
   for(int i = 0; i < desired_joint_states.q.rows(); ++i)
   {
      PID_commands(i) = (desired_joint_states.q(i) - joint_states.q(i))*Kp[i] +
      (desired_joint_states.qdot(i) - joint_states.qdot(i))*Kd[i] +
       desired_joint_states.qdotdot(i);

      dynam_commands(i) = coriolis(i)*(desired_joint_states.qdot(i) - joint_states.qdot(i)) + gravity(i);
   }
   torque_commands = mass_mat.data * PID_commands.data + dynam_commands.data;
   
   // Create torque commands message.
   for(int i=0; i<torque_commands.rows(); ++i)
      torque_commands_msg.effort[i] = torque_commands[i];

   // Write torque commands.
   outPort_TorqueCommands.write(torque_commands_msg);
}
