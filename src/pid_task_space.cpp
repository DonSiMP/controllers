#include <controllers/pid_task_space.h>

// Create a new instance of PIDtaskSpace. The PreOperational argument forces you to call configureHook().
PIDtaskSpace::PIDtaskSpace(const std::string& name) : RTT::TaskContext(name, PreOperational)
{
   // Add RTT ports.
   this->addPort("torque_commands", outPort_TorqueCommands);
   this->addPort("start_pose_to_trajec_gen", outPort_TrajectoryGenerator);
   this->addPort("current_joint_states", inPort_CurrentJointStates);
   this->addPort("desired_trajectory_pos", inPort_DesiredTaskPos);
   this->addPort("desired_trajectory_vel", inPort_DesiredTaskVel);

   // Add RTT properties.
   this->addProperty("task_P_gains", Kp).doc("propertional gains");
   this->addProperty("task_I_gains", Ki).doc("integral gains");
   this->addProperty("task_D_gains", Kd).doc("derivative gians");
   this->addProperty("robot_description", robot_description).doc("URDF or SDF");
   this->addProperty("root_link",root_link).doc("first link in KDL chain");
   this->addProperty("tip_link",tip_link).doc("last link in KDL chain");
   this->addProperty("end_effector_frame_name", ee_frame_name).doc("name of end effector frame");
   this->addProperty("joint_start_pose", joint_start_pos).doc("array of joint start positions");
}

// Configure this component.
bool PIDtaskSpace::configureHook()
{
   // Abort if any ports are not connected.
   if(!inPort_CurrentJointStates.connected()){
      RTT::log(RTT::Error) << "inPort_CurrentJointStates is not connected!" << RTT::endlog();
      return false;
   }
   if(!inPort_DesiredTaskPos.connected()){
      RTT::log(RTT::Error) << "inPort_DesiredTaskPos is not connected!" << RTT::endlog();
      return false;
   }
   if(!inPort_DesiredTaskVel.connected()){
      RTT::log(RTT::Error) << "inPort_DesiredTaskVel is not connected!" << RTT::endlog();
      return false;
   }
   if(!outPort_TorqueCommands.connected()){
      RTT::log(RTT::Error) << "outPort_TorqueCommands is not connected!" << RTT::endlog();
      return false;
   }
   if(!outPort_TrajectoryGenerator.connected()){
      RTT::log(RTT::Error) << "outPort_TrajectoryGenerator is not connected!" << RTT::endlog();
      return false;
   }
   // Parse parameters.
   rtt_ros_tools::getChainFromURDF(kdl_chain, this);
   rtt_ros_tools::getTaskSpacePgains(Kp, this);
   rtt_ros_tools::getTaskSpaceIgains(Ki, this);
   rtt_ros_tools::getTaskSpaceDgains(Kd, this);
   rtt_ros_tools::getJointStartPose(kdl_chain, joint_start_pos);
   rtt_ros_tools::getEndEffectorFrameName(ee_frame_name, this);
   rtt_ros_tools::getSegmentIndex(ee_frame_index, ee_frame_name, kdl_chain);

   // Create solvers.
   fk_pos_solver.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain));
   fk_vel_solver.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain));
   jacobian_solver.reset(new KDL::ChainJntToJacSolver(kdl_chain));

   // Find end effector start pose from joint start pose array.
   fk_pos_solver->JntToCart(joint_start_pos, ee_start_pose_frame, ee_frame_index);

   // Convert KDL frame to pose message.
   tf::poseKDLToMsg(ee_start_pose_frame, ee_start_pose_msg);

   // Push pose message into pose array for port compatibility.
   ee_start_pose_to_port.poses.push_back(ee_start_pose_msg);

   // Resize.
   Kp.resize(kdl_chain.getNrOfJoints());
   Ki.resize(kdl_chain.getNrOfJoints());
   Kd.resize(kdl_chain.getNrOfJoints());
   joint_states_msg.position.resize(kdl_chain.getNrOfJoints());
   joint_states_msg.velocity.resize(kdl_chain.getNrOfJoints());
   joint_states.q.resize(kdl_chain.getNrOfJoints());
   joint_states.qdot.resize(kdl_chain.getNrOfJoints());
   joint_start_pos.resize(kdl_chain.getNrOfJoints());
   torque_commands_msg.effort.resize(kdl_chain.getNrOfJoints());
   torque_commands.resize(kdl_chain.getNrOfJoints());
   jacobian.resize(kdl_chain.getNrOfJoints());
   jacobian_transpose(jacobian.rows(), jacobian.columns());

   return true;
}

// Start this component.
bool PIDtaskSpace::startHook()
{
   return true;
}

// Run update loop.
void PIDtaskSpace::updateHook()
{
   // If ports have no data, set the robot to its starting position.
   if(inPort_DesiredTaskPos.read(desired_task_pos_msg) == RTT::NewData &&
      inPort_DesiredTaskVel.read(desired_task_vel_msg) == RTT::NewData){

      tf::poseMsgToKDL(desired_task_pos_msg, desired_task_pos);
      tf::twistMsgToKDL(desired_task_vel_msg, desired_task_vel);
   }
   // Use start position to generate trajectory.
   else outPort_TrajectoryGenerator.write(ee_start_pose_to_port);
   
   // Read current joint states from robot or Gazebo.
   inPort_CurrentJointStates.read(joint_states_msg);

   // Move data from messages to JntArrayVel for compatibility with fk velocity solvers.
   joint_states.q.data = Eigen::VectorXd::Map(joint_states_msg.position.data(), joint_states_msg.position.size());
   joint_states.qdot.data = Eigen::VectorXd::Map(joint_states_msg.velocity.data(), joint_states_msg.velocity.size());

   // Get current end effector pose.
   fk_pos_solver->JntToCart(joint_states.q, current_task_pos, ee_frame_index);

   // Solve for current end effector velocity.
   fk_vel_solver->JntToCart(joint_states, current_task_vel_solver, ee_frame_index);

   // Get current end effector velocity.
   current_task_vel = current_task_vel_solver.GetTwist();

   // Get current position error.
   pose_error = diff(desired_task_pos, current_task_pos);

   // Make a vector of PID commands.
   for(int i = 0; i < 6; ++i)
      PID_commands(i) = pose_error(i)*Kp[i] + pose_error(i)*Ki[i] + (desired_task_vel(i) - current_task_vel(i))*Kd[i];

   // Convert KDL::Twist to Eigen::Matrix so we can use matrix multiplications.
   tf::twistKDLToEigen(PID_commands, pid_commands);

   // Get jacobian.
   jacobian_solver->JntToJac(joint_states.q, jacobian, ee_frame_index);

   // Convert jacobian to Eigen::Matrix so transpose operation can be used.
   jacobian_transpose = jacobian.data;

   // transpose jacobian.
   jacobian_transpose.transposeInPlace();

   // Get torque commands.
   torque_commands = jacobian_transpose * pid_commands;

   for(int i=0; i<torque_commands.rows(); ++i)
      torque_commands_msg.effort[i] = torque_commands[i];
   
   torque_commands_msg.header.stamp = rtt_rosclock::rtt_now();

   // Write torque commands to port.
   outPort_TorqueCommands.write(torque_commands_msg);
}
