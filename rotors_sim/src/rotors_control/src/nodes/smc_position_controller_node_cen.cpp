#include <ros/ros.h>
#include <mav_msgs/default_topics.h>

#include "smc_position_controller_node.h"

#include "rotors_control/parameters_ros.h"

namespace rotors_control {
	SmcPositionControllerNode::SmcPositionControllerNode(
		const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
	:nh_(nh), private_nh_(private_nh) {
		InitializeParams();

		cmd_mdj_traj_sub_ = nh_.subscribe(mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
			&SmcPositionControllerNode::MdjTrajCallback, this);
		cmd_odom_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
			&SmcPositionControllerNode::OdometryCallback, this);

		motor_vel_pub_ = nh_.advertise<mav_msgs::Actuators>(
			mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

		command_timer_ = nh_.createTimer(ros::Duration(0), &SmcPositionControllerNode::TimedCallback, this,
			true, false);
	}

	SmcPositionControllerNode::~SmcPositionControllerNode() {}

	void SmcPositionControllerNode::InitializeParams() {
		GetRosParameter(private_nh_, "theta_p",
                  position_controller_.controller_parameters_.theta_p_,
                  &position_controller_.controller_parameters_.theta_p_);
		GetRosParameter(private_nh_, "theta_q",
                  position_controller_.controller_parameters_.theta_q_,
                  &position_controller_.controller_parameters_.theta_q_);

		GetRosParameter(private_nh_, "alpha/0",
                  position_controller_.controller_parameters_.alpha_.x(),
                  &position_controller_.controller_parameters_.alpha_.x());
		GetRosParameter(private_nh_, "alpha/1",
                  position_controller_.controller_parameters_.alpha_.y(),
                  &position_controller_.controller_parameters_.alpha_.y());
		GetRosParameter(private_nh_, "alpha/2",
                  position_controller_.controller_parameters_.alpha_.z(),
                  &position_controller_.controller_parameters_.alpha_.z());

		GetRosParameter(private_nh_, "hatKp/0",
                  position_controller_.controller_parameters_.hatKp_.x(),
                  &position_controller_.controller_parameters_.hatKp_.x());
		GetRosParameter(private_nh_, "hatKp/1",
                  position_controller_.controller_parameters_.hatKp_.y(),
                  &position_controller_.controller_parameters_.hatKp_.y());
		GetRosParameter(private_nh_, "hatKp/2",
                  position_controller_.controller_parameters_.hatKq_.z(),
                  &position_controller_.controller_parameters_.hatKq_.z());

		GetRosParameter(private_nh_, "hatKq/0",
                  position_controller_.controller_parameters_.hatKq_.x(),
                  &position_controller_.controller_parameters_.hatKq_.x());
		GetRosParameter(private_nh_, "hatKq/1",
                  position_controller_.controller_parameters_.hatKq_.y(),
                  &position_controller_.controller_parameters_.hatKq_.y());
		GetRosParameter(private_nh_, "hatKq/2",
                  position_controller_.controller_parameters_.hatKq_.z(),
                  &position_controller_.controller_parameters_.hatKq_.z());

		GetRosParameter(private_nh_, "lam_p/x",
                  position_controller_.controller_parameters_.lam_p_.x(),
                  &position_controller_.controller_parameters_.lam_p_.x());
		GetRosParameter(private_nh_, "lam_p/y",
                  position_controller_.controller_parameters_.lam_p_.y(),
                  &position_controller_.controller_parameters_.lam_p_.y());
		GetRosParameter(private_nh_, "lam_p/z",
                  position_controller_.controller_parameters_.lam_p_.z(),
                  &position_controller_.controller_parameters_.lam_p_.z());

		GetRosParameter(private_nh_, "lam_q/x",
                  position_controller_.controller_parameters_.lam_q_.x(),
                  &position_controller_.controller_parameters_.lam_q_.x());
		GetRosParameter(private_nh_, "lam_q/y",
                  position_controller_.controller_parameters_.lam_q_.y(),
                  &position_controller_.controller_parameters_.lam_q_.y());
		GetRosParameter(private_nh_, "lam_q/z",
                  position_controller_.controller_parameters_.lam_q_.z(),
                  &position_controller_.controller_parameters_.lam_q_.z());

		GetRosParameter(private_nh_, "var_pi",
                  position_controller_.controller_parameters_.var_pi_,
                  &position_controller_.controller_parameters_.var_pi_);
		
		// ROS_INFO_STREAM("Rotor size:" << position_controller_.vehicle_parameters_.rotor_configuration_.rotors.size());
		GetVehicleParameters(private_nh_, &position_controller_.vehicle_parameters_);
		// ROS_INFO_STREAM("Rotor size:" << position_controller_.vehicle_parameters_.rotor_configuration_.rotors.size());

		position_controller_.InitializeParameters();
	}

	void SmcPositionControllerNode::MdjTrajCallback(
		const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {

		// Clear all pending commands.
		command_timer_.stop();
		commands_.clear();
		command_waiting_times_.clear();

		const size_t n_commands = msg->points.size();

		if(n_commands < 1){
		  ROS_WARN_STREAM("Got MultiDOFJointTrajectory message, but message has no points.");
		  return;
		}	


		mav_msgs::EigenTrajectoryPoint eigen_reference;
		mav_msgs::eigenTrajectoryPointFromMsg(msg->points.front(), &eigen_reference);
		commands_.push_front(eigen_reference);	

		for (size_t i = 1; i < n_commands; ++i) {
		  const trajectory_msgs::MultiDOFJointTrajectoryPoint& reference_before = msg->points[i-1];
		  const trajectory_msgs::MultiDOFJointTrajectoryPoint& current_reference = msg->points[i];	
		  mav_msgs::eigenTrajectoryPointFromMsg(current_reference, &eigen_reference);	
		  commands_.push_back(eigen_reference);
		  command_waiting_times_.push_back(current_reference.time_from_start - reference_before.time_from_start);
		}	

		// We can trigger the first command immediately.
		position_controller_.SetTrajectoryPoint(commands_.front());
		commands_.pop_front();	
		if (n_commands > 1) {
		  command_timer_.setPeriod(command_waiting_times_.front());
		  command_waiting_times_.pop_front();
		  command_timer_.start();
		}
	}

	void SmcPositionControllerNode::OdometryCallback(
		const nav_msgs::OdometryConstPtr& odometry_msg) {
		ROS_INFO_ONCE("SmcPositionController got first odometry message.");

		EigenOdometry odometry;
		eigenOdometryFromMsg(odometry_msg, &odometry);
		position_controller_.SetOdometry(odometry);

		Eigen::VectorXd ref_rotor_velocities;
		position_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

		// Todo(ffurrer): Do this in the conversions header.
		mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

		actuator_msg->angular_velocities.clear();
		for (int i = 0; i < ref_rotor_velocities.size(); i++)
		  actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
		actuator_msg->header.stamp = odometry_msg->header.stamp;

		motor_vel_pub_.publish(actuator_msg);

	}

	void SmcPositionControllerNode::TimedCallback(const ros::TimerEvent& e) {
		if(commands_.empty()){
		  ROS_WARN("Commands empty, this should not happen here");
		  return;
		}
		
		const mav_msgs::EigenTrajectoryPoint eigen_reference = commands_.front();
		position_controller_.SetTrajectoryPoint(commands_.front());
		commands_.pop_front();
		command_timer_.stop();

		if(!command_waiting_times_.empty()){
		  command_timer_.setPeriod(command_waiting_times_.front());
		  command_waiting_times_.pop_front();
		  command_timer_.start();
		}
	}



}

int main(int argc, char** argv) {
  ros::init(argc, argv, "smc_position_controller_node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  rotors_control::SmcPositionControllerNode smc_position_controller_node(nh, private_nh);

  ros::spin();

  return 0;
}