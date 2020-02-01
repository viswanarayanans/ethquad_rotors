#include <ros/ros.h>
#include <mav_msgs/default_topics.h>

#include "asmc_position_controller_node.h"

#include "rotors_control/parameters_ros.h"

namespace rotors_control {
	ASmcPositionControllerNode::ASmcPositionControllerNode(
		const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
	:nh_(nh), private_nh_(private_nh) {
		InitializeParams();

		cmd_mdj_traj_sub_ = nh_.subscribe(mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
			&ASmcPositionControllerNode::MdjTrajCallback, this);
		cmd_odom_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
			&ASmcPositionControllerNode::OdometryCallback, this);

		motor_vel_pub_ = nh_.advertise<mav_msgs::Actuators>(
			mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

		command_timer_ = nh_.createTimer(ros::Duration(0), &ASmcPositionControllerNode::TimedCallback, this,
			true, false);
	}

	ASmcPositionControllerNode::~ASmcPositionControllerNode() {}

	void ASmcPositionControllerNode::InitializeParams() {
		GetRosParameter(private_nh_, "theta_p/x",
                  position_controller_.controller_parameters_.theta_p_.x(),
                  &position_controller_.controller_parameters_.theta_p_.x());
		GetRosParameter(private_nh_, "theta_p/y",
                  position_controller_.controller_parameters_.theta_p_.y(),
                  &position_controller_.controller_parameters_.theta_p_.y());
		GetRosParameter(private_nh_, "theta_p/z",
                  position_controller_.controller_parameters_.theta_p_.z(),
                  &position_controller_.controller_parameters_.theta_p_.z());

		GetRosParameter(private_nh_, "theta_q/x",
                  position_controller_.controller_parameters_.theta_q_.x(),
                  &position_controller_.controller_parameters_.theta_q_.x());
		GetRosParameter(private_nh_, "theta_q/y",
                  position_controller_.controller_parameters_.theta_q_.y(),
                  &position_controller_.controller_parameters_.theta_q_.y());
		GetRosParameter(private_nh_, "theta_q/z",
                  position_controller_.controller_parameters_.theta_q_.z(),
                  &position_controller_.controller_parameters_.theta_q_.z());

		GetRosParameter(private_nh_, "alpha_p/x",
                  position_controller_.controller_parameters_.alpha_p_.x(),
                  &position_controller_.controller_parameters_.alpha_p_.x());
		GetRosParameter(private_nh_, "alpha_p/y",
                  position_controller_.controller_parameters_.alpha_p_.y(),
                  &position_controller_.controller_parameters_.alpha_p_.y());
		GetRosParameter(private_nh_, "alpha_p/z",
                  position_controller_.controller_parameters_.alpha_p_.z(),
                  &position_controller_.controller_parameters_.alpha_p_.z());

		GetRosParameter(private_nh_, "alpha_q0/x",
                  position_controller_.controller_parameters_.alpha_q0_.x(),
                  &position_controller_.controller_parameters_.alpha_q0_.x());
		GetRosParameter(private_nh_, "alpha_q0/y",
                  position_controller_.controller_parameters_.alpha_q0_.y(),
                  &position_controller_.controller_parameters_.alpha_q0_.y());
		GetRosParameter(private_nh_, "alpha_q0/z",
                  position_controller_.controller_parameters_.alpha_q0_.z(),
                  &position_controller_.controller_parameters_.alpha_q0_.z());

		GetRosParameter(private_nh_, "alpha_q1/x",
                  position_controller_.controller_parameters_.alpha_q1_.x(),
                  &position_controller_.controller_parameters_.alpha_q1_.x());
		GetRosParameter(private_nh_, "alpha_q1/y",
                  position_controller_.controller_parameters_.alpha_q1_.y(),
                  &position_controller_.controller_parameters_.alpha_q1_.y());
		GetRosParameter(private_nh_, "alpha_q1/z",
                  position_controller_.controller_parameters_.alpha_q1_.z(),
                  &position_controller_.controller_parameters_.alpha_q1_.z());

		GetRosParameter(private_nh_, "alpha_q2/x",
                  position_controller_.controller_parameters_.alpha_q2_.x(),
                  &position_controller_.controller_parameters_.alpha_q2_.x());
		GetRosParameter(private_nh_, "alpha_q2/y",
                  position_controller_.controller_parameters_.alpha_q2_.y(),
                  &position_controller_.controller_parameters_.alpha_q2_.y());
		GetRosParameter(private_nh_, "alpha_q2/z",
                  position_controller_.controller_parameters_.alpha_q2_.z(),
                  &position_controller_.controller_parameters_.alpha_q2_.z());

		GetRosParameter(private_nh_, "hatKp/x",
                  position_controller_.controller_parameters_.hatKp_.x(),
                  &position_controller_.controller_parameters_.hatKp_.x());
		GetRosParameter(private_nh_, "hatKp/y",
                  position_controller_.controller_parameters_.hatKp_.y(),
                  &position_controller_.controller_parameters_.hatKp_.y());
		GetRosParameter(private_nh_, "hatKp/z",
                  position_controller_.controller_parameters_.hatKp_.z(),
                  &position_controller_.controller_parameters_.hatKp_.z());

		GetRosParameter(private_nh_, "hatKq_0/x",
                  position_controller_.controller_parameters_.hatKq_0_.x(),
                  &position_controller_.controller_parameters_.hatKq_0_.x());
		GetRosParameter(private_nh_, "hatKq_0/y",
                  position_controller_.controller_parameters_.hatKq_0_.y(),
                  &position_controller_.controller_parameters_.hatKq_0_.y());
		GetRosParameter(private_nh_, "hatKq_0/z",
                  position_controller_.controller_parameters_.hatKq_0_.z(),
                  &position_controller_.controller_parameters_.hatKq_0_.z());

		GetRosParameter(private_nh_, "hatKq_1/x",
                  position_controller_.controller_parameters_.hatKq_1_.x(),
                  &position_controller_.controller_parameters_.hatKq_1_.x());
		GetRosParameter(private_nh_, "hatKq_1/y",
                  position_controller_.controller_parameters_.hatKq_1_.y(),
                  &position_controller_.controller_parameters_.hatKq_1_.y());
		GetRosParameter(private_nh_, "hatKq_1/z",
                  position_controller_.controller_parameters_.hatKq_1_.z(),
                  &position_controller_.controller_parameters_.hatKq_1_.z());

		GetRosParameter(private_nh_, "hatKq_2/x",
                  position_controller_.controller_parameters_.hatKq_2_.x(),
                  &position_controller_.controller_parameters_.hatKq_2_.x());
		GetRosParameter(private_nh_, "hatKq_2/y",
                  position_controller_.controller_parameters_.hatKq_2_.y(),
                  &position_controller_.controller_parameters_.hatKq_2_.y());
		GetRosParameter(private_nh_, "hatKq_2/z",
                  position_controller_.controller_parameters_.hatKq_2_.z(),
                  &position_controller_.controller_parameters_.hatKq_2_.z());

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

		GetRosParameter(private_nh_, "alphaM",
                  position_controller_.controller_parameters_.alpha_m_,
                  &position_controller_.controller_parameters_.alpha_m_);

		GetRosParameter(private_nh_, "hatM",
                  position_controller_.controller_parameters_.hatM_,
                  &position_controller_.controller_parameters_.hatM_);

		GetRosParameter(private_nh_, "var_pi_p",
                  position_controller_.controller_parameters_.var_pi_p_,
                  &position_controller_.controller_parameters_.var_pi_p_);

		GetRosParameter(private_nh_, "var_pi_q",
                  position_controller_.controller_parameters_.var_pi_q_,
                  &position_controller_.controller_parameters_.var_pi_q_);
		
		// ROS_INFO_STREAM("Rotor size:" << position_controller_.vehicle_parameters_.rotor_configuration_.rotors.size());
		GetVehicleParameters(private_nh_, &position_controller_.vehicle_parameters_);
		// ROS_INFO_STREAM("Rotor size:" << position_controller_.vehicle_parameters_.rotor_configuration_.rotors.size());

		position_controller_.InitializeParameters();
	}

	void ASmcPositionControllerNode::MdjTrajCallback(
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

	void ASmcPositionControllerNode::OdometryCallback(
		const nav_msgs::OdometryConstPtr& odometry_msg) {
		ROS_INFO_ONCE("ASmcPositionController got first odometry message.");

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

	void ASmcPositionControllerNode::TimedCallback(const ros::TimerEvent& e) {
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
  ros::init(argc, argv, "asmc_position_controller_node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  rotors_control::ASmcPositionControllerNode asmc_position_controller_node(nh, private_nh);

  ros::spin();

  return 0;
}