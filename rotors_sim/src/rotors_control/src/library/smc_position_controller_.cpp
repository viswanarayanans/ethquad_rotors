#include "rotors_control/smc_position_controller.h"


namespace rotors_control{
	SmcPositionController::SmcPositionController() {}
	SmcPositionController::~SmcPositionController() {}
	void SmcPositionController::InitializeParameters() {
	calculateAllocationMatrix(vehicle_parameters_.rotor_configuration_, &(controller_parameters_.allocation_matrix_));


  	rotor_vel_coef_.resize(vehicle_parameters_.rotor_configuration_.rotors.size(), 4);
	// Calculate the pseude-inverse A^{ \dagger} and then multiply by the inertia matrix I.
	// A^{ \dagger} = A^T*(A*A^T)^{-1}
	rotor_vel_coef_ = controller_parameters_.allocation_matrix_.transpose()
	    * (controller_parameters_.allocation_matrix_
	    * controller_parameters_.allocation_matrix_.transpose()).inverse();

    // ROS_INFO_STREAM("Allocation matrix:\n" << controller_parameters_.allocation_matrix_);
	initialized_params_ = true;
	}

	void SmcPositionController::CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const {
		assert(rotor_velocities);
	  	assert(initialized_params_);
	  	// ROS_INFO("Calculating Rotor Velocities");

		rotor_velocities->resize(vehicle_parameters_.rotor_configuration_.rotors.size());
		// Return 0 velocities on all rotors, until the first command is received.
		if (!controller_active_) {
		  *rotor_velocities = Eigen::VectorXd::Zero(rotor_velocities->rows());
	  		// ROS_INFO_STREAM("Rotor velocities calculated: " << rotor_velocities->size());
		  return;
		}
		
		const Eigen::Matrix3d R_W_I = odometry_.orientation.toRotationMatrix();

	  	// ROS_INFO("Controller Active");
	  	Eigen::Vector3d command_angles;
		// mav_msgs::getEulerAnglesFromQuaternion(command_trajectory_.orientation, &command_angles);

		Eigen::Vector3d force;
		CalculateThrust(R_W_I, &force);
		// force_W = R_W_I.transpose()*force;

		double thrust;
		thrust = force.dot(R_W_I.transpose().col(2));

	  	// ROS_INFO("Thrust calculated");

		Eigen::Vector3d moments;
		CalculateMoments(force, &moments);

		Eigen::Vector4d moment_thrust;
	  	moment_thrust.block<3, 1>(0, 0) = moments;
	  	moment_thrust(3) = thrust;
	  	// ROS_INFO("Moments and thrust combined");

	  	*rotor_velocities = rotor_vel_coef_ * moment_thrust;
	  	*rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(rotor_velocities->rows()));
	  	*rotor_velocities = rotor_velocities->cwiseSqrt();

	  	// ROS_INFO_STREAM("Rotor velocities calculated: " << rotor_velocities->size());




	  // Eigen::Vector3d acceleration;
	  // ComputeDesiredAcceleration(&acceleration);

	  // Eigen::Vector3d angular_acceleration;
	  // ComputeDesiredAngularAcc(acceleration, &angular_acceleration);

	  // // Project thrust onto body z axis.
	  // double thrust = -vehicle_parameters_.mass_ * acceleration.dot(odometry_.orientation.toRotationMatrix().col(2));

	  // Eigen::Vector4d angular_acceleration_thrust;
	  // angular_acceleration_thrust.block<3, 1>(0, 0) = angular_acceleration;
	  // angular_acceleration_thrust(3) = thrust;

	  // *rotor_velocities = angular_acc_to_rotor_velocities_ * angular_acceleration_thrust;
	  // *rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(rotor_velocities->rows()));
	  // *rotor_velocities = rotor_velocities->cwiseSqrt();
	}

	void SmcPositionController::SetOdometry(const EigenOdometry& odometry) {
	  odometry_ = odometry;
	}

	void SmcPositionController::SetTrajectoryPoint(
	    const mav_msgs::EigenTrajectoryPoint& command_trajectory) {
	  command_trajectory_ = command_trajectory;
	  controller_active_ = true;
	}

	void SmcPositionController::CalculateThrust(Eigen::Matrix3d R_W_I, Eigen::Vector3d* thrust) const {
		assert(thrust);

	  	static ros::Time last_time = ros::Time::now();
	  	ros::Time current_time;

		Eigen::Vector3d position_error;
	  	position_error = odometry_.position - command_trajectory_.position_W;

		// Convert velocity in world frame
		Eigen::Vector3d velocity_W =  R_W_I * odometry_.velocity;
		Eigen::Vector3d velocity_error;
		velocity_error = velocity_W - command_trajectory_.velocity_W;

		Eigen::VectorXd xp(6);
		xp << position_error, velocity_error;

		Eigen::Vector3d sp;
		sp = velocity_error + controller_parameters_.theta_p_*position_error;

		Eigen::Vector3d dot_hatKp;
		double xp_norm = xp.norm();
		double sp_norm = sp.norm();

		static Eigen::Vector3d hatKp = controller_parameters_.hatKp_;

		dot_hatKp[0] = sp_norm - controller_parameters_.alpha_[0]*hatKp[0];
		dot_hatKp[1] = sp_norm*xp_norm - controller_parameters_.alpha_[1]*hatKp[1];
		dot_hatKp[2] = sp_norm*pow(xp_norm,2) - controller_parameters_.alpha_[2]*hatKp[2];

		current_time = ros::Time::now();
		double dt = (current_time - last_time).toSec();
		
		hatKp += dot_hatKp*dt;
		last_time = current_time;

		Eigen::Matrix3d lam_p = controller_parameters_.lam_p_.asDiagonal();
	 	double rho_p = hatKp[0] + hatKp[1]*xp_norm + hatKp[2]*pow(xp_norm,2);

	 	Eigen::Vector3d delTau_p;
	 	if(sp_norm >= controller_parameters_.var_pi_) {
	 		delTau_p = rho_p*(sp/sp_norm);
	 	}
	 	else{
	 		delTau_p = rho_p*(sp/controller_parameters_.var_pi_);
	 	}


	 	Eigen::Vector3d tau_p = -lam_p*sp - delTau_p;
	 	// Eigen::Vector3d tau_p = -lam_p*sp ;
	 	*thrust = tau_p;
	}

	void SmcPositionController::CalculateMoments(Eigen::Vector3d force, Eigen::Vector3d* moments) const {
		assert(moments);

	  	static ros::Time last_time = ros::Time::now();
	  	ros::Time current_time;

  		Eigen::Matrix3d R = odometry_.orientation.toRotationMatrix();
		Eigen::Vector3d b1_des;
		double yaw = command_trajectory_.getYaw();
		b1_des << cos(yaw), sin(yaw), 0;

		Eigen::Vector3d b3_des;
		b3_des = force / force.norm();

		Eigen::Vector3d b2_des;
		b2_des = b3_des.cross(b1_des);
		b2_des.normalize();

		Eigen::Matrix3d R_des;
		R_des.col(0) = b2_des.cross(b3_des);
		R_des.col(1) = b2_des;
		R_des.col(2) = b3_des;

		// Angle error according to lee et al.
		Eigen::Matrix3d angle_error_matrix = 0.5 * (R_des.transpose() * R - R.transpose() * R_des);
		Eigen::Vector3d angle_error;
		vectorFromSkewMatrix(angle_error_matrix, &angle_error);

		// TODO(burrimi) include angular rate references at some point.
		Eigen::Vector3d angular_rate_des(Eigen::Vector3d::Zero());
		angular_rate_des[2] = command_trajectory_.getYawRate();

		Eigen::Vector3d angular_rate_error = odometry_.angular_velocity - R_des.transpose() * R * angular_rate_des;
		// Eigen::Vector3d angular_rate_error = odometry_.angular_velocity - angular_rate_des;

		Eigen::VectorXd xq(6);
		xq << angle_error, angular_rate_error;

		Eigen::Vector3d sq;
		sq = angular_rate_error + controller_parameters_.theta_q_*angle_error;

		Eigen::Vector3d dot_hatKq;
		double xq_norm = xq.norm();
		double sq_norm = sq.norm();

		static Eigen::Vector3d hatKq = controller_parameters_.hatKq_;

		dot_hatKq[0] = sq_norm - controller_parameters_.alpha_[0]*hatKq[0];
		dot_hatKq[1] = sq_norm*xq_norm - controller_parameters_.alpha_[1]*hatKq[1];
		dot_hatKq[2] = sq_norm*pow(xq_norm,2) - controller_parameters_.alpha_[2]*hatKq[2];

		current_time = ros::Time::now();
		double dt = (current_time - last_time).toSec();
		
		hatKq += dot_hatKq*dt;
		last_time = current_time;
		// hatKq += dot_hatKq;		

		Eigen::Matrix3d lam_q = controller_parameters_.lam_q_.asDiagonal();
	 	double rho_q = hatKq[0] + hatKq[1]*xq_norm + hatKq[2]*pow(xq_norm,2);

	 	Eigen::Vector3d delTau_q;
	 	if(sq_norm >= controller_parameters_.var_pi_) {
	 		delTau_q = rho_q*(sq/sq_norm);
	 	}
	 	else{
	 		delTau_q = rho_q*(sq/controller_parameters_.var_pi_);
	 	}

	 	Eigen::Vector3d tau_q = -lam_q*sq - delTau_q;
	 	// Eigen::Vector3d tau_q = -lam_q*sq ;
	 	*moments = tau_q;
	}

	// void SmcPositionController::CalculateMoments(Eigen::Vector3d command_angles, Eigen::Vector3d* moments) const {
	// 	assert(moments);

	// 	static ros::Time last_time = ros::Time::now();
	//   	ros::Time current_time;

	// 	Eigen::Vector3d odometry_angles;
	// 	mav_msgs::getEulerAnglesFromQuaternion(odometry_.orientation, &odometry_angles);
		
	// 	Eigen::Vector3d angle_error;
	// 	// Eigen::Vector3d command_angles(0,0,0);
	// 	// mav_msgs::getEulerAnglesFromQuaternion(command_trajectory_.orientation, &command_angles);

	// 	angle_error = odometry_angles - command_angles;

	// 	Eigen::Vector3d ang_vel_error;
	// 	Eigen::Vector3d command_velocity(0,0,0);
	// 	ang_vel_error = odometry_.angular_velocity - command_velocity; // Odometry is in body frame. It needs to be changed to world frame in case trajectory is given for angles.

	// 	Eigen::VectorXd xq(6);
	// 	xq << angle_error, ang_vel_error;

	// 	Eigen::Vector3d sq;
	// 	sq = angle_error + controller_parameters_.theta_q_*ang_vel_error;

	// 	Eigen::Vector3d dot_hatKq;
	// 	double xq_norm = xq.norm();
	// 	double sq_norm = sq.norm();

	// 	static Eigen::Vector3d hatKq = controller_parameters_.hatKq_;

	// 	dot_hatKq[0] = sq_norm - controller_parameters_.alpha_[0]*hatKq[0];
	// 	dot_hatKq[1] = sq_norm*xq_norm - controller_parameters_.alpha_[1]*hatKq[1];
	// 	dot_hatKq[2] = sq_norm*pow(xq_norm,2) - controller_parameters_.alpha_[2]*hatKq[2];

	// 	current_time = ros::Time::now();
	// 	double dt = (current_time - last_time).toSec();
		
	// 	hatKq = dot_hatKq*dt;
	// 	last_time = current_time;
	// 	// hatKq += dot_hatKq;		

	// 	Eigen::Matrix3d lam_q = controller_parameters_.lam_q_.asDiagonal();
	//  	double rho_q = hatKq[0] + hatKq[1]*xq.norm() + hatKq[2]*pow(xq.norm(),2);

	//  	Eigen::Vector3d delTau_q;
	//  	if(sq_norm >= controller_parameters_.var_pi_) {
	//  		delTau_q = rho_q*(sq/sq_norm);
	//  	}
	//  	else{
	//  		delTau_q = rho_q*(sq/controller_parameters_.var_pi_);
	//  	}

	//  	Eigen::Vector3d tau_q = -lam_q*sq - delTau_q;
	//  	*moments = tau_q;
	// }


}
int main(void) {
	ROS_INFO("Nothing");
	return 0;
}