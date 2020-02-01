#ifndef ROTORS_CONTROL_SMC_POSITION_CONTROLLER_H
#define ROTORS_CONTROL_SMC_POSITION_CONTROLLER_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>


#include "rotors_control/common.h"
#include "rotors_control/parameters.h"

namespace rotors_control {
	static const double kDefaultTheta_p = 10;
	static const double kDefaultTheta_q = 10;

	static const Eigen::Vector3d kDefaultAlpha(10,10,10);

	static const Eigen::Vector3d kDefaultHatKp(1,1,1);
	static const Eigen::Vector3d kDefaultHatKq(1,1,1);

	static const Eigen::Vector3d kDefaultLam_p(100, 100, 600);
	static const Eigen::Vector3d kDefaultLam_q(10, 10, 60);

	static const double kDefaultVarPi = 0.1;


	class SmcPositionControllerParameters{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		SmcPositionControllerParameters()
			: theta_p_(kDefaultTheta_p),
			  theta_q_(kDefaultTheta_q),
			  alpha_(kDefaultAlpha),
			  hatKp_(kDefaultHatKp),
			  hatKq_(kDefaultHatKq),
			  lam_p_(kDefaultLam_p),
			  lam_q_(kDefaultLam_q),
			  var_pi_(kDefaultVarPi) {
			  	calculateAllocationMatrix(rotor_configuration_, &allocation_matrix_);
			  }

		Eigen::Matrix4Xd allocation_matrix_;
		Eigen::Vector3d lam_p_;
		Eigen::Vector3d lam_q_;
		Eigen::Vector3d alpha_;
		Eigen::Vector3d hatKp_;
		Eigen::Vector3d hatKq_;

		double theta_p_;
		double theta_q_;
		double var_pi_;
		RotorConfiguration rotor_configuration_;
	};


	class SmcPositionController{
	public:
		SmcPositionController();
		~SmcPositionController();

		void InitializeParameters();
		void CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const;
		// void CalculateForceVector(Eigen::Vector3d* force) const;
		void CalculateThrust(Eigen::Vector3d* thrust) const;
		void CalculateMoments(Eigen::Vector3d thrust, Eigen::Vector3d* moments) const;
		// void CalculateThrust(Eigen::Matrix3d R_W_I, Eigen::Vector3d* thrust) const;
		// void CalculateMoments(Eigen::Vector3d force, Eigen::Vector3d* moments) const;

		void SetOdometry(const EigenOdometry& odometry);
		void SetTrajectoryPoint(
		  const mav_msgs::EigenTrajectoryPoint& command_trajectory);

		SmcPositionControllerParameters controller_parameters_;
		VehicleParameters vehicle_parameters_;	

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	private:
		bool initialized_params_;
		bool controller_active_;

		Eigen::Vector3d normalized_attitude_gain_;
		Eigen::Vector3d normalized_angular_rate_gain_;
		Eigen::MatrixX4d rotor_vel_coef_;

		mav_msgs::EigenTrajectoryPoint command_trajectory_;
		EigenOdometry odometry_;
	};
}

#endif