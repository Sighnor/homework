
#ifndef ROBOTCONFIG_H_
#define ROBOTCONFIG_H_
#include <eigen3/Eigen/Dense>
using namespace Eigen;
const double PI = 3.1415926;

namespace HLRobot
{ 
	//Subproblems and Matrix
	void Paden_Kahan_1(const Eigen::Vector4d& p, const Eigen::Vector4d& q, const Eigen::Vector4d& r, const Eigen::Vector3d& omega, double& theta);
	void Paden_Kahan_2(const Eigen::Vector4d& p, const Eigen::Vector4d& q, const Eigen::Vector4d& r, const Eigen::Vector3d& omega1, const Eigen::Vector3d& omega2, double& theta1, double& theta2, const bool& config);
	void Paden_Kahan_3(const Eigen::Vector4d& p, const Eigen::Vector4d& q, const Eigen::Vector4d& r, const double& delta, const Eigen::Vector3d& omega, double& theta, const bool& config);
	Eigen::Matrix4d R_theta_omega_q(const double& theta, const Eigen::Vector3d& omega, const Eigen::Vector3d& q);
	
	//Inverse kinematics solution
	void SetRobotEndPos(double x, double y, double z, double yaw, double pitch, double roll);
	void GetJointAngles(double &angle1, double &angle2, double &angle3, double &angle4, double &angle5, double &angle6, int i);
	
	//Forward kinematics solution
	void SetRobotJoint(double angle1, double angle2, double angle3, double angle4, double angle5, double angle6);
	void GetJointEndPos(double &x, double &y, double &z, double &yaw, double &pitch, double &roll);

	//Inverse kinematics and Forward kinematics method function
	void robotBackward(const double* TransVector, bool* config, double* theta);
	void robotForward(const double* q, double* TransVector, bool* config);
}

#endif
