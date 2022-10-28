
#include "HLrobotconfig.h"
#include "eigen3/Eigen/Dense"
#include <math.h>
#include <iostream>

using namespace std;
using namespace Eigen;

const double L1 = 491;
const double L2 = 450;
const double L3 = 450;
const double L4 = 84;
Eigen::Vector3d omega1(0, 0, 1);
Eigen::Vector3d omega2(0, 1, 0);
Eigen::Vector3d omega3(0, 1, 0);
Eigen::Vector3d omega4(0, 0, 1);
Eigen::Vector3d omega5(0, 1, 0);
Eigen::Vector3d omega6(0, 0, 1);

Eigen::Vector4d q1(0, 0, L1, 1);
Eigen::Vector4d q2(0, 0, L1, 1);
Eigen::Vector4d q3(0, 0, L1 + L2, 1);
Eigen::Vector4d q4(0, 0, L1 + L2 + L3, 1);
Eigen::Vector4d q5(0, 0, L1 + L2 + L3, 1);
Eigen::Vector4d q6(0, 0, L1 + L2 + L3, 1);

namespace HLRobot
{
	//初始化TransMatrix
	double mTransMatrix[16] {0};

	void Paden_Kahan_1(const Eigen::Vector4d& p, const Eigen::Vector4d& q, const Eigen::Vector4d& r, const Eigen::Vector3d &omega,double& theta)
	{
		Eigen::Vector3d u(p.x() - r.x(), p.y() - r.y(), p.z() - r.z());
		Eigen::Vector3d v(q.x() - r.x(), q.y() - r.y(), q.z() - r.z());

		Eigen::Vector3d my_u = u - omega * omega.dot(u);
		Eigen::Vector3d my_v = v - omega * omega.dot(v);

		theta = atan2(omega.dot(my_u.cross(my_v)), my_u.dot(my_v)) / PI * 180;
	}

	void Paden_Kahan_2(const Eigen::Vector4d& p, const Eigen::Vector4d& q, const Eigen::Vector4d& r, const Eigen::Vector3d& omega1, const Eigen::Vector3d& omega2, double& theta1, double& theta2, const bool& config)
	{
		Eigen::Vector3d u(p.x() - r.x(), p.y() - r.y(), p.z() - r.z());
		Eigen::Vector3d v(q.x() - r.x(), q.y() - r.y(), q.z() - r.z());

		double my_dot = omega1.dot(omega2);
		Eigen::Vector3d my_cross = omega1.cross(omega2);

		double alpha = (my_dot * omega2.dot(u) - omega1.dot(v)) / (my_dot * my_dot - 1);
		double beta = (my_dot * omega1.dot(v) - omega2.dot(u)) / (my_dot * my_dot - 1);
		double gamma = sqrt(u.norm() * u.norm() - alpha * alpha - beta * beta - 2 * alpha * beta * my_dot) / (my_cross.norm() * my_cross.norm());

		Eigen::Vector3d z1 = alpha * omega1 + beta * omega2 + gamma * my_cross;
		Eigen::Vector3d z2 = alpha * omega1 + beta * omega2 - gamma * my_cross;

		Eigen::Vector4d c1 = Eigen::Vector4d(z1.x(), z1.y(), z1.z(), 1) + r;
		Eigen::Vector4d c2 = Eigen::Vector4d(z2.x(), z2.y(), z2.z(), 1) + r;

		if (config)
		{
			Paden_Kahan_1(c1, q, r, omega1, theta1);
			Paden_Kahan_1(p, c1, r, omega2, theta2);
		}
		else
		{
			Paden_Kahan_1(c2, q, r, omega1, theta1);
			Paden_Kahan_1(p, c2, r, omega2, theta2);
		}
	}

	void Paden_Kahan_3(const Eigen::Vector4d& p, const Eigen::Vector4d& q, const Eigen::Vector4d& r, const double& delta, const Eigen::Vector3d& omega, double& theta, const bool& config)
	{
		Eigen::Vector3d u(p.x() - r.x(), p.y() - r.y(), p.z() - r.z());
		Eigen::Vector3d v(q.x() - r.x(), q.y() - r.y(), q.z() - r.z());

		Eigen::Vector3d p_q(p.x() - q.x(), p.y() - q.y(), p.z() - q.z());
		Eigen::Vector3d my_u = u - omega * omega.dot(u);
		Eigen::Vector3d my_v = v - omega * omega.dot(v);
		double my_delta = sqrt(delta * delta - omega.dot(p_q) * omega.dot(p_q));

		double theta_0 = atan2(omega.dot(my_u.cross(my_v)), my_u.dot(my_v));
		double theta_1 = acos((my_u.norm() * my_u.norm() + my_v.norm() * my_v.norm() - my_delta * my_delta) / (2 * my_u.norm() * my_v.norm()));

		if (config)
		{
			theta = (theta_0 + theta_1) / PI * 180;
		}
		else
		{
			theta = (theta_0 - theta_1) / PI * 180;
		}
		if (theta > 180)
		{
			theta = theta - 360;
		}
	}

	Eigen::Matrix4d R_theta_omega_q(const double& theta, const Eigen::Vector3d& omega, const Eigen::Vector4d& q)
	{
		Eigen::Matrix3d eye, e_omega_theta;
		Eigen::Matrix4d output;

		Eigen::Vector3d my_q(q.x(), q.y(), q.z());
		Eigen::Vector3d velocity = -omega.cross(my_q);

		double my_theta = theta * PI / 180;
		double v = 1 - cos(my_theta);
		double c = cos(my_theta);
		double s = sin(my_theta);
		double w1 = omega.x();
		double w2 = omega.y();
		double w3 = omega.z();

		eye << 1, 0, 0,
			0, 1, 0,
			0, 0, 1;

		e_omega_theta << w1 * w1 * v + c, w1* w2* v - w3 * s, w1* w3* v + w2 * s,
			w1* w2* v + w3 * s, w2* w2* v + c, w2* w3* v - w1 * s,
			w1* w3* v - w2 * s, w2* w3* v + w1 * s, w3* w3* v + c;

		Eigen::Vector3d e_velocity_theta = (eye - e_omega_theta) * omega.cross(velocity) + omega * omega.dot(velocity) * my_theta;

		output << e_omega_theta(0, 0), e_omega_theta(0, 1), e_omega_theta(0, 2), e_velocity_theta.x(),
			e_omega_theta(1, 0), e_omega_theta(1, 1), e_omega_theta(1, 2), e_velocity_theta.y(),
			e_omega_theta(2, 0), e_omega_theta(2, 1), e_omega_theta(2, 2), e_velocity_theta.z(),
			0, 0, 0, 1;

		return output;
	}

	
	void SetRobotEndPos(double x, double y, double z, double yaw, double pitch, double roll)
	{
		double startAngle[3];

		startAngle[0] = yaw * PI / 180;
		startAngle[1] = pitch * PI / 180;
		startAngle[2] = roll * PI / 180;

		mTransMatrix[0] = cos(startAngle[0]) * cos(startAngle[1]) * cos(startAngle[2]) - sin(startAngle[0]) * sin(startAngle[2]);
		mTransMatrix[1] = -cos(startAngle[0]) * cos(startAngle[1]) * sin(startAngle[2]) - sin(startAngle[0]) * cos(startAngle[2]);
		mTransMatrix[2] = cos(startAngle[0]) * sin(startAngle[1]);
		mTransMatrix[3] = x;

		mTransMatrix[4] = sin(startAngle[0]) * cos(startAngle[1]) * cos(startAngle[2]) + cos(startAngle[0]) * sin(startAngle[2]);
		mTransMatrix[5] = -sin(startAngle[0]) * cos(startAngle[1]) * sin(startAngle[2]) + cos(startAngle[0]) * cos(startAngle[2]);
		mTransMatrix[6] = sin(startAngle[0]) * sin(startAngle[1]);
		mTransMatrix[7] = y;

		mTransMatrix[8] = -sin(startAngle[1]) * cos(startAngle[2]);
		mTransMatrix[9] = sin(startAngle[1]) * sin(startAngle[2]);
		mTransMatrix[10] = cos(startAngle[1]);
		mTransMatrix[11] = z;

		mTransMatrix[12] = 0;
		mTransMatrix[13] = 0;
		mTransMatrix[14] = 0;
		mTransMatrix[15] = 1;
	}

	void GetJointAngles(double &angle1, double &angle2, double &angle3, double &angle4, double &angle5, double &angle6, int i)
	{
        bool mConfig[3] = { false, false, false };
        if (i == 1)
        {
            mConfig[0] = true;
            mConfig[2] = true;
        }
        if (i == 2)
        {
            mConfig[0] = true;
        }
        if (i == 3)
        {
            mConfig[2] = true;
        }
		double theta[6];
		robotBackward(mTransMatrix, mConfig, theta);

		angle1 = theta[0];
		angle2 = theta[1];
		angle3 = theta[2];
		angle4 = theta[3];
		angle5 = theta[4];
		angle6 = theta[5];
	}

	void SetRobotJoint(double angle1, double angle2, double angle3, double angle4, double angle5, double angle6)
	{
		double theta[6] = { angle1, angle2, angle3, angle4, angle5, angle6 };
		bool mConfig[3] = { false, false, false };
		robotForward(theta, mTransMatrix, mConfig);
	}

	void GetJointEndPos(double &x, double &y, double &z, double &yaw, double &pitch, double &roll)
	{
		x = mTransMatrix[3];
		y = mTransMatrix[7];
		z = mTransMatrix[11];
		yaw = atan(mTransMatrix[6] / mTransMatrix[2]);
		pitch = acos(mTransMatrix[10]);
		roll = -atan(mTransMatrix[9] / mTransMatrix[8]);
	}


	/********************************************************************
	ABSTRACT:	机器人逆运动学

	INPUTS:		T[16]:	位姿矩阵，其中长度距离为米

				config[3]：姿态，六轴机器人对应有8种姿态（即对应的逆运动学8个解），为了安全，
				实验室中我们只计算一种即可。config用来作为选解的标志数。

	OUTPUTS:    theta[6] 6个关节角, 单位为弧度

	RETURN:		<none>
	***********************************************************************/
	void robotBackward(const double* TransVector, bool* mconfig, double* theta)
	{
		Eigen::Matrix4d g0, g1, g2, g3, input, output;

		g0 << -1, 0, 0, 0,
			0, -1, 0, 0,
			0, 0, 1, L1 + L2 + L3 + L4,
			0, 0, 0, 1;
		input << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;
		output << TransVector[0], TransVector[1], TransVector[2], TransVector[3],
			TransVector[4], TransVector[5], TransVector[6], TransVector[7],
			TransVector[8], TransVector[9], TransVector[10], TransVector[11],
			TransVector[12], TransVector[13], TransVector[14], TransVector[15];

		g1 = output * (g0 * input).inverse();

		Eigen::Vector4d p, q, r;

		Eigen::Matrix4d R12, R3, R45;

		double delta = (g1 * q4 - q1).norm();
		p = q4;
		q = q1;
		r = q3;

		Paden_Kahan_3(p, q, r, delta, omega3, theta[2], mconfig[0]);

		R3 = R_theta_omega_q(theta[2], omega3, q3);
		p = R3 * q4;
		q = g1 * q4;
		r = q1;

		Paden_Kahan_2(p, q, r, omega1, omega2, theta[0], theta[1], mconfig[1]);

		R12 = R_theta_omega_q(theta[0], omega1, q1) * R_theta_omega_q(theta[1], omega2, q2);
		g2 = (R12 * R3).inverse() * g1;

		p = q6 + Eigen::Vector4d(0, 0, 1, 0);
		q = g2 * p;
		r = q6;

		Paden_Kahan_2(p, q, q6, omega4, omega5, theta[3], theta[4], mconfig[2]);

		R45 = R_theta_omega_q(theta[3], omega4, q4) * R_theta_omega_q(theta[4], omega5, q5);
		g3 = R45.inverse() * g2;

		p = q6 + Eigen::Vector4d(1, 0, 0, 0);
		q = g3 * p;
		r = q6;

		Paden_Kahan_1(p, q, r, omega6, theta[5]);
	}

	/********************************************************************
	ABSTRACT:	机器人正运动学
	
	INPUTS:		q[6]: 6个关节角, 单位为弧度
	
	OUTPUTS:	config[3]：姿态，六轴机器人对应有8种姿态，为了安全，
				实验室中我们只计算一种即可。config用来作为选解的标志数。

				TransVector[16] : 刚体变换矩阵，也就是末端的位姿描述，其中长度距离为米
	
	RETURN:		<none>
	***********************************************************************/
	void robotForward(const double* q, double* TransVector, bool* mconfig)
	{
		Eigen::Matrix4d g0, input, output;
		g0 << -1, 0, 0, 0,
			0, -1, 0, 0,
			0, 0, 1, L1 + L2 + L3 + L4,
			0, 0, 0, 1;
		input << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;

		Eigen::Matrix4d R = R_theta_omega_q(q[0], omega1, q1) * R_theta_omega_q(q[1], omega2, q2) * R_theta_omega_q(q[2], omega3, q3) * R_theta_omega_q(q[3], omega4, q4) * R_theta_omega_q(q[4], omega5, q5) * R_theta_omega_q(q[5], omega6, q6);

		output = R * g0 * input;

		for (int i = 0;i < 4;i++)
		{
			for (int j = 0;j < 4;j++)
			{
				TransVector[4 * i + j] = output(i, j);
			}
		}
	}

}
