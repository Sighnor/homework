#include <algorithm>
#include <Eigen/Eigen>
#include <iostream>
#include <math.h>
#include <fstream>

const double PI = 3.1415926;
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

double mTransMatrix[16]{ 0 };
bool mConfig[3] = {true, false, false};
std::vector<double> mJointAngle;			//起始点位的关节角度,单位度
std::vector<double> mJointCoord;
double mSampleTime;							//采样点位，单位S
double mVel;								//速度，单位m/s
double mAcc;								//加速度，单位m/s/s
double mDec;								//减速度，单位m / s / s

void Paden_Kahan_1(const Eigen::Vector4d& p, const Eigen::Vector4d& q, const Eigen::Vector4d& r, const Eigen::Vector3d &omega,double& theta)
{
	Eigen::Vector3d u(p.x() - r.x(), p.y() - r.y(), p.z() - r.z());
	Eigen::Vector3d v(q.x() - r.x(), q.y() - r.y(), q.z() - r.z());

	Eigen::Vector3d my_u = u - omega * omega.dot(u);
	Eigen::Vector3d my_v = v - omega * omega.dot(v);

	theta = atan2(omega.dot(my_u.cross(my_v)),my_u.dot(my_v)) / PI * 180;
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
		if(theta > 180)
		{
			theta = theta - 360;
		}
	}
	else
	{
		theta = (theta_0 - theta_1) / PI * 180;
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

	e_omega_theta << w1 * w1 * v + c, w1 * w2 * v - w3 * s, w1 * w3 * v + w2 * s,
						w1 * w2 * v + w3 * s, w2 * w2 * v + c, w2 * w3 * v - w1 * s,
						w1 * w3 * v - w2 * s, w2 * w3 * v + w1 * s, w3 * w3 * v + c;

	Eigen::Vector3d e_velocity_theta = (eye  - e_omega_theta) * omega.cross(velocity) + omega * omega.dot(velocity) * my_theta;

	output << e_omega_theta(0,0), e_omega_theta(0,1), e_omega_theta(0,2), e_velocity_theta.x(),
			  e_omega_theta(1,0), e_omega_theta(1,1), e_omega_theta(1,2), e_velocity_theta.y(),
			  e_omega_theta(2,0), e_omega_theta(2,1), e_omega_theta(2,2), e_velocity_theta.z(),
				0, 0, 0, 1;
	
	return output;
}

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

	Eigen::Vector4d p,q,r;

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

	Paden_Kahan_2(p, q, q6, omega4, omega5, theta[3] ,theta[4], mconfig[2]);
	
	R45 = R_theta_omega_q(theta[3], omega4, q4) * R_theta_omega_q(theta[4], omega5, q5);
	g3 = R45.inverse() * g2;

	p = q6 + Eigen::Vector4d(1, 0, 0 ,0);
	q = g3 * p;
	r = q6;

	Paden_Kahan_1(p, q, r, omega6, theta[5]);
}

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

	Eigen::Matrix4d R = R_theta_omega_q(q[0], omega1, q1) * R_theta_omega_q(q[1], omega2, q2) * R_theta_omega_q(q[2], omega3, q3) * 
						R_theta_omega_q(q[3], omega4, q4) * R_theta_omega_q(q[4], omega5, q5) * R_theta_omega_q(q[5], omega6, q6);

	output = R * g0 * input;

	std::cout << output << std::endl;

	for(int i = 0;i < 4;i++)
	{
		for(int j = 0;j < 4;j++)
		{
			TransVector[4 * i + j] = output(i,j);
		}
	}
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

void GetJointAngles(double& angle1, double& angle2, double& angle3, double& angle4, double& angle5, double& angle6)
{
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
	bool config[3] = { false, false, false };
	robotForward(theta, mTransMatrix, config);
}

void GetJointEndPos(double& x, double& y, double& z, double& yaw, double& pitch, double& roll)
{
	x = mTransMatrix[3];
	y = mTransMatrix[7];
	z = mTransMatrix[11];
	yaw = atan(mTransMatrix[6] / mTransMatrix[2]) * 180 / PI;
	pitch = acos(mTransMatrix[10]) * 180 / PI;
	roll = atan(-mTransMatrix[9] / mTransMatrix[8]) * 180 / PI;
}

void SetPlanAngles()
{
	double temp_angle;
    std::ifstream angle;
    angle.open("angle.txt");

    while (angle >> temp_angle)
    {
        mJointAngle.push_back(temp_angle);
    }

	for(auto &temp : mJointAngle)
	{
		temp = temp + 180;
	}

    //std::cout << mJointAngle.size() << std::endl;

	angle.close();
}

void SetPlanPoints()
{	
	double temp_pos;
    std::ifstream pos;
    pos.open("pos.txt");

    while (pos >> temp_pos)
    {
        mJointCoord.push_back(temp_pos);
    }

    std::cout << mJointCoord.size() << std::endl;

	pos.close();

	for(int i = 0;i < mJointCoord.size() / 6;i++)
	{
		double angle1, angle2, angle3, angle4, angle5, angle6;
		SetRobotEndPos(mJointCoord[6 * i + 0], mJointCoord[6 * i + 1], mJointCoord[6 * i + 2], mJointCoord[6 * i + 3], mJointCoord[6 * i + 4], mJointCoord[6 * i + 5]);
		GetJointAngles(angle1, angle2, angle3, angle4, angle5, angle6);

		mJointAngle.push_back(angle1);
		mJointAngle.push_back(angle2);
		mJointAngle.push_back(angle3);
		mJointAngle.push_back(angle4);
		mJointAngle.push_back(angle5);
		mJointAngle.push_back(angle6); 

		std::cout << angle1 << ',' << angle2 << ',' << angle3 << ',' << angle4 << ',' << angle5 << ',' << angle6 << '\n';
	}
}

void GetPlanPoints()
{
	std::ofstream outfile;               			//创建文件
	outfile.open("data.txt");

	int count = 0;

	int num_points = mJointAngle.size() / 6;

	double mSampleTime = 0.05;

	std::vector<double> td;
	double temp_time;
	std::ifstream time;
	time.open("time.txt");

	while (time >> temp_time)
	{
		td.push_back(temp_time);
	}

	time.close();

	std::vector<double> acc_value = { 18, 400, 180, 2, 150, 22 };
	std::vector<double> t(num_points);

	std::vector<double> tb(num_points * 6);
	std::vector<double> acc(num_points * 6);
	std::vector<double> velocity_line(num_points * 6);

	t[0] = 0;
	for (int i = 1;i < num_points;i++)
	{
		t[i] = t[i - 1] + td[i - 1];
	}
	int length = t[num_points - 1] / mSampleTime;

	std::vector<double> my_angle(length * 6);

	for (int j = 0;j < 6;j++)
	{
		//初始加速度和结束加速度的方向
		if (mJointAngle[1 * 6 + j] == mJointAngle[0 * 6 + j])
		{
			acc[0 * 6 + j] = 0;
			tb[0 * 6 + j] = 0;
		}
		else
		{
			acc[0 * 6 + j] = (mJointAngle[1 * 6 + j] - mJointAngle[0 * 6 + j]) / abs(mJointAngle[1 * 6 + j] - mJointAngle[0 * 6 + j]) * acc_value[j];
			tb[0 * 6 + j] = td[0] - sqrt(td[0] * td[0] - 2 * (mJointAngle[1 * 6 + j] - mJointAngle[0 * 6 + j]) / acc[0 * 6 + j]);
		}
		if (mJointAngle[(num_points - 2) * 6 + j] == mJointAngle[(num_points - 1) * 6 + j])
		{
			acc[(num_points - 1) * 6 + j] = 0;
			tb[(num_points - 1) * 6 + j] = 0;
		}
		else
		{
			acc[(num_points - 1) * 6 + j] = (mJointAngle[(num_points - 2) * 6 + j] - mJointAngle[(num_points - 1) * 6 + j]) / abs(mJointAngle[(num_points - 2) * 6 + j] - mJointAngle[(num_points - 1) * 6 + j]) * acc_value[j];
			tb[(num_points - 1) * 6 + j] = td[num_points - 2] - sqrt(td[num_points - 2] * td[num_points - 2] - 2 * (mJointAngle[(num_points - 2) * 6 + j] - mJointAngle[(num_points - 1) * 6 + j]) / acc[(num_points - 1) * 6 + j]);
		}
		//第一段直线速度和最后一段直线速度
		velocity_line[0 * 6 + j] = (mJointAngle[1 * 6 + j] - mJointAngle[0 * 6 + j]) / (td[0] - 0.5 * tb[0 * 6 + j]);
		velocity_line[(num_points - 2) * 6 + j] = (mJointAngle[(num_points - 1) * 6 + j] - mJointAngle[(num_points - 2) * 6 + j]) / (td[num_points - 2] - 0.5 * tb[(num_points - 1) * 6 + j]);
		//计算剩下每段的直线速度
		for (int i = 1;i < num_points - 2;i++)
		{
			velocity_line[i * 6 + j] = (mJointAngle[(i + 1) * 6 + j] - mJointAngle[i * 6 + j]) / td[i];
		}
		//计算剩下每段的加速度和加速时间
		for (int i = 1;i < num_points - 1;i++)
		{
			if (velocity_line[i * 6 + j] == velocity_line[(i - 1) * 6 + j])
			{
				acc[i * 6 + j] = 0;
				tb[i * 6 + j] = 0;
			}
			else
			{
				acc[i * 6 + j] = (velocity_line[i * 6 + j] - velocity_line[(i - 1) * 6 + j]) / abs(velocity_line[i * 6 + j] - velocity_line[(i - 1) * 6 + j]) * acc_value[j];
				tb[i * 6 + j] = (velocity_line[i * 6 + j] - velocity_line[(i - 1) * 6 + j]) / (2 * acc[i * 6 + j]);
			}
		}
		//位移补偿量
		for (int k = 1;k < num_points - 1;k++)
		{
			mJointAngle[k * 6 + j] = mJointAngle[k * 6 + j] + 0.5 * acc[k * 6 + j] * tb[k * 6 + j] * tb[k * 6 + j];
		}
		//计算每个时刻的角度
		for (int k = 0;k < num_points - 1;k++)
		{
			//ceil向上取整，保证大于对应函数范围下界
			for (int i = ceil(t[k] / mSampleTime);i < (t[k] + tb[k * 6 + j]) / mSampleTime;i++)
			{
				count++;
				my_angle[i * 6 + j] = mJointAngle[k * 6 + j] + (velocity_line[k * 6 + j] + 0.5 * acc[k * 6 + j] * i * mSampleTime - acc[k * 6 + j] * (t[k] + tb[k * 6 + j])) * i * mSampleTime
					+ (-velocity_line[k * 6 + j] + acc[k * 6 + j] * (0.5 * t[k] + tb[k * 6 + j])) * t[k];
			}
			for (int i = ceil((t[k] + tb[k * 6 + j]) / mSampleTime);i < (t[k + 1] - tb[(k + 1) * 6 + j]) / mSampleTime;i++)
			{
				count++;
				//my_angle[i * 6 + j] = 0;
				my_angle[i * 6 + j] = mJointAngle[k * 6 + j] + velocity_line[k * 6 + j] * (i * mSampleTime - t[k]) - 0.5 * acc[k * 6 + j] * tb[k * 6 + j] * tb[k * 6 + j];
				//my_angle[i * 6 + j] = mJointAngle[(k + 1) * 6 + j] + velocity_line[k * 6 + j] * (i * mSampleTime - t[k + 1]) - 0.5 * acc[(k + 1) * 6 + j] * tb[(k + 1) * 6 + j] * tb[(k + 1) * 6 + j];
			}
			for (int i = ceil((t[k + 1] - tb[(k + 1) * 6 + j]) / mSampleTime);i < t[k + 1] / mSampleTime;i++)
			{
				count++;
				my_angle[i * 6 + j] = mJointAngle[(k + 1) * 6 + j] + (velocity_line[k * 6 + j] + 0.5 * acc[(k + 1) * 6 + j] * i * mSampleTime - acc[(k + 1) * 6 + j] * (t[k + 1] - tb[(k + 1) * 6 + j])) * i * mSampleTime
					+ (-velocity_line[k * 6 + j] + acc[(k + 1) * 6 + j] * (0.5 * t[k + 1] - tb[(k + 1) * 6 + j])) * t[k + 1];
			}
		}
	}
	std::cout << count << std::endl;
	outfile << my_angle[t[1] / mSampleTime * 6 + 0] << " ";
	outfile << my_angle[t[1] / mSampleTime * 6 + 1] << " ";
	outfile << my_angle[t[1] / mSampleTime * 6 + 2] << " ";
	outfile << my_angle[t[1] / mSampleTime * 6 + 3] << " ";
	outfile << my_angle[t[1] / mSampleTime * 6 + 4] << " ";
	outfile << my_angle[t[1] / mSampleTime * 6 + 5] << " ";
	outfile << '\n';
	outfile << my_angle[t[num_points - 2] / mSampleTime * 6 + 0] << " ";
	outfile << my_angle[t[num_points - 2] / mSampleTime * 6 + 1] << " ";
	outfile << my_angle[t[num_points - 2] / mSampleTime * 6 + 2] << " ";
	outfile << my_angle[t[num_points - 2] / mSampleTime * 6 + 3] << " ";
	outfile << my_angle[t[num_points - 2] / mSampleTime * 6 + 4] << " ";
	outfile << my_angle[t[num_points - 2] / mSampleTime * 6 + 5] << " ";
	outfile << '\n';
	//打印
	for (int i = 0;i < length;i++)
	{
		outfile << my_angle[i * 6 + 0] << " ";
		outfile << my_angle[i * 6 + 1] << " ";
		outfile << my_angle[i * 6 + 2] << " ";
		outfile << my_angle[i * 6 + 3] << " ";
		outfile << my_angle[i * 6 + 4] << " ";
		outfile << my_angle[i * 6 + 5] << " ";
		outfile << '\n';//保存初始的时间、六个关节角度
	}
	//完成代码
}

int main(int argc, const char** argv)
{
	/*double x,y,z,yaw,pitch,roll;
	SetRobotJoint(0, 0.1, 0, 0, 0, 0);
	GetJointEndPos(x, y, z, yaw, pitch, roll);
	std::cout << x << std::endl;
	std::cout << y << std::endl;
	std::cout << z << std::endl;
	std::cout << yaw << std::endl;
	std::cout << pitch << std::endl;
	std::cout << roll << std::endl;*/
	SetPlanAngles();
	//SetPlanPoints();
	GetPlanPoints();
}