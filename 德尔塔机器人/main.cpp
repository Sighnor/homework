#include <algorithm>
#include <Eigen/Eigen>
#include <iostream>
#include <math.h>
#include <fstream>

double R = 200;
double r = 45;
double L = 350;
double l = 800;
Eigen::Vector3d alpha(0, 120, -120);
const double PI = 3.1415926;
double mTransMatrix[16]{ 0 };
bool mConfig;

std::vector<double> mJointAngle;			//起始点位的关节角度,单位度
std::vector<double> mJointCoord;
double mSampleTime;							//采样点位，单位S
double mVel;								//速度，单位m/s
double mAcc;								//加速度，单位m/s/s
double mDec;								//减速度，单位m / s / s

/********************************************************************
ABSTRACT:	机器人逆运动学

INPUTS:		T[16]:	位姿矩阵，其中长度距离为米

			config：姿态，六轴机器人对应有8种姿态（即对应的逆运动学8个解），
			Scara机器人有2个解，Delta机器人有一个解，因此Delta可以不考虑config
			为了安全，实验室中我们只计算一种即可。config用来作为选解的标志数。

OUTPUTS:    theta[6] 6个关节角, 单位为弧度

RETURN:		<none>
***********************************************************************/
void robotBackward(const double* TransVector, bool mconfig, double* theta)
{
	double A[3];
	double B[3];
	double C[3];
	double t[3];

	double x = TransVector[3];
	double y = TransVector[7];
	double z = TransVector[11];

	double a = R - r;
	double b = x * x + y * y + z * z + (R - r) * (R - r) + L * L - l * l;
	double c[3] = { cos(PI / 180 * alpha.x()) * x + sin(PI / 180 * alpha.x()) * y,
					cos(PI / 180 * alpha.y()) * x + sin(PI / 180 * alpha.y()) * y,
					cos(PI / 180 * alpha.z()) * x + sin(PI / 180 * alpha.z()) * y };

	A[0] = (b - 2 * a * c[0]) / (2 * L) - (a - c[0]);
	A[1] = (b - 2 * a * c[1]) / (2 * L) - (a - c[1]);
	A[2] = (b - 2 * a * c[2]) / (2 * L) - (a - c[2]);

	B[0] = 2 * z;
	B[1] = 2 * z;
	B[2] = 2 * z;

	C[0] = (b - 2 * a * c[0]) / (2 * L) + (a - c[0]);
	C[1] = (b - 2 * a * c[1]) / (2 * L) + (a - c[1]);
	C[2] = (b - 2 * a * c[2]) / (2 * L) + (a - c[2]);

	for (int i = 0;i < 3;i++)
	{
		t[i] = -(B[i] + sqrt(B[i] * B[i] - 4 * A[i] * C[i])) / (2 * A[i]);
		theta[i] = 2 * atan(t[i]) * 180 / PI;
	}

	theta[3] = 180 + acos(-TransVector[0]) * 180 / PI;
	theta[4] = 0;
	theta[5] = 0;
}

/********************************************************************
ABSTRACT:	机器人正运动学

INPUTS:		q[6]: 6个关节角, 单位为弧度

OUTPUTS:	config用来作为选解的标志数。

			TransVector[16] : 刚体变换矩阵，也就是末端的位姿描述，其中长度距离为米

RETURN:		<none>
***********************************************************************/
void robotForward(const double* q, double* TransVector, bool mconfig)
{
	Eigen::Vector3d C1(((R - r + L * cos(PI / 180 * q[0])) * cos(PI / 180 * alpha.x())), ((R - r + L * cos(PI / 180 * q[0])) * sin(PI / 180 * alpha.x())), -L * sin(PI / 180 * q[0]));
	Eigen::Vector3d C2(((R - r + L * cos(PI / 180 * q[1])) * cos(PI / 180 * alpha.y())), ((R - r + L * cos(PI / 180 * q[1])) * sin(PI / 180 * alpha.y())), -L * sin(PI / 180 * q[1]));
	Eigen::Vector3d C3(((R - r + L * cos(PI / 180 * q[2])) * cos(PI / 180 * alpha.z())), ((R - r + L * cos(PI / 180 * q[2])) * sin(PI / 180 * alpha.z())), -L * sin(PI / 180 * q[2]));

	Eigen::Vector3d C1C2 = C2 - C1;
	Eigen::Vector3d C2C3 = C3 - C2;
	Eigen::Vector3d C3C1 = C1 - C3;

	double p = (C1C2.norm() + C2C3.norm() + C3C1.norm()) / 2;
	double norm_EC = (C1C2.norm() * C2C3.norm() * C3C1.norm()) / (4 * sqrt(p * (p - C1C2.norm()) * (p - C2C3.norm()) * (p - C3C1.norm())));
	Eigen::Vector3d F = (C1 + C2) / 2;
	Eigen::Vector3d n_FE = (C2C3.cross(C3C1)).cross(C1C2) / (C2C3.cross(C3C1)).cross(C1C2).norm();
	double norm_FE = sqrt(norm_EC * norm_EC - C1C2.norm() * C1C2.norm() / 4);
	Eigen::Vector3d E = norm_FE * n_FE + F;
	Eigen::Vector3d n_EP = -C1C2.cross(C2C3) / C1C2.cross(C2C3).norm();
	double norm_EP = sqrt(l * l - norm_EC * norm_EC);
	Eigen::Vector3d P = norm_EP * n_EP + E;

	double yaw = q[3];
	double pitch = q[4];
	double roll = q[5];

	TransVector[0] = cos(yaw) * cos(pitch) * cos(roll) - sin(yaw) * sin(roll);
	TransVector[1] = -cos(yaw) * cos(pitch) * sin(roll) - sin(yaw) * cos(roll);
	TransVector[2] = cos(yaw) * sin(pitch);
	TransVector[3] = P.x();

	TransVector[4] = sin(yaw) * cos(pitch) * cos(roll) + cos(yaw) * sin(roll);
	TransVector[5] = -sin(yaw) * cos(pitch) * sin(roll) + cos(yaw) * cos(roll);
	TransVector[6] = sin(yaw) * sin(pitch);
	TransVector[7] = P.y();

	TransVector[8] = -sin(pitch) * cos(roll);
	TransVector[9] = sin(pitch) * sin(roll);
	TransVector[10] = cos(pitch);
	TransVector[11] = P.z();

	TransVector[12] = 0;
	TransVector[13] = 0;
	TransVector[14] = 0;
	TransVector[15] = 1;
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
	bool config[3] = { false, false, false };
	robotBackward(mTransMatrix, config, theta);

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
	yaw = atan(mTransMatrix[6] / mTransMatrix[2]);
	pitch = acos(mTransMatrix[10]);
	roll = -atan(mTransMatrix[9] / mTransMatrix[8]);
}

void SetPlanPoints()
{
	mJointCoord.push_back(-48.95);
	mJointCoord.push_back(438.5);
	mJointCoord.push_back(-711);
	mJointCoord.push_back(0);
	mJointCoord.push_back(180);
	mJointCoord.push_back(22.190);

	mJointCoord.push_back(-48.95);
	mJointCoord.push_back(438.5);
	mJointCoord.push_back(-561.8805);
	mJointCoord.push_back(0);
	mJointCoord.push_back(180);
	mJointCoord.push_back(20.2575);

	mJointCoord.push_back(-5.21);
	mJointCoord.push_back(348.2);
	mJointCoord.push_back(-560.6);
	mJointCoord.push_back(0);
	mJointCoord.push_back(180);
	mJointCoord.push_back(18.325);

	mJointCoord.push_back(-5.21);
	mJointCoord.push_back(348.2);
	mJointCoord.push_back(-711);
	mJointCoord.push_back(0);
	mJointCoord.push_back(180);
	mJointCoord.push_back(18.325);

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
	}
}

void GetPlanPoints()
{
	std::ofstream outfile;               			//创建文件
	outfile.open("data.txt");

	int count = 0;

	int num_points = mJointAngle.size() / 6;

	double mSampleTime = 0.001;

	std::vector<double> td;
	double temp_time;
	std::ifstream time;
	time.open("time.txt");

	while (time >> temp_time)
	{
		td.push_back(temp_time);
	}

	time.close();

	std::vector<double> acc_value = { 10, 10, 10, 10, 10, 10 };
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
				my_angle[i * 6 + j] = mJointAngle[k * 6 + j] + (velocity_line[k * 6 + j] + 0.5 * acc[k * 6 + j] * i * mSampleTime - acc[k * 6 + j] * (t[k] + tb[k * 6 + j])) * i * mSampleTime
					+ (-velocity_line[k * 6 + j] + acc[k * 6 + j] * (0.5 * t[k] + tb[k * 6 + j])) * t[k];
			}
			for (int i = ceil((t[k] + tb[k * 6 + j]) / mSampleTime);i < (t[k + 1] - tb[(k + 1) * 6 + j]) / mSampleTime;i++)
			{
				my_angle[i * 6 + j] = mJointAngle[k * 6 + j] + velocity_line[k * 6 + j] * (i * mSampleTime - t[k]) - 0.5 * acc[k * 6 + j] * tb[k * 6 + j] * tb[k * 6 + j];
				//my_angle[i * 6 + j] = mJointAngle[(k + 1) * 6 + j] + velocity_line[k * 6 + j] * (i * mSampleTime - t[k + 1]) - 0.5 * acc[(k + 1) * 6 + j] * tb[(k + 1) * 6 + j] * tb[(k + 1) * 6 + j];
			}
			for (int i = ceil((t[k + 1] - tb[(k + 1) * 6 + j]) / mSampleTime);i < t[k + 1] / mSampleTime;i++)
			{
				my_angle[i * 6 + j] = mJointAngle[(k + 1) * 6 + j] + (velocity_line[k * 6 + j] + 0.5 * acc[(k + 1) * 6 + j] * i * mSampleTime - acc[(k + 1) * 6 + j] * (t[k + 1] - tb[(k + 1) * 6 + j])) * i * mSampleTime
					+ (-velocity_line[k * 6 + j] + acc[(k + 1) * 6 + j] * (0.5 * t[k + 1] - tb[(k + 1) * 6 + j])) * t[k + 1];
			}
		}
	}
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
	SetPlanPoints();
	GetPlanPoints();
}