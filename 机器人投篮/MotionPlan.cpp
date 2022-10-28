#include <iostream>
#include <fstream>
#include "MotionPlan.h"
#include "HLRobotconfig.h"
#include <algorithm>
#include <Windows.h>
#include "eigen3/Eigen/Dense"
#include <iomanip>

using namespace std;
using namespace HLRobot;
using namespace Eigen;

/********************************************************************
ABSTRACT:	构造函数

INPUTS:		<none>

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/

CHLMotionPlan::CHLMotionPlan()
{
	mSampleTime = 0.001;
	mVel = 0;
	mAcc = 0;
	mDec = 0;
}

/********************************************************************
ABSTRACT:	析构函数

INPUTS:		<none>

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/
CHLMotionPlan::~CHLMotionPlan()
{

}

/********************************************************************
ABSTRACT:	设置采样时间

INPUTS:		sampleTime			采样时间，单位S

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/
void CHLMotionPlan::SetSampleTime(double sampleTime)
{
	if (sampleTime < 0.001)
	{
		mSampleTime = 0.001;
	}
	else
	{
		mSampleTime = sampleTime;
	}
}

/********************************************************************
ABSTRACT:	设置运动参数

INPUTS:		vel			速度，单位m/s
			acc			加速度，单位m/s/s
			dec			减速度，单位m / s / s

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/
void CHLMotionPlan::SetProfile(double vel, double acc, double dec)
{
	mVel = vel;
	mAcc = acc;
	mDec = dec;
}

/********************************************************************
ABSTRACT:	设置规划的起始单位和技术点位

INPUTS:		startPos			起始点位笛卡尔坐标
			endPos				结束点位笛卡尔坐标

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/
void CHLMotionPlan::SetPlanPoints(PosStruct startPos, PosStruct endPos)
{
    double temp_pos;
    ifstream pos;
    pos.open("pos.txt");

    while (pos >> temp_pos)
    {
        mJointCoord.push_back(temp_pos);
    }

    std::cout << mJointCoord.size() << endl;

    pos.close();

    int temp_bool;
    ifstream my_bool;
    my_bool.open("bool.txt");
    std::vector<int> mConfig;

    while (my_bool >> temp_bool)
    {
        mConfig.push_back(temp_bool);
    }

    my_bool.close();

	for (int i = 0; i < mJointCoord.size() / 6; i++)
	{
		SetRobotEndPos(mJointCoord[6 * i + 0], mJointCoord[6 * i + 1], mJointCoord[6 * i + 2], mJointCoord[6 * i + 3], mJointCoord[6 * i + 4], mJointCoord[6 * i + 5]);
		double angle1, angle2, angle3, angle4, angle5, angle6;
		GetJointAngles(angle1, angle2, angle3, angle4, angle5, angle6, mConfig[i]);
		mJointAngle.push_back(angle1);
		mJointAngle.push_back(angle2);
		mJointAngle.push_back(angle3);
		mJointAngle.push_back(angle4);
		mJointAngle.push_back(angle5);
		mJointAngle.push_back(angle6);
	}
}

void CHLMotionPlan::SetPlanAngles(PosStruct startPos, PosStruct endPos)
{
	double temp_angle;
	ifstream angle;
	angle.open("angle.txt");

	while (angle >> temp_angle)
	{
		mJointAngle.push_back(temp_angle);
	}
}

/********************************************************************
ABSTRACT:	运动轨迹规划部分（以关节空间为例）

INPUTS:		pos						二维位置向量

OUTPUTS:	pos						二维位置向量（第一维：位置个数，第二维：每个轴的关节角度，单位弧度）

RETURN:		<none>
***********************************************************************/

/******
 * 参考步骤
 * 步骤1：创建文件并写入初始角度
 * 步骤2：计算每个轴旋转的角度
 * 步骤3：计算每个轴移动到终止点所需要时间
 * 步骤4：根据采样时间计算离散点位数
 * 步骤5：根据离散点位数得到每刻的关节角数值
 * 步骤6：将每时刻的关节角数值写入到文件
 */
void CHLMotionPlan::GetPlanPoints()
{
	ofstream outfile;               			//创建文件
	outfile.open("data.txt");

	int num_points = mJointAngle.size() / 6;

	double mSampleTime = 0.001;

	std::vector<double> td;

	double temp_time;
	ifstream time;
	time.open("time.txt");

	while (time >> temp_time)
	{
		td.push_back(temp_time);
	}

	time.close();

	std::vector<double> acc_value = { 600, 600, 600, 600, 600, 600 };
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
			for (int i = t[k] / mSampleTime;i < (t[k] + tb[k * 6 + j]) / mSampleTime;i++)
			{
				my_angle[i * 6 + j] = mJointAngle[k * 6 + j] + (velocity_line[k * 6 + j] + 0.5 * acc[k * 6 + j] * i * mSampleTime - acc[k * 6 + j] * (t[k] + tb[k * 6 + j])) * i * mSampleTime
					+ (-velocity_line[k * 6 + j] + acc[k * 6 + j] * (0.5 * t[k] + tb[k * 6 + j])) * t[k];
			}
			for (int i = (t[k] + tb[k * 6 + j]) / mSampleTime;i < (t[k + 1] - tb[(k + 1) * 6 + j]) / mSampleTime;i++)
			{
				//my_angle[i * 6 + j] = 0;
				my_angle[i * 6 + j] = mJointAngle[k * 6 + j] + velocity_line[k * 6 + j] * (i * mSampleTime - t[k]) - 0.5 * acc[k * 6 + j] * tb[k * 6 + j] * tb[k * 6 + j];
				//my_angle[i * 6 + j] = mJointAngle[(k + 1) * 6 + j] + velocity_line[k * 6 + j] * (i * mSampleTime - t[k + 1]) - 0.5 * acc[(k + 1) * 6 + j] * tb[(k + 1) * 6 + j] * tb[(k + 1) * 6 + j];
			}
			for (int i = (t[k + 1] - tb[(k + 1) * 6 + j]) / mSampleTime;i < t[k + 1] / mSampleTime;i++)
			{
				my_angle[i * 6 + j] = mJointAngle[(k + 1) * 6 + j] + (velocity_line[k * 6 + j] + 0.5 * acc[(k + 1) * 6 + j] * i * mSampleTime - acc[(k + 1) * 6 + j] * (t[k + 1] - tb[(k + 1) * 6 + j])) * i * mSampleTime
					+ (-velocity_line[k * 6 + j] + acc[(k + 1) * 6 + j] * (0.5 * t[k + 1] - tb[(k + 1) * 6 + j])) * t[k + 1];
			}
		}
	}
	//打印
	for (int i = 0;i < length - 1;i++)
	{
		outfile << my_angle[i * 6 + 0] << " ";
		outfile << my_angle[i * 6 + 1] << " ";
		outfile << my_angle[i * 6 + 2] << " ";
		outfile << my_angle[i * 6 + 3] << " ";
		outfile << my_angle[i * 6 + 4] << " ";
		outfile << my_angle[i * 6 + 5] << " ";
		outfile << '\n';//保存初始的时间、六个关节角度
	}
    outfile.close();
	//完成代码
}