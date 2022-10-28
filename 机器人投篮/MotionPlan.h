#pragma once
#include <vector>
using namespace std;

struct PosStruct
{

};

class CHLMotionPlan
{
private:
	std::vector<double> mJointAngle;			//点位的关节角度,单位度				
	std::vector<double> mJointCoord;			//点位的笛卡尔坐标
	double mSampleTime;							//采样点位，单位S
	double mVel;								//速度，单位m/s
	double mAcc;								//加速度，单位m/s/s
	double mDec;								//减速度，单位m / s / s
	bool mConfig[3];							//机器人姿态

public:
	CHLMotionPlan();
	virtual ~CHLMotionPlan();

	void SetSampleTime(double sampleTime);		//设置采样时间
	void SetPlanPoints(PosStruct startPos, PosStruct endPos);		//输入起始点位和结束点位的笛卡尔坐标
	void SetPlanAngles(PosStruct startPos, PosStruct endPos);
	void SetProfile(double vel, double acc, double dec);			//设置运动参数，速度、加速度和减速度
	void GetPlanPoints();											//关节空间梯形速度规划
};