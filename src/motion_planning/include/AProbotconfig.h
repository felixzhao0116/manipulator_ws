
#ifndef ROBOTCONFIG_H_
#define ROBOTCONFIG_H_
const double PI = 3.1415926;
const int R=200;    //动平台到电机转轴的距离
const int L=350;    //主动臂长
const int l=800;    //从动臂长
const int r=45;     //静平台半径 单位mm
const double theta_max=100*PI/180;
const double theta_min=-39*PI/180;//定义静平台向下移动时旋转角度为正

namespace APRobot
{
    const double home[6]={0,0,-620.464,0,180,180};//定义初始位姿,这个指导书上是不是有问题
    //double TransVector[6];//保存当前位姿
    //double theta[3];//保存当前电机转角
	//Inverse kinematics solution
	void SetRobotEndPos(double x, double y, double z, double yaw, double pitch, double roll);
	void GetJointAngles(double &angle1, double &angle2, double &angle3, double &angle4);
	
	//Forward kinematics solution
	void SetRobotJoint(double angle1, double angle2, double angle3, double angle4);
	void GetJointEndPos(double &x, double &y, double &z, double &yaw, double &pitch, double &roll);

	//Inverse kinematics and Forward kinematics method function
	void robotBackward(const double* TransVector, bool config, double* theta);
	void robotForward(const double* q, double* TransVector, bool config);
}

#endif
