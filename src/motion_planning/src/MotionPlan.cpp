#include <iostream>
#include <fstream>
#include "MotionPlan.h"
#include "AProbotconfig.h"
#include <algorithm>
//#include <Windows.h>
#include "math.h"
#include "Eigen/Dense"

using namespace std;
using namespace APRobot;
using namespace Eigen;

/********************************************************************
ABSTRACT:	构造函数

INPUTS:		<none>

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/

inline int CHLMotionPlan::sgn(double x) {
    if(x>0) return 1;
    if(x<0) return -1;
    if(x=0) return 0;
}
CHLMotionPlan::CHLMotionPlan()
{
	for (int i = 0; i < 6; i++)
	{
		mJointAngleBegin[i] = 0;
		mJointAngleEnd[i] = 0;
	}

	for (int i = 0; i < 16; i++)
	{
		mStartMatrixData[i] = 0;
		mEndMatrixData[i] = 0;
	}

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

	double angle1, angle2, angle3, angle4;
	APRobot::SetRobotEndPos(startPos.x, startPos.y, startPos.z, startPos.yaw, startPos.pitch, startPos.roll);
	APRobot::GetJointAngles(angle1, angle2, angle3, angle4);

	for(int i=0;i<16;i++) mStartMatrixData[i]=TransVector[i];
	mJointAngleBegin[0] = angle1;
	mJointAngleBegin[1] = angle2;
	mJointAngleBegin[2] = angle3;
	mJointAngleBegin[3] = angle4;

	APRobot::SetRobotEndPos(endPos.x, endPos.y, endPos.z, endPos.yaw, endPos.pitch, endPos.roll);
	APRobot::GetJointAngles(angle1, angle2, angle3, angle4);

    for(int i=0;i<16;i++) mEndMatrixData[i]=TransVector[i];
	mJointAngleEnd[0] = angle1;
	mJointAngleEnd[1] = angle2;
	mJointAngleEnd[2] = angle3;
	mJointAngleEnd[3] = angle4;

    for(int i=0;i<4;i++){
        cout<<mJointAngleBegin[i]<<endl;
        cout<<mJointAngleEnd[i]<<endl;
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
	outfile << mJointAngleBegin[0] << "  "
			<< mJointAngleBegin[1] << "  "
			<< mJointAngleBegin[2] << "  "
			<< mJointAngleBegin[3] << "  ";
    outfile << endl;//保存初始的时间、六个关节角度
	//计算期望运动时间
	int status[4]={1};//用来区分是三角形规划还是梯形规划，0为三角形，1为梯形
	double desired_time[4];
	double acc_time[4];//存储加速时间和减速时间
	for(int i=0;i<4;i++){
	    //路程较短，不需要保持最大速度匀速运行
	    if(abs((mJointAngleBegin[i]-mJointAngleEnd[i]))<pow(mVel,2)/mAcc){
	        status[i]=0;
	        desired_time[i]=2*sqrt(abs((mJointAngleBegin[i]-mJointAngleEnd[i]))/mAcc);
	        acc_time[i]=0.5*desired_time[i];
	    }
	    //路程较长，需要保持最大速度匀速运行
	    else {
	        status[i]=1;
            desired_time[i] = 2*mVel/mAcc+(abs((mJointAngleBegin[i]-mJointAngleEnd[i]))-pow(mVel,2)/mAcc)/mVel;
            acc_time[i]=mVel/mAcc;
        }
	}

	//计算采样点个数
	int sample_num=0;
	for(int i=0;i<4;i++){
	    int temp = ceil(desired_time[i]/mSampleTime);//计算出当前维度运动需要选取多少采样点
	    if (temp>sample_num) sample_num=temp;//记录需要最多采样点维度需要的点数
	}
	double sample_value;//记录各个维度的采样值
    double border_value=0;//保存交接点的数值
    for(int j=0;j<sample_num;j++){
        for(int i=0;i<4;i++){
            if(status[i]==1){
                if((j+1)*mSampleTime<acc_time[i])
                    sample_value=mJointAngleBegin[i]+0.5*sgn(mJointAngleEnd[i]-mJointAngleBegin[i])*mAcc*pow((j+1)*mSampleTime,2);
                else if((j+1)*mSampleTime>=acc_time[i]&&(j+1)*mSampleTime<desired_time[i]-acc_time[i]) {
                    border_value = mJointAngleBegin[i] +
                            0.5 * sgn(mJointAngleEnd[i] - mJointAngleBegin[i]) * mAcc *pow(acc_time[i], 2);
                    sample_value= border_value + sgn(mJointAngleEnd[i] - mJointAngleBegin[i])*mVel * ((j + 1) * mSampleTime - acc_time[i]);
                }
                else if((j+1)*mSampleTime>=desired_time[i]-acc_time[i]&&(j+1)*mSampleTime<desired_time[i]) {
                    border_value = mJointAngleBegin[i] +
                                   0.5 * sgn(mJointAngleEnd[i] - mJointAngleBegin[i]) * mAcc *pow(acc_time[i], 2)
                                   +sgn(mJointAngleEnd[i] - mJointAngleBegin[i])*mVel*(desired_time[i]-2*acc_time[i]);
                    sample_value = border_value+
                                    sgn(mJointAngleEnd[i] - mJointAngleBegin[i])*mVel*((j+1)*mSampleTime-(desired_time[i]-acc_time[i]))-
                                    0.5*sgn(mJointAngleEnd[i]-mJointAngleBegin[i])*mAcc*pow((j+1)*mSampleTime-(desired_time[i]-acc_time[i]),2);
                }
                else sample_value=mJointAngleBegin[i] +
                                    0.5 * sgn(mJointAngleEnd[i] - mJointAngleBegin[i]) * mAcc *pow(acc_time[i], 2)
                                    +sgn(mJointAngleEnd[i] - mJointAngleBegin[i])*mVel*(desired_time[i]-2*acc_time[i])+
                                    sgn(mJointAngleEnd[i] - mJointAngleBegin[i])*mVel*acc_time[i]-0.5* sgn(mJointAngleEnd[i] - mJointAngleBegin[i]) * mAcc *pow(acc_time[i], 2);
	         }
            else{
                if((j+1)*mSampleTime<acc_time[i]){
                    sample_value=mJointAngleBegin[i]+0.5*sgn(mJointAngleEnd[i]-mJointAngleBegin[i])*mAcc*pow((j+1)*mSampleTime,2);
                }
                else if((j+1)*mSampleTime>=acc_time[i]&&(j+1)*mSampleTime<desired_time[i]){
                    border_value = mJointAngleBegin[i] +
                            0.5 * sgn(mJointAngleEnd[i] - mJointAngleBegin[i]) * mAcc *pow(acc_time[i], 2);
                    sample_value=border_value+
                            sgn(mJointAngleEnd[i]-mJointAngleBegin[i])*mAcc*acc_time[i]*((j+1)*mSampleTime-acc_time[i])-
                            0.5*sgn(mJointAngleEnd[i]-mJointAngleBegin[i])*mAcc*pow((j+1)*mSampleTime-acc_time[i],2);
                }
                else sample_value=mJointAngleBegin[i] +
                        2 * 0.5 * sgn(mJointAngleEnd[i] - mJointAngleBegin[i]) * mAcc *pow(acc_time[i], 2);
                }
            outfile << sample_value << "  ";
	    }
        outfile << endl;
	}
}

void CHLMotionPlan::GetPlanPoints_line(int i)
{
    ofstream outfile;//创建文件
    string filename,index;
    index =to_string(i);
    filename="data"+index+".txt";
    outfile.open(filename);
    outfile << mStartMatrixData[3] << "  "
            << mStartMatrixData[7] << "  "
            << mStartMatrixData[11] << "  "
            << 0 << "  "
            << 180 <<"  "
            << atan2(mStartMatrixData[1],mStartMatrixData[0])*180/PI;
    outfile << endl;//保存初始的时间、六个关节角度
    //计算期望运动时间
    int status[4]={1};//用来区分是三角形规划还是梯形规划，0为三角形，1为梯形
    double desired_time[4];
    double acc_time[4];//存储加速时间和减速时间
    double start[4],end[4];
    double distance;
    Vector3d direction;
    direction(0, 0) = mEndMatrixData[3]-mStartMatrixData[3];
    direction(1, 0) = mEndMatrixData[7] - mStartMatrixData[7];
    direction(2, 0) = mEndMatrixData[11] - mStartMatrixData[11];
    distance = direction.norm();
    start[0] = 0;
    start[1]=0;
    start[2]=180;
    start[3]=atan2(mStartMatrixData[1],mStartMatrixData[0])*180/PI;
    end[0]=distance;
    end[1]=0;
    end[2]=180;
    end[3]=atan2(mEndMatrixData[1],mEndMatrixData[0])*180/PI;
    for(int i=0;i<4;i++){
        //路程较短，不需要保持最大速度匀速运行
        if(abs((start[i]-end[i]))<pow(mVel,2)/mAcc){
            status[i]=0;
            desired_time[i]=2*sqrt(abs((start[i]-end[i]))/mAcc);
            acc_time[i]=0.5*desired_time[i];
        }
            //路程较长，需要保持最大速度匀速运行
        else {
            status[i]=1;
            desired_time[i] = 2*mVel/mAcc+(abs((start[i]-end[i]))-pow(mVel,2)/mAcc)/mVel;
            acc_time[i]=mVel/mAcc;
        }
    }

    //计算采样点个数
    int sample_num=0;
    for(int i=0;i<4;i++){
        int temp = ceil(desired_time[i]/mSampleTime);//计算出当前维度运动需要选取多少采样点
        if (temp>sample_num) sample_num=temp;//记录需要最多采样点维度需要的点数
    }
    double sample_value;//记录各个维度的采样值
    double border_value=0;//保存交接点的数值
    for(int j=0;j<sample_num;j++){
        for(int i=0;i<4;i++){
            if(status[i]==1){
                if((j+1)*mSampleTime<acc_time[i])
                    sample_value=start[i]+0.5*sgn(end[i]-start[i])*mAcc*pow((j+1)*mSampleTime,2);
                else if((j+1)*mSampleTime>=acc_time[i]&&(j+1)*mSampleTime<desired_time[i]-acc_time[i]) {
                    border_value = start[i] +
                                   0.5 * sgn(end[i] - start[i]) * mAcc *pow(acc_time[i], 2);
                    sample_value= border_value + sgn(end[i] - start[i])*mVel * ((j + 1) * mSampleTime - acc_time[i]);
                }
                else if((j+1)*mSampleTime>=desired_time[i]-acc_time[i]&&(j+1)*mSampleTime<desired_time[i]) {
                    border_value = start[i] +
                                   0.5 * sgn(end[i] - start[i]) * mAcc *pow(acc_time[i], 2)
                                   +sgn(end[i] - start[i])*mVel*(desired_time[i]-2*acc_time[i]);
                    sample_value = border_value+
                                   sgn(end[i] - start[i])*mVel*((j+1)*mSampleTime-(desired_time[i]-acc_time[i]))-
                                   0.5*sgn(end[i]-start[i])*mAcc*pow((j+1)*mSampleTime-(desired_time[i]-acc_time[i]),2);
                }
                else sample_value=start[i] +
                                  0.5 * sgn(end[i] - start[i]) * mAcc *pow(acc_time[i], 2)
                                  +sgn(end[i] - start[i])*mVel*(desired_time[i]-2*acc_time[i])+
                                  sgn(end[i] - start[i])*mVel*acc_time[i]-0.5* sgn(end[i] - start[i]) * mAcc *pow(acc_time[i], 2);
            }
            else{
                if((j+1)*mSampleTime<acc_time[i]){
                    sample_value=start[i]+0.5*sgn(end[i]-start[i])*mAcc*pow((j+1)*mSampleTime,2);
                }
                else if((j+1)*mSampleTime>=acc_time[i]&&(j+1)*mSampleTime<desired_time[i]){
                    border_value = start[i] +
                                   0.5 * sgn(end[i] - start[i]) * mAcc *pow(acc_time[i], 2);
                    sample_value=border_value+
                                 sgn(end[i]-start[i])*mAcc*acc_time[i]*((j+1)*mSampleTime-acc_time[i])-
                                 0.5*sgn(end[i]-start[i])*mAcc*pow((j+1)*mSampleTime-acc_time[i],2);
                }
                else sample_value=start[i] +
                                  2 * 0.5 * sgn(end[i] - start[i]) * mAcc *pow(acc_time[i], 2);
            }
            if (i == 0) {
                double current_x = mStartMatrixData[3] + sample_value * direction.normalized()(0);
                double current_y = mStartMatrixData[7] + sample_value * direction.normalized()(1);
                double current_z = mStartMatrixData[11] + sample_value * direction.normalized()(2);
                outfile << current_x << "  " << current_y << "  " << current_z << "  ";
            }
            else outfile << sample_value << "  ";
        }
        outfile << endl;
    }
}

void CHLMotionPlan::Palletizing() {
    PosStruct start_up,start_down,end_up,end_down;
    double h=110;
    start_up.x=mStartMatrixData[3];start_up.y=mStartMatrixData[7];start_up.z=mStartMatrixData[11];
    start_up.yaw=0; start_up.pitch=180;start_up.roll=atan2(mStartMatrixData[1],mStartMatrixData[0])*180/PI;

    start_down=start_up;
    start_down.z=start_up.z-h;

    end_up.x=mEndMatrixData[3];end_up.y=mEndMatrixData[7];end_up.z=mEndMatrixData[11];
    end_up.yaw=0; end_up.pitch=180;end_up.roll=atan2(mEndMatrixData[1],mEndMatrixData[0])*180/PI;

    end_down=end_up;
    end_down.z=end_up.z-h;

    SetPlanPoints(start_up,start_down);
    GetPlanPoints_line(1);
    SetPlanPoints(start_down,start_up);
    GetPlanPoints_line(2);
    SetPlanPoints(start_up,end_up);
    GetPlanPoints_line(3);
    SetPlanPoints(end_up,end_down);
    GetPlanPoints_line(4);
    SetPlanPoints(end_down,end_up);
    GetPlanPoints_line(5);
}