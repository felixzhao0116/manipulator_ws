
#include "AProbotconfig.h"
#include "eigen3/Eigen/Dense"
#include "iostream"
using namespace std;
using namespace Eigen;
extern double TransVector[16];
namespace APRobot
{
	//初始化TransMatrix
	double mTransMatrix[16] {0};

	//只使用一种姿态
	bool mConfig = 1;

	
	void SetRobotEndPos(double x, double y, double z, double yaw, double pitch, double roll)
	{
        TransVector[0]=cos(roll*PI/180);
        TransVector[1]=sin(roll*PI/180);
        TransVector[2]=0;
        TransVector[3]=x;
        TransVector[4]=-sin(roll*PI/180);
        TransVector[5]=cos(roll*PI/180);
        TransVector[6]=0;
        TransVector[7]=y;
        TransVector[8]=0;
        TransVector[9]=0;
        TransVector[10]=1;
        TransVector[11]=z;
        TransVector[12]=0;
        TransVector[13]=0;
        TransVector[14]=0;
        TransVector[15]=1;

	}

	void GetJointAngles(double &angle1, double &angle2, double &angle3, double &angle4)
	{
	    double theta[6]={angle1,angle2,angle3,angle4};
        robotBackward(TransVector,0,theta);
        angle1=theta[0];
        angle2=theta[1];
        angle3=theta[2];
        angle4=theta[3];
	}

	void SetRobotJoint(double angle1, double angle2, double angle3, double angle4)
	{
        double joint_angle[4]={angle1,angle2,angle3,angle4};
	    robotForward(joint_angle,TransVector,0);
	}

	void GetJointEndPos(double &x, double &y, double &z, double &yaw, double &pitch, double &roll)
	{
        x=TransVector[3];
        y=TransVector[7];
        z=TransVector[11];
        yaw=0;
        pitch=180;
        roll=atan2(TransVector[1],TransVector[0])*180/PI;
        if(roll<0) roll+=360;
	}


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
		double x=TransVector[3];
		double y=TransVector[7];
		double z=TransVector[11];
		theta[3]=atan2(TransVector[1],TransVector[0])*180/PI+180;


		Vector3d p(x,y,z);
		Vector3d A1(R*cos(0),R*sin(0),0);
        Vector3d A2(R*cos(120*PI/180),R*sin(120*PI/180),0);
        Vector3d A3(R*cos(240*PI/180),R*sin(240*PI/180),0);
		Vector3d B1(x+r*cos(0),y+r*sin(0),z);
        Vector3d B2(x+r*cos(120*PI/180),y+r*sin(120*PI/180),z);
        Vector3d B3(x+r*cos(240*PI/180),y+r*sin(240*PI/180),z);
        Vector3d D1((B1).dot(A1.normalized())*cos(0),(B1).dot(A1.normalized())*sin(0),z);
        Vector3d D2((B2).dot(A2.normalized())*cos(120*PI/180),(B2).dot(A2.normalized())*sin(120*PI/180),z);
        Vector3d D3((B3).dot(A3.normalized())*cos(240*PI/180),(B3).dot(A3.normalized())*sin(240*PI/180),z);

        double A1D1_norm=(A1-D1).norm();
        double C1D1_norm=sqrt(pow(l,2)-pow((B1-D1).norm(),2));
        double OD1_norm=D1.norm();
        double A2D2_norm=(A2-D2).norm();
        double C2D2_norm=sqrt(pow(l,2)-pow((B2-D2).norm(),2));
        double OD2_norm=D2.norm();
        double A3D3_norm=(A3-D3).norm();
        double C3D3_norm=sqrt(pow(l,2)-pow((B3-D3).norm(),2));
        double OD3_norm=D3.norm();

        double rad_D1A1C1=acos((pow(A1D1_norm,2)+pow(L,2)-pow(C1D1_norm,2))/2/A1D1_norm/L);
        double rad_OA1D1=acos((pow(R,2)+pow(A1D1_norm,2)-pow(OD1_norm,2))/2/R/A1D1_norm);
        theta[0]=(PI-rad_D1A1C1-rad_OA1D1)*180/PI;
        double rad_D2A2C2=acos((pow(A2D2_norm,2)+pow(L,2)-pow(C2D2_norm,2))/2/A2D2_norm/L);
        double rad_OA2D2=acos((pow(R,2)+pow(A2D2_norm,2)-pow(OD2_norm,2))/2/R/A2D2_norm);
        theta[1]=(PI-rad_D2A2C2-rad_OA2D2)*180/PI;
        double rad_D3A3C3=acos((pow(A3D3_norm,2)+pow(L,2)-pow(C3D3_norm,2))/2/A3D3_norm/L);
        double rad_OA3D3=acos((pow(R,2)+pow(A3D3_norm,2)-pow(OD3_norm,2))/2/R/A3D3_norm);
        theta[2]=(PI-rad_D3A3C3-rad_OA3D3)*180/PI;

	}

	/********************************************************************
	ABSTRACT:	机器人正运动学
	
	INPUTS:		q[4]: 4个关节角, 单位为弧度

	
	OUTPUTS:	config用来作为选解的标志数。

				TransVector[16] : 刚体变换矩阵，也就是末端的位姿描述，其中长度距离为米
	
	RETURN:		<none>
	***********************************************************************/
	void robotForward(const double* q, double* TransVector, bool mconfig)
	{
        Vector3d OC1_((R-r+L*cos(q[0]))*cos(0),(R-r+L*cos(q[0]))*sin(0),-L*sin(q[0]));
        Vector3d OC2_((R-r+L*cos(q[1]))*cos(120*PI/180),(R-r+L*cos(q[1]))*sin(120*PI/180),-L*sin(q[1]));
        Vector3d OC3_((R-r+L*cos(q[2]))*cos(240*PI/180),(R-r+L*cos(q[2]))*sin(240*PI/180),-L*sin(q[2]));

        //海伦公式求外接圆半径
        double p=0.5*((OC1_-OC2_).norm()+(OC2_-OC3_).norm()+(OC3_-OC1_).norm());
        double EC1_=(OC1_-OC2_).norm()*(OC2_-OC3_).norm()*(OC3_-OC1_).norm()
                /(4*sqrt(p*(p-(OC1_-OC2_).norm())*(p-(OC2_-OC3_).norm())*(p-(OC3_-OC1_).norm())));
        //求F点坐标
        Vector3d OF=(OC1_+OC2_)/2;
        //求E点坐标
        Vector3d FE_direction=(((OC1_-OC3_).cross(OC2_-OC3_)).cross(OC2_-OC1_)).normalized();
        double FE_norm=sqrt(pow(EC1_,2)-pow((OC1_-OC2_).norm(),2)/4);
        Vector3d OE=FE_norm*FE_direction+OF;
        //求P点坐标
        Vector3d EP_direction=((OC3_-OC1_).cross(OC2_-OC1_)).normalized();
        double EP_norm=sqrt(pow(l,2)-pow(EC1_,2));
        Vector3d OP=EP_norm*EP_direction+OE;

        //确定末端位姿
        TransVector[0]=cos(q[3]-PI);//初始位姿为180对应电机转角为0
        TransVector[1]=sin(q[3]-PI);
        TransVector[2]=0;
        TransVector[3]=OP(0,0);
        TransVector[4]=-sin(q[3]-PI);
        TransVector[5]=cos(q[3]-PI);
        TransVector[6]=0;
        TransVector[7]=OP(1,0);
        TransVector[8]=0;
        TransVector[9]=0;
        TransVector[10]=1;
        TransVector[11]=OP(2,0);
        TransVector[12]=0;
        TransVector[13]=0;
        TransVector[14]=0;
        TransVector[15]=1;

    }
}
