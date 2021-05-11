// ConsoleApplication1.cpp : 定义控制台应用程序的入口点。


#include<iostream>
#include<winsock.h>
#include <conio.h>
#include "MotionPlan.h"
#pragma comment(lib,"ws2_32.lib")
using namespace std;
#include "FtpControl.h"
void initialization();
#pragma comment(lib, "WS2_32.lib")


double TransVector[16];//位姿矩阵
double joint[4] = { 0 };
int main()
{   
	/***********************
	*
	*该部分为与机器人之间的通讯，不需要更改
	*
	************************/

	//定义长度变量
	int send_len = 0;
	int recv_len = 0;
	//定义发送缓冲区和接受缓冲区
    char send_buf[100] = {};
	char recv_buf[200] = {};
    string recvstr;
	//定义服务端套接字，接受请求套接字
	SOCKET s_server;
	//服务端地址客户端地址
	SOCKADDR_IN server_addr;
	initialization();
	//填充服务端信息
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.S_un.S_addr = inet_addr("192.168.10.120");
	server_addr.sin_port = htons(2090);
	//创建套接字
	s_server = socket(AF_INET, SOCK_STREAM, 0);
	if (connect(s_server, (SOCKADDR *)&server_addr, sizeof(SOCKADDR)) == SOCKET_ERROR) {
		cout << "服务器连接失败！" << endl;
		WSACleanup();
	}
	else {
		cout << "服务器连接成功！" << endl;
	}

	/***********************
	*
	*该部分是机器人宏指令的使用方法
	*
	************************/

    send_len = send(s_server, "[1# System.Login 0]", 100, 0);
    recv_len = recv(s_server, recv_buf, 100, 0);
    cout << recv_buf << endl;
    memset(recv_buf,'\0',sizeof(recv_buf));
    Sleep(1200);

    send_len = send(s_server, "[2# Robot.PowerEnable 1,1]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);

    send_len = send(s_server, "[3# System.Abort 1]", 100, 0);
    recv_len = recv(s_server, recv_buf, 100, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);
    send_len = send(s_server, "[4# System.Start 1]", 100, 0);
    recv_len = recv(s_server, recv_buf, 100, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);
    send_len = send(s_server, "[5# Robot.Home 1]", 100, 0);
    recv_len = recv(s_server, recv_buf, 100, 0);

    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);

    send_len = send(s_server, "[6# System.Auto 1]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);


	/***********************
	*
	*该函数为将本机的文件上传至机器人控制器
	*data是文件夹的名字
	*mydata.txt是自己生成的txt文件名字
	*severdata.txt是上传到服务器端后文件的名字，和前者可以相同也可以不同
	*
	************************/
    PosStruct Start;
    Start.x = -5.239; Start.y = 338.12; Start.z = -600.7;
    Start.yaw = 0; Start.pitch = 180; Start.roll = -31.675;
    /*PosStruct Start;
    Start.x = 21.741; Start.y = 185.457; Start.z = -560.968;
    Start.yaw = 0; Start.pitch = 180; Start.roll = 22.19;*/
    //终止点
    PosStruct End;
    End.x = -165.2; End.y = 237.98; End.z = -600.7;
    End.yaw = 0; End.pitch = 180; End.roll = -31.675;
    /*PosStruct End;
    End.x = -175.239; End.y = 78.127; End.z = -600.793;
    End.yaw = 0; End.pitch = 180; End.roll = 18.325;*/

    //梯型速度规划
    CHLMotionPlan trajectory1;
    trajectory1.SetPlanPoints(Start, End);
    trajectory1.SetProfile(20, 20, 10);    //vel °/s， acc °/s.s, dec °/s.s
    trajectory1.SetSampleTime(0.001);      //s
    //trajectory1.GetPlanPoints();
    //trajectory1.GetPlanPoints_line(1);
    trajectory1.Palletizing();
    FtpControl::Upload("192.168.10.101", "data", "data1.txt", "data1.txt");
    FtpControl::Upload("192.168.10.101", "data", "data2.txt", "data2.txt");
    FtpControl::Upload("192.168.10.101", "data", "data3.txt", "data3.txt");
    FtpControl::Upload("192.168.10.101", "data", "data4.txt", "data4.txt");
    FtpControl::Upload("192.168.10.101", "data", "data5.txt", "data5.txt");

    send_len = send(s_server, "[7# PPB.Enable 1,1]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);

    send_len = send(s_server, "[8# Robot.Frame 1,2]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);

    send_len = send(s_server, "[9# IO.Set DOUT(20102),0]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    //Sleep(1200);

    send_len = send(s_server, "[9# IO.Set DOUT(20103),0]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);

    send_len = send(s_server, "[9# PPB.ReadFile 1,/data/data1.txt]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);

    send_len = send(s_server, "[9# PPB.J2StartPoint 1,0,1]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);

    send_len = send(s_server, "[9# PPB.Run 1]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);

    send_len = send(s_server, "[9# WaitTime 6000]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);
     
    send_len = send(s_server, "[10# IO.Set DOUT(20102),1]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
   // Sleep(1200);

    send_len = send(s_server, "[10# IO.Set DOUT(20103),0]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);

    send_len = send(s_server, "[9# WaitTime 1000]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);

    send_len = send(s_server, "[9# PPB.ReadFile 1,/data/data2.txt]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);

    send_len = send(s_server, "[9# PPB.J2StartPoint 1,0,1]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);

    send_len = send(s_server, "[9# PPB.Run 1]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);

    send_len = send(s_server, "[9# PPB.ReadFile 1,/data/data3.txt]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);

    send_len = send(s_server, "[9# PPB.J2StartPoint 1,0,1]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);

    send_len = send(s_server, "[9# PPB.Run 1]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);

    send_len = send(s_server, "[9# PPB.ReadFile 1,/data/data4.txt]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);

    send_len = send(s_server, "[9# PPB.J2StartPoint 1,0,1]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);

    send_len = send(s_server, "[9# PPB.Run 1]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);

    send_len = send(s_server, "[9# WaitTime 6000]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);

    send_len = send(s_server, "[10# IO.Set DOUT(20102),0]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    //Sleep(1200);

    send_len = send(s_server, "[10# IO.Set DOUT(20103),1]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);

    send_len = send(s_server, "[9# WaitTime 1000]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);

    send_len = send(s_server, "[9# PPB.ReadFile 1,/data/data5.txt]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);

    send_len = send(s_server, "[9# PPB.J2StartPoint 1,0,1]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);

    send_len = send(s_server, "[9# PPB.Run 1]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);

    send_len = send(s_server, "[10# IO.Set DOUT(20102),0]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    //Sleep(1200);

    send_len = send(s_server, "[10# IO.Set DOUT(20103),0]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);
	/*FtpControl::Upload("192.168.10.101", "data", "data.txt", "data.txt");

    send_len = send(s_server, "[7# PPB.Enable 1,1]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);

    send_len = send(s_server, "[8# Robot.Frame 1,1]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);

    send_len = send(s_server, "[9# PPB.ReadFile 1,/data/data.txt]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);

    send_len = send(s_server, "[9# PPB.J2StartPoint 1,0,1]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);

    send_len = send(s_server, "[9# PPB.Run 1]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);*/
    //cin >> send_buf;

	//关闭套接字
	closesocket(s_server);
	//释放DLL资源
	WSACleanup();
	return 0;
}
void initialization() {
	//初始化套接字库
	WORD w_req = MAKEWORD(2, 2);//版本号
	WSADATA wsadata;
	int err;
	err = WSAStartup(w_req, &wsadata);
	if (err != 0) {
		cout << "初始化套接字库失败！" << endl;
	}
	else {
		cout << "初始化套接字库成功！" << endl;
	}
	//检测版本号
	if (LOBYTE(wsadata.wVersion) != 2 || HIBYTE(wsadata.wHighVersion) != 2) {
		cout << "套接字库版本号不符！" << endl;
		WSACleanup();
	}
	else {
		cout << "套接字库版本正确！" << endl;
	}
	//填充服务端地址信息

}
