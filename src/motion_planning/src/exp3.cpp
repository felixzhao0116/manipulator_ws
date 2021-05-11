#include<iostream>
#include<winsock.h>
#include <conio.h>
#include "AProbotconfig.h"
#include "Eigen\Dense"
#pragma comment(lib,"ws2_32.lib")
double TransVector[16];//位姿矩阵
double joint[4] = { 0 };
using namespace std;
using namespace Eigen;
void initialization();
#pragma comment(lib, "WS2_32.lib")
int main()
{   //
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

    //登录
    send_len = send(s_server, "[1# System.Login 0]", 100, 0);
    recv_len = recv(s_server, recv_buf, 100, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);
    //使能
    send_len = send(s_server, "[2# Robot.PowerEnable 1,1]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);

    //在此添加程序
    double p1[6] = { 21.7431,  185.458, -560.968,0, 180, 22.196 };
    double p2[6] = { -175.239,  78.127, -600.793,0, 180, 18.325 };
    double p3[6] = { 3.61595, -131.886, -578.863,0, 180, 21.923 };
    double centre[6] = { -54.339,-25.648,-573.122,0,180,21.923 };
    double radius = 80;

    Matrix<double, 18, 6> tri_point;
    Matrix<double, 18, 6> tri_joint_point;
    Matrix<double, 36, 6> circle_point;
    Matrix<double, 36, 6> circle_joint_point;

    for (int i = 0; i < 6; i++)
    {
        tri_point(i, 0) = (p1[0] * (6 - i) + p2[0] * i) / 6.0;
        tri_point(i, 1) = (p1[1] * (6 - i) + p2[1] * i) / 6.0;
        tri_point(i, 2) = (p1[2] * (6 - i) + p2[2] * i) / 6.0;
        tri_point(i, 3) = 0;
        tri_point(i, 4) = 180;
        tri_point(i, 5) = 22.196;


    }
    for (int i = 0; i < 6; i++)
    {
        tri_point(i + 6, 0) = (p2[0] * (6 - i) + p3[0] * i) / 6.0;
        tri_point(i + 6, 1) = (p2[1] * (6 - i) + p3[1] * i) / 6.0;
        tri_point(i + 6, 2) = (p2[2] * (6 - i) + p3[2] * i) / 6.0;
        tri_point(i + 6, 3) = 0;
        tri_point(i + 6, 4) = 180;
        tri_point(i + 6, 5) = 22.196;
    }
    for (int i = 0; i < 6; i++)
    {
        tri_point(i + 12, 0) = (p3[0] * (6 - i) + p1[0] * i) / 6.0;
        tri_point(i + 12, 1) = (p3[1] * (6 - i) + p1[1] * i) / 6.0;
        tri_point(i + 12, 2) = (p3[2] * (6 - i) + p1[2] * i) / 6.0;
        tri_point(i + 12, 3) = 0;
        tri_point(i + 12, 4) = 180;
        tri_point(i + 12, 5) = 22.196;
    }
    for (int i = 0; i < 36; i++)
    {
        circle_point(i, 0) = centre[0] + radius * cos(10.0 * i / 180.0 * PI);
        circle_point(i, 1) = centre[1] + radius * sin(10.0 * i / 180.0 * PI);
        circle_point(i, 2) = -573.122;
        circle_point(i, 3) = 0;
        circle_point(i, 4) = 180;
        circle_point(i, 5) = 22.196;

    }
    for (int i = 0; i < 18; i++)
    {
        APRobot::SetRobotEndPos(tri_point(i, 0), tri_point(i, 1), tri_point(i, 2), tri_point(i, 3), tri_point(i, 4), tri_point(i, 5));
        APRobot::GetJointAngles(joint[0], joint[1], joint[2], joint[3]);
        for (int j = 0; j < 6; j++)
        {
            tri_joint_point(i, j) = joint[j];
        }
    }

    for (int i = 0; i < 36; i++)
    {
        APRobot::SetRobotEndPos(circle_point(i, 0), circle_point(i, 1), circle_point(i, 2), circle_point(i, 3), circle_point(i, 4), circle_point(i, 5));
        APRobot::GetJointAngles(joint[0], joint[1], joint[2], joint[3]);
        for (int j = 0; j < 6; j++)
        {
            circle_joint_point(i, j) = joint[j];
        }
    }

    send_len = send(s_server, "[3# System.Abort 1]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);

    send_len = send(s_server, "[4# System.Start]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);

    send_len = send(s_server, "[5# System.Auto 1]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);

    send_len = send(s_server, "[6# Robot.Home 1]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);

    send_len = send(s_server, "[6# LocationJ joy =0,0,0,0]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);


    //三角形
    /*string macro_a = "[7# joy = ";
    string macro_b = "[8# Move.Joint ";
    string joint_str[6], command1, command2,temp;
    for (int i = 0; i < 18; i++)
    {
        const char* macro_A;
        const char* macro_B;
        for (int j = 0; j < 4; j++)
        {
            joint_str[j] = to_string(tri_joint_point(i, j));
        }
        temp = joint_str[0] + "," + joint_str[1] + "," + joint_str[2] + "," + joint_str[3] + "]";
        command1 = macro_a + temp;
        macro_A = command1.data();
        send_len = send(s_server, macro_A, 100, 0);
        recv_len = recv(s_server, recv_buf, 200, 0);
        cout << recv_buf << endl;
        memset(recv_buf, '\0', sizeof(recv_buf));



        command2 = macro_b + "joy" + "]";
        macro_B = command2.data();

        send_len = send(s_server, macro_B, 100, 0);
        recv_len = recv(s_server, recv_buf, 200, 0);
        cout << recv_buf << endl;
        memset(recv_buf, '\0', sizeof(recv_buf));
        Sleep(500);


    }*/


    //圆形
    string macro_a = "[7# joy = ";
    string macro_b = "[8# Move.Joint ";
    string joint_str[6], command1, command2, temp;
    for (int i = 0; i < 36; i++)
    {
        const char* macro_A;
        const char* macro_B;
        for (int j = 0; j < 4; j++)
        {
            joint_str[j] = to_string(circle_joint_point(i, j));
        }
        temp = joint_str[0] + "," + joint_str[1] + "," + joint_str[2] + "," + joint_str[3] + "]";
        command1 = macro_a + temp;
        macro_A = command1.data();
        send_len = send(s_server, macro_A, 100, 0);
        recv_len = recv(s_server, recv_buf, 200, 0);
        cout << recv_buf << endl;
        memset(recv_buf, '\0', sizeof(recv_buf));

        command2 = macro_b + "joy" + "]";
        macro_B = command2.data();
        send_len = send(s_server, macro_B, 100, 0);
        recv_len = recv(s_server, recv_buf, 200, 0);
        cout << recv_buf << endl;
        memset(recv_buf, '\0', sizeof(recv_buf));
        Sleep(500);

    }


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
}
