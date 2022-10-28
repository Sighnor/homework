// ConsoleApplication1.cpp : 定义控制台应用程序的入口点。
//

#include<iostream>
#include<winsock.h>
#include <conio.h>
#include "ftp/FtpControl.h"
#include "MotionPlan.h"
#include <fstream>

#pragma comment(lib,"ws2_32.lib")
using namespace std;

void initialization();

int main()
{
	//起始点
	PosStruct Start;

	//终止点
	PosStruct End;

	//梯型速度规划
	CHLMotionPlan trajectory1;
	trajectory1.SetPlanAngles(Start, End);
	trajectory1.SetProfile(10, 10, 10);    //vel °/s， acc °/s.s, dec °/s.s
	trajectory1.SetSampleTime(0.001);      //s
	trajectory1.GetPlanPoints();           //关节空间梯形速度规划

	std::vector<double> td;

	double temp_time;
	ifstream time;
	time.open("time.txt");

	while (time >> temp_time)
	{
		td.push_back(temp_time * 1000);
	}

	time.close();

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
	if (connect(s_server, (SOCKADDR*)&server_addr, sizeof(SOCKADDR)) == SOCKET_ERROR) {
		cout << "服务器连接失败！" << endl;
		WSACleanup();
	}
	else {
		cout << "服务器连接成功！" << endl;
	}

	//登录
	send_len = send(s_server, "[1# System.Login 0]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[Login]" << '\t' << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(1200);
	//使能
	send_len = send(s_server, "[2# Robot.PowerEnable 1,1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	cout << "[Power]" << '\t' << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(1200);

	send_len = send(s_server, "[3# Thread.Abort]", 100, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	cout << "[Abort]" << '\t' << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(1200);

	send_len = send(s_server, "[4# Thread.Start]", 100, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	cout << "[Start]" << '\t' << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(1200);

	send_len = send(s_server, "[5# Robot.Home 1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	cout << "[Home]" << '\t' << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(1200);

	send_len = send(s_server, "[6# System.Auto 1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	cout << "[Auto]" << '\t' << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(1200);

	send_len = send(s_server, "[7# System.Speed 100]", 100, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	cout << "[Speed]" << '\t' << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(1200);
	
	FtpControl::Upload("192.168.10.101", "data", "data.txt", "serverdata.txt");//upload the joint data to control

	send_len = send(s_server, "[12# PPB.Enable 1,1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	cout << "[PPB. Enable]" << '\t' << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(1200);

	send_len = send(s_server, "[13# Robot.Frame 1,1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	cout << "[Frame2]" << '\t' << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(1200);

	send_len = send(s_server, "[14# PPB.ReadFile 1,/data/serverdata.txt]", 100, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	cout << "[ReadFile]" << '\t' << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(1200);

	send_len = send(s_server, "[15# PPB.J2StartPoint 1,0,1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	cout << "[ToStartPoint]" << '\t' << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));

	send_len = send(s_server, "[16# PPB.Run 1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	cout << "[Run]" << '\t' << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));

	Sleep(0.9 * td[0]);

	send_len = send(s_server, "[17# IO.Set DOUT(20101),1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[IO_ON]" << '\t' << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));

	Sleep(0.1 * td[0] + td[1] + td[2] + 0.4 * td[3]);

	send_len = send(s_server, "[18# IO.Set DOUT(20101),0]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << "[IO_OFF]" << '\t' << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));

	for (size_t i = 0; i < 45; i+=5)
	{
		Sleep(0.6 * td[i + 3] + td[i + 4] + td[i + 5]);

		send_len = send(s_server, "[17# IO.Set DOUT(20101),1]", 100, 0);
		recv_len = recv(s_server, recv_buf, 100, 0);
		cout << "[IO_ON]" << '\t' << recv_buf << endl;
		memset(recv_buf, '\0', sizeof(recv_buf));

		Sleep(0 * td[i + 5] + td[i + 6] + td[i + 7] + 0.4 * td[i + 8]);

		send_len = send(s_server, "[18# IO.Set DOUT(20101),0]", 100, 0);
		recv_len = recv(s_server, recv_buf, 100, 0);
		cout << "[IO_OFF]" << '\t' << recv_buf << endl;
		memset(recv_buf, '\0', sizeof(recv_buf));
	}

	send_len = send(s_server, "[19# Robot.PowerEnable 1,0]", 100, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	cout << "[Power]" << '\t' << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(1200);


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
