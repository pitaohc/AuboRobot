#include "stdafx.h"
// auboi5-sdk-for-windows-x64.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include "rsdef.h"
#include <string>
#include "example.h"

#define ROBOT_ADDR "192.168.1.10"
#define ROBOT_PORT 8899

//机械臂控制上下文句柄
RSHD g_rshd = -1;

int _tmain(int argc, _TCHAR* argv[])
{
	//登录服务器
	if (example_login(g_rshd, ROBOT_ADDR, ROBOT_PORT))
	{
		////启动机械臂(必须连接真实机械臂）
		//example_robotStartup(g_rshd);

		//工程启动
		rs_project_startup(g_rshd);

		////机械臂轴动测试
		//example_moveJ(g_rshd);

		////机械臂保持当前姿态直线运动测试
		//example_moveL(g_rshd);

		////机械臂轨迹运动测试
		//example_moveP(g_rshd);

		////机械臂正逆解测试
		//example_ik_fk(g_rshd);

		////机械臂控制柜IO测试(必须连接真实机械臂）
		//example_boardIO(g_rshd);

		////机械臂工具端IO测试(必须连接真实机械臂）
		//example_ToolIO(g_rshd);

		////实时路点信息回调函数测试
		example_callbackRobotRoadPoint(g_rshd);



		////延时2两秒，观察回调函数
		//Sleep(2000);

		//rs_move_fast_stop(g_rshd);

		////抱住刹车
		//rs_robot_control(g_rshd, RobotBrake);

		////关闭机械臂（必须连接真实机械臂）
		//example_robotShutdown(g_rshd);

		//诊断信息
		//example_get_diagnosis(g_rshd);

		//工程停止
		rs_project_stop(g_rshd);

		//退出登录
		example_logout(g_rshd);
	}

	//反初始化接口库
	rs_uninitialize();

	std::cout<<"please enter to exit"<<std::endl;
	getchar();

	return 0;
}

