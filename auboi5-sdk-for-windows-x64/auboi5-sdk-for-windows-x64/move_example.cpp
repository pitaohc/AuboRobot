/**
* 机器人样例测试程序
* 功能1：读取测试点位.txt中的点位序列，并让机器人按照指定的点位顺序移动
* 功能2：
*/

#include "stdafx.h"
#include "robot.h"
#include <rsdef.h>
#include <string>
#include <queue>
#include <fstream>
#include <sstream>
#include <Windows.h>


int _tmain(int argc, _TCHAR* argv[])
{
    RSHD g_rshd = -1;
    setWayPointVector("路径.txt");
    int res = 0;
    // 登录
    res = login(g_rshd, ROBOT_ADDR, ROBOT_PORT); //登录
    if (!res) {
        return -1;
    }
    //启动机械臂(必须连接真实机械臂)
    res = robotStartup(g_rshd);
    if (!res) {
        return -1;
    }
    //工程启动，可以包含在启动机械臂中
    std::cout << "机械臂运行中" << std::endl;
    moveTest(g_rshd);

    //system("Pause");
    //关闭机械臂(必须连接真实机械臂)
    robotShutdown(g_rshd);
    //注销
    logout(g_rshd); 
    system("Pause");
    return 0;
}
