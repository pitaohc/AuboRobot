/**
* 机器人样例测试程序
* 功能1：读取测试点位.txt中的点位序列，并让机器人按照指定的点位顺序移动
* 功能2：
*/

#include "stdafx.h"
#include <rsdef.h>
#include <string>
#include <queue>
#include <fstream>
#include <sstream>
#include<Windows.h>
constexpr char ROBOT_ADDR[] = "192.168.1.10"; //机器人ip地址
constexpr unsigned int ROBOT_PORT = 8899; //机器人端口
constexpr double M_PI = 3.14159265358979323846; //派
bool login(RSHD& rshd, const char* addr, int port); //登录
bool logout(RSHD rshd); //注销
bool robotStartup(RSHD rshd); //启动
bool robotShutdown(RSHD rshd);//停止
void moveTest(RSHD rshd); //移动机械臂
std::queue<wayPoint_S> setWayPointQueue(std::string filepath, bool hasHeader = true); //从文件读取点位路径

int _tmain(int argc, _TCHAR* argv[])
{
    setWayPointQueue("测试位点.txt");
    RSHD g_rshd = -1;
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

    system("Pause");
    //关闭机械臂(必须连接真实机械臂)
    robotShutdown(g_rshd);
    //注销
    logout(g_rshd); 
    system("Pause");
    return 0;
}
/****************************************************
    function:	login
    purpose :	登陆机械臂
    param   :	rshd 输出上下文句柄
                addr 机械臂服务器地址
                port 机械臂服务器端口
    return  :	true 成功 false 失败
****************************************************/
bool login(RSHD& rshd, const char* addr, int port)
{
    int status; //各个操作的状态信息
    rshd = RS_FAILED;
    //初始化接口库
    status = RS_FAILED;
    status = rs_initialize();
    if (status != RS_SUCC) {
        std::cout << "初始化接口库错误" << std::endl;
        return false;
    }
    //创建上下文
    status = RS_FAILED;
    status = rs_create_context(&rshd);
    if (status != RS_SUCC) {
        std::cout << "创建上下文错误" << std::endl;
        return false;
    }
    //登陆机械臂服务器
    status = RS_FAILED;
    status = rs_login(rshd, addr, port);
    if (status != RS_SUCC) {
        std::cout << "登陆机械臂失败" << std::endl;
        return false;
    }

    std::cout << "登录成功" << std::endl;
    return true;
}

/**************************************************
    function:	logout
    purpose :	退出登陆
    param   :	rshd 上下文句柄

    return  :	true 成功 false 失败
**************************************************/

bool logout(RSHD rshd)
{
    std::cout << "注销成功" << std::endl;
    rs_logout(rshd);
    return rs_uninitialize() == RS_SUCC ? true : false;
}

/********************************************************************
    function:	robotStartup
    purpose :	启动机械臂(必须连接真实机械臂）
    param   :	rshd 上下文句柄

    return  :	true 成功 false 失败
*********************************************************************/
bool robotStartup(RSHD rshd)
{
    int result = RS_FAILED;

    //工具的动力学参数和运动学参数
    ToolDynamicsParam tool_dynamics = { 0 };
    //机械臂碰撞等级
    uint8 colli_class = 6;
    //机械臂启动是否读取姿态（默认开启）
    bool read_pos = true;
    //机械臂静态碰撞检测（默认开启）
    bool static_colli_detect = true;
    //机械臂最大加速度（系统自动控制，默认为30000)
    int board_maxacc = 30000;
    //机械臂服务启动状态
    ROBOT_SERVICE_STATE state = ROBOT_SERVICE_READY;

    result = rs_robot_startup(
        rshd, 
        &tool_dynamics, 
        colli_class, 
        read_pos, 
        static_colli_detect, 
        board_maxacc, 
        &state);
    if (result != RS_SUCC)
    {
        std::cout << "机器人启动失败" << std::endl;
        return false;
    }
    
    rs_project_startup(rshd);

    std::cout << "机器人启动成功, 机器人状态:" << state << std::endl;
    return true;
}

bool robotShutdown(RSHD rshd)
{
    bool result = false;
    result = rs_project_stop(rshd);
    result &= rs_robot_shutdown(rshd) == RS_SUCC;
    return result;
}

/********************************************************************
    function:	printRoadPoint
    purpose :	输出路点信息
    param   :	wayPoint 待输出的路点

    return  :	void
*********************************************************************/
void printRoadPoint(const aubo_robot_namespace::wayPoint_S* wayPoint)
{
    //输出坐标
    std::cout << "pos.x=" << wayPoint->cartPos.position.x << std::endl;
    std::cout << "pos.y=" << wayPoint->cartPos.position.y << std::endl;
    std::cout << "pos.z=" << wayPoint->cartPos.position.z << std::endl;
    //输出方向，由四元数表示
    std::cout << "ori.w=" << wayPoint->orientation.w << std::endl;
    std::cout << "ori.x=" << wayPoint->orientation.x << std::endl;
    std::cout << "ori.y=" << wayPoint->orientation.y << std::endl;
    std::cout << "ori.z=" << wayPoint->orientation.z << std::endl;
    //关节角度
    std::cout << "joint_1=" << wayPoint->jointpos[0] * 180.0 / M_PI << std::endl;
    std::cout << "joint_2=" << wayPoint->jointpos[1] * 180.0 / M_PI << std::endl;
    std::cout << "joint_3=" << wayPoint->jointpos[2] * 180.0 / M_PI << std::endl;
    std::cout << "joint_4=" << wayPoint->jointpos[3] * 180.0 / M_PI << std::endl;
    std::cout << "joint_5=" << wayPoint->jointpos[4] * 180.0 / M_PI << std::endl;
    std::cout << "joint_6=" << wayPoint->jointpos[5] * 180.0 / M_PI << std::endl;
}



/*
    移动机械臂测试
*/
void moveTest(RSHD rshd)
{
    std::queue<wayPoint_S> que;
    //添加节点
    que = setWayPointQueue("测试位点.txt");
    //移动节点
    RobotRecongnitionParam param;
    rs_get_robot_recognition_param(rshd, 1, &param);

    wayPoint_S* currentWayPoint_S;
    while (!que.empty())
    {
        double jointpos[6];
        for (size_t i = 0; i < 6; i++)
        {
            jointpos[i] = que.front().jointpos[i] / 180 * M_PI;
        }
        rs_move_joint(rshd, jointpos, true); //移动,，移动参数设置问题
        //rs_get_current_waypoint(rshd, currentWayPoint_S); //得到当前位置
        //printRoadPoint(currentWayPoint_S); //输出当前位置
        que.pop();
    }
}

std::queue<wayPoint_S> setWayPointQueue(std::string filepath,bool hasHeader)
{
    std::queue<wayPoint_S> que;
    std::ifstream input(filepath); //文件输入
    std::string info; //暂存的输入
    //读取首行表头
    if (hasHeader) 
    {
        std::getline(input, info);
    }
    //读取剩下所有行
    while (!input.eof())
    {
        unsigned int id; //序号
        wayPoint_S point;
        Pos pos;
        Ori orientation;
        input >> id;
        input >> pos.x >> pos.y >> pos.z
            >> orientation.w >> orientation.x >> orientation.y >> orientation.z;
        for (size_t i = 0; i < ARM_DOF; i++)
        {
            input >> point.jointpos[i];
        }
        input >> info;
        point.cartPos.position = pos;
        point.orientation = orientation;
        std::cout << "序号" << id <<"  "<<info << std::endl;
        printRoadPoint(&point);
        std::cout << std::endl;
        que.push(point);
    }

    return que;
}