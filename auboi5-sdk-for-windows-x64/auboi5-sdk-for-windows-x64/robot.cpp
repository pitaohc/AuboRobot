#include"stdafx.h"
#include"robot.h"
using namespace std;

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
        cout << "初始化接口库错误" << endl;
        return false;
    }
    //创建上下文
    status = RS_FAILED;
    status = rs_create_context(&rshd);
    if (status != RS_SUCC) {
        cout << "创建上下文错误" << endl;
        return false;
    }
    //登陆机械臂服务器
    status = RS_FAILED;
    status = rs_login(rshd, addr, port);
    if (status != RS_SUCC) {
        cout << "登陆机械臂失败" << endl;
        return false;
    }

    cout << "登录成功" << endl;
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
    cout << "注销成功" << endl;
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
        cout << "机器人启动失败" << endl;
        return false;
    }
    rs_project_startup(rshd);
    cout << "机器人启动成功, 机器人状态:" << state << endl;
    return true;
}

/********************************************************************
    function:	robotShutdown
    purpose :	关闭机械臂(必须连接真实机械臂）
    param   :	rshd 上下文句柄

    return  :	true 成功 false 失败
*********************************************************************/
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
    cout << "pos.x=" << wayPoint->cartPos.position.x << endl;
    cout << "pos.y=" << wayPoint->cartPos.position.y << endl;
    cout << "pos.z=" << wayPoint->cartPos.position.z << endl;
    //输出方向，由四元数表示
    cout << "ori.w=" << wayPoint->orientation.w << endl;
    cout << "ori.x=" << wayPoint->orientation.x << endl;
    cout << "ori.y=" << wayPoint->orientation.y << endl;
    cout << "ori.z=" << wayPoint->orientation.z << endl;
    //关节角度
    cout << "joint_1=" << wayPoint->jointpos[0] * 180.0 / M_PI << endl;
    cout << "joint_2=" << wayPoint->jointpos[1] * 180.0 / M_PI << endl;
    cout << "joint_3=" << wayPoint->jointpos[2] * 180.0 / M_PI << endl;
    cout << "joint_4=" << wayPoint->jointpos[3] * 180.0 / M_PI << endl;
    cout << "joint_5=" << wayPoint->jointpos[4] * 180.0 / M_PI << endl;
    cout << "joint_6=" << wayPoint->jointpos[5] * 180.0 / M_PI << endl;
}

/********************************************************************
    function:	moveTest
    purpose :	移动机械臂测试
    param   :	rshd 上下文句柄

    return  :	void
*********************************************************************/
void moveTest(RSHD rshd)
{
    //添加节点
    auto path = setWayPointVector("路径.txt");
    //移动节点
    RobotRecongnitionParam param;
    rs_get_robot_recognition_param(rshd, 1, &param);

    wayPoint_S* currentWayPoint_S;
    for (auto waypoint: path)
    {
        double jointpos[6];
        for (size_t i = 0; i < 6; i++)
        {
            jointpos[i] = waypoint.jointpos[i];
            //jointpos[i] = que.front().jointpos[i] / 180 * M_PI;
        }
        rs_move_joint(rshd, jointpos, true); //移动,，移动参数设置问题
        rs_get_current_waypoint(rshd, currentWayPoint_S); //得到当前位置
        printRoadPoint(currentWayPoint_S); //输出当前位置
        Sleep(2000);
    }
}

/********************************************************************
    function:	setWayPointQueue
    purpose :	设置路点队列
    param   :	filepath 路点文件路径
                hasHeader 是否包含了表头

    return  :	queue<wayPoint_S> 一个路点队列
*********************************************************************/
vector<wayPoint_S> setWayPointVector(string filepath, bool hasHeader)
{
    //比较结构
    struct cmp {
        bool operator()(const pair<int, wayPoint_S>& left, const pair<int, wayPoint_S>& right) {
            return left.first < right.first;
        }
    };
    priority_queue<pair<int, wayPoint_S>,vector<pair<int, wayPoint_S>>,cmp> que; //优先队列
    ifstream input(filepath); //文件输入
    string info; //暂存的输入
    //读取首行表头
    if (hasHeader)
    {
        getline(input, info);
    }
    //读取剩下所有行
    while (!input.eof())
    {
        unsigned int id; //序号
        wayPoint_S point; //路点
        Pos pos; //坐标
        Rpy rpy; //欧拉角表示
        Ori orientation; //四元数表示
        input >> id;
        input >> pos.x >> pos.y >> pos.z
            >> rpy.rx >> rpy.ry >> rpy.rz;

        rs_rpy_to_quaternion(-1, &rpy, &orientation); //不知道使用-1是否可行，欧拉角转四元数
        for (size_t i = 0; i < ARM_DOF; i++)
        {
            input >> point.jointpos[i];
        }
        point.cartPos.position = pos;
        point.orientation = orientation;
        que.push(make_pair(id,point));
    }
    vector<wayPoint_S> res;
    while (!que.empty())
    {
        cout << que.top().first << endl;
        res.push_back(que.top().second);
        que.pop();
    }
    return res;
}

/********************************************************************
    function:	getWayPoint
    purpose :	得到当前路点
    param   :	rshd 上下文句柄

    return  :	wayPoint_S 一个路点结构体
*********************************************************************/
wayPoint_S getWayPoint(RSHD rshd)
{
    wayPoint_S currentWayPoint_S;
    rs_get_current_waypoint(rshd, &currentWayPoint_S); //得到当前位置
    return currentWayPoint_S;
}

/********************************************************************
    function:	getPos
    purpose :	得到当前位置
    param   :	rshd 上下文句柄

    return  :	wayPoint_S 一个位置结构体
*********************************************************************/
Pos getPos(RSHD rshd)
{
    Pos pos;
    wayPoint_S currentWayPoint_S;
    rs_get_current_waypoint(rshd, &currentWayPoint_S); //得到当前位置
    pos = currentWayPoint_S.cartPos.position;
    return pos;
}

/********************************************************************
    function:	getOri
    purpose :	得到当前四元数角度
    param   :	rshd 上下文句柄

    return  :	Ori 一个四元数结构体
*********************************************************************/
Ori getOri(RSHD rshd)
{
    Ori ori;
    wayPoint_S currentWayPoint_S;
    rs_get_current_waypoint(rshd, &currentWayPoint_S); //得到当前位置
    ori = currentWayPoint_S.orientation;
    return ori;
}

/********************************************************************
    function:	getRpy
    purpose :	得到当前欧拉角角度
    param   :	rshd 上下文句柄

    return  :	Rpy 一个欧拉角结构体
*********************************************************************/
Rpy getRpy(RSHD rshd)
{
    Ori ori = getOri(rshd);
    Rpy rpy;
    rs_quaternion_to_rpy(rshd, &ori, &rpy);
    return rpy;
}

/********************************************************************
    function:	getJoint
    purpose :	得到当前六轴关节角度
    param   :	rshd 上下文句柄

    return  :	vector<double> 一个关节vector
*********************************************************************/
vector<double> getJoint(RSHD rshd)
{
    vector<double> joints(ARM_DOF);
    wayPoint_S wp = getWayPoint(rshd);
    for (size_t i = 0; i < ARM_DOF; i++)
    {
        joints[i] = wp.jointpos[i];
    }
    return joints;
}
