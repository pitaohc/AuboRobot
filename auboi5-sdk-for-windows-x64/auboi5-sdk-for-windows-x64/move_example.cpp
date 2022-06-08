/**
* �������������Գ���
* ����1����ȡ���Ե�λ.txt�еĵ�λ���У����û����˰���ָ���ĵ�λ˳���ƶ�
* ����2��
*/

#include "stdafx.h"
#include <rsdef.h>
#include <string>
#include <queue>
#include <fstream>
#include <sstream>
#include<Windows.h>
constexpr char ROBOT_ADDR[] = "192.168.1.10"; //������ip��ַ
constexpr unsigned int ROBOT_PORT = 8899; //�����˶˿�
constexpr double M_PI = 3.14159265358979323846; //��
bool login(RSHD& rshd, const char* addr, int port); //��¼
bool logout(RSHD rshd); //ע��
bool robotStartup(RSHD rshd); //����
bool robotShutdown(RSHD rshd);//ֹͣ
void moveTest(RSHD rshd); //�ƶ���е��
std::queue<wayPoint_S> setWayPointQueue(std::string filepath, bool hasHeader = true); //���ļ���ȡ��λ·��

int _tmain(int argc, _TCHAR* argv[])
{
    setWayPointQueue("����λ��.txt");
    RSHD g_rshd = -1;
    int res = 0;
    // ��¼
    res = login(g_rshd, ROBOT_ADDR, ROBOT_PORT); //��¼
    if (!res) {
        return -1;
    }
    //������е��(����������ʵ��е��)
    res = robotStartup(g_rshd);
    if (!res) {
        return -1;
    }
    //�������������԰�����������е����
    std::cout << "��е��������" << std::endl;
    moveTest(g_rshd);

    system("Pause");
    //�رջ�е��(����������ʵ��е��)
    robotShutdown(g_rshd);
    //ע��
    logout(g_rshd); 
    system("Pause");
    return 0;
}
/****************************************************
    function:	login
    purpose :	��½��е��
    param   :	rshd ��������ľ��
                addr ��е�۷�������ַ
                port ��е�۷������˿�
    return  :	true �ɹ� false ʧ��
****************************************************/
bool login(RSHD& rshd, const char* addr, int port)
{
    int status; //����������״̬��Ϣ
    rshd = RS_FAILED;
    //��ʼ���ӿڿ�
    status = RS_FAILED;
    status = rs_initialize();
    if (status != RS_SUCC) {
        std::cout << "��ʼ���ӿڿ����" << std::endl;
        return false;
    }
    //����������
    status = RS_FAILED;
    status = rs_create_context(&rshd);
    if (status != RS_SUCC) {
        std::cout << "���������Ĵ���" << std::endl;
        return false;
    }
    //��½��е�۷�����
    status = RS_FAILED;
    status = rs_login(rshd, addr, port);
    if (status != RS_SUCC) {
        std::cout << "��½��е��ʧ��" << std::endl;
        return false;
    }

    std::cout << "��¼�ɹ�" << std::endl;
    return true;
}

/**************************************************
    function:	logout
    purpose :	�˳���½
    param   :	rshd �����ľ��

    return  :	true �ɹ� false ʧ��
**************************************************/

bool logout(RSHD rshd)
{
    std::cout << "ע���ɹ�" << std::endl;
    rs_logout(rshd);
    return rs_uninitialize() == RS_SUCC ? true : false;
}

/********************************************************************
    function:	robotStartup
    purpose :	������е��(����������ʵ��е�ۣ�
    param   :	rshd �����ľ��

    return  :	true �ɹ� false ʧ��
*********************************************************************/
bool robotStartup(RSHD rshd)
{
    int result = RS_FAILED;

    //���ߵĶ���ѧ�������˶�ѧ����
    ToolDynamicsParam tool_dynamics = { 0 };
    //��е����ײ�ȼ�
    uint8 colli_class = 6;
    //��е�������Ƿ��ȡ��̬��Ĭ�Ͽ�����
    bool read_pos = true;
    //��е�۾�̬��ײ��⣨Ĭ�Ͽ�����
    bool static_colli_detect = true;
    //��е�������ٶȣ�ϵͳ�Զ����ƣ�Ĭ��Ϊ30000)
    int board_maxacc = 30000;
    //��е�۷�������״̬
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
        std::cout << "����������ʧ��" << std::endl;
        return false;
    }
    
    rs_project_startup(rshd);

    std::cout << "�����������ɹ�, ������״̬:" << state << std::endl;
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
    purpose :	���·����Ϣ
    param   :	wayPoint �������·��

    return  :	void
*********************************************************************/
void printRoadPoint(const aubo_robot_namespace::wayPoint_S* wayPoint)
{
    //�������
    std::cout << "pos.x=" << wayPoint->cartPos.position.x << std::endl;
    std::cout << "pos.y=" << wayPoint->cartPos.position.y << std::endl;
    std::cout << "pos.z=" << wayPoint->cartPos.position.z << std::endl;
    //�����������Ԫ����ʾ
    std::cout << "ori.w=" << wayPoint->orientation.w << std::endl;
    std::cout << "ori.x=" << wayPoint->orientation.x << std::endl;
    std::cout << "ori.y=" << wayPoint->orientation.y << std::endl;
    std::cout << "ori.z=" << wayPoint->orientation.z << std::endl;
    //�ؽڽǶ�
    std::cout << "joint_1=" << wayPoint->jointpos[0] * 180.0 / M_PI << std::endl;
    std::cout << "joint_2=" << wayPoint->jointpos[1] * 180.0 / M_PI << std::endl;
    std::cout << "joint_3=" << wayPoint->jointpos[2] * 180.0 / M_PI << std::endl;
    std::cout << "joint_4=" << wayPoint->jointpos[3] * 180.0 / M_PI << std::endl;
    std::cout << "joint_5=" << wayPoint->jointpos[4] * 180.0 / M_PI << std::endl;
    std::cout << "joint_6=" << wayPoint->jointpos[5] * 180.0 / M_PI << std::endl;
}



/*
    �ƶ���е�۲���
*/
void moveTest(RSHD rshd)
{
    std::queue<wayPoint_S> que;
    //��ӽڵ�
    que = setWayPointQueue("����λ��.txt");
    //�ƶ��ڵ�
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
        rs_move_joint(rshd, jointpos, true); //�ƶ�,���ƶ�������������
        //rs_get_current_waypoint(rshd, currentWayPoint_S); //�õ���ǰλ��
        //printRoadPoint(currentWayPoint_S); //�����ǰλ��
        que.pop();
    }
}

std::queue<wayPoint_S> setWayPointQueue(std::string filepath,bool hasHeader)
{
    std::queue<wayPoint_S> que;
    std::ifstream input(filepath); //�ļ�����
    std::string info; //�ݴ������
    //��ȡ���б�ͷ
    if (hasHeader) 
    {
        std::getline(input, info);
    }
    //��ȡʣ��������
    while (!input.eof())
    {
        unsigned int id; //���
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
        std::cout << "���" << id <<"  "<<info << std::endl;
        printRoadPoint(&point);
        std::cout << std::endl;
        que.push(point);
    }

    return que;
}