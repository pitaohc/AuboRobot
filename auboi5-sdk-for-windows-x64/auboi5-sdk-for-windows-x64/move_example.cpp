/**
* �������������Գ���
* ����1����ȡ���Ե�λ.txt�еĵ�λ���У����û����˰���ָ���ĵ�λ˳���ƶ�
* ����2��
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
    setWayPointVector("·��.txt");
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

    //system("Pause");
    //�رջ�е��(����������ʵ��е��)
    robotShutdown(g_rshd);
    //ע��
    logout(g_rshd); 
    system("Pause");
    return 0;
}
