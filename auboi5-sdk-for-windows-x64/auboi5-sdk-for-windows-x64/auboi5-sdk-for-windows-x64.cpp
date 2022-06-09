#include "stdafx.h"
// auboi5-sdk-for-windows-x64.cpp : �������̨Ӧ�ó������ڵ㡣
//

#include "stdafx.h"
#include "rsdef.h"
#include <string>
#include "example.h"

#define ROBOT_ADDR "192.168.1.10"
#define ROBOT_PORT 8899

//��е�ۿ��������ľ��
RSHD g_rshd = -1;

int _tmain(int argc, _TCHAR* argv[])
{
	//��¼������
	if (example_login(g_rshd, ROBOT_ADDR, ROBOT_PORT))
	{
		////������е��(����������ʵ��е�ۣ�
		//example_robotStartup(g_rshd);

		//��������
		rs_project_startup(g_rshd);

		////��е���ᶯ����
		//example_moveJ(g_rshd);

		////��е�۱��ֵ�ǰ��ֱ̬���˶�����
		//example_moveL(g_rshd);

		////��е�۹켣�˶�����
		//example_moveP(g_rshd);

		////��е����������
		//example_ik_fk(g_rshd);

		////��е�ۿ��ƹ�IO����(����������ʵ��е�ۣ�
		//example_boardIO(g_rshd);

		////��е�۹��߶�IO����(����������ʵ��е�ۣ�
		//example_ToolIO(g_rshd);

		////ʵʱ·����Ϣ�ص���������
		example_callbackRobotRoadPoint(g_rshd);



		////��ʱ2���룬�۲�ص�����
		//Sleep(2000);

		//rs_move_fast_stop(g_rshd);

		////��סɲ��
		//rs_robot_control(g_rshd, RobotBrake);

		////�رջ�е�ۣ�����������ʵ��е�ۣ�
		//example_robotShutdown(g_rshd);

		//�����Ϣ
		//example_get_diagnosis(g_rshd);

		//����ֹͣ
		rs_project_stop(g_rshd);

		//�˳���¼
		example_logout(g_rshd);
	}

	//����ʼ���ӿڿ�
	rs_uninitialize();

	std::cout<<"please enter to exit"<<std::endl;
	getchar();

	return 0;
}

