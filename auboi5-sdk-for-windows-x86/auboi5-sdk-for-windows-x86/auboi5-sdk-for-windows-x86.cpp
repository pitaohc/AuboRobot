// auboi5-sdk-for-windows-x86.cpp : �������̨Ӧ�ó������ڵ㡣
//

#include "stdafx.h"
#include "auboi5-sdk-for-windows-x86.h"
#include "example.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

#define ROBOT_ADDR "192.168.1.100"
#define ROBOT_PORT 8899

// Ψһ��Ӧ�ó������
CWinApp theApp;

//��е�ۿ��������ľ��
RSHD g_rshd = -1;

int _tmain(int argc, TCHAR* argv[], TCHAR* envp[])
{
	int nRetCode = 0;

	HMODULE hModule = ::GetModuleHandle(NULL);

	if (hModule != NULL)
	{
		// ��ʼ�� MFC ����ʧ��ʱ��ʾ����
		if (!AfxWinInit(hModule, NULL, ::GetCommandLine(), 0))
		{
			// TODO: ���Ĵ�������Է���������Ҫ
			_tprintf(_T("����: MFC ��ʼ��ʧ��\n"));
			nRetCode = 1;
		}
		else
		{
			//��¼������
			if (example_login(g_rshd, ROBOT_ADDR, ROBOT_PORT))
			{
				//������е��(����������ʵ��е�ۣ�
				//example_robotStartup(g_rshd);

				//�����̬����
				//demo_relativeOri(g_rshd);

				//��������
				rs_project_startup(g_rshd);

				////��е���ᶯ����
				example_moveJ(g_rshd);

				////��е�۱��ֵ�ǰ��ֱ̬���˶�����
				//example_moveL(g_rshd);

				////��е�۹켣�˶�����
				//example_moveP(g_rshd);

				//��е����������
				//example_ik_fk(g_rshd);

				////��е�ۿ��ƹ�IO����(����������ʵ��е�ۣ�
				//example_boardIO(g_rshd);

				////��е�۹��߶�IO����(����������ʵ��е�ۣ�
				//example_ToolIO(g_rshd);

				////ʵʱ·����Ϣ�ص���������
				//example_callbackRobotRoadPoint(g_rshd);

				////��ʱ2���룬�۲�ص�����
				//Sleep(2000);

				//�رջ�е�ۣ�����������ʵ��е�ۣ�
				//example_robotShutdown(g_rshd);

				//����ģʽ�µ��ᶯ����
				//example_follow_mode_movej(g_rshd);

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
		}
	}
	else
	{
		_tprintf(_T("����: GetModuleHandle ʧ��\n"));
		nRetCode = 1;
	}

	return nRetCode;
}
