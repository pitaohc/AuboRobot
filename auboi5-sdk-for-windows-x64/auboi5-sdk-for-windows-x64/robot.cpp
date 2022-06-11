#include"stdafx.h"
#include"robot.h"
using namespace std;

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
        cout << "��ʼ���ӿڿ����" << endl;
        return false;
    }
    //����������
    status = RS_FAILED;
    status = rs_create_context(&rshd);
    if (status != RS_SUCC) {
        cout << "���������Ĵ���" << endl;
        return false;
    }
    //��½��е�۷�����
    status = RS_FAILED;
    status = rs_login(rshd, addr, port);
    if (status != RS_SUCC) {
        cout << "��½��е��ʧ��" << endl;
        return false;
    }

    cout << "��¼�ɹ�" << endl;
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
    cout << "ע���ɹ�" << endl;
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
        cout << "����������ʧ��" << endl;
        return false;
    }
    rs_project_startup(rshd);
    cout << "�����������ɹ�, ������״̬:" << state << endl;
    return true;
}

/********************************************************************
    function:	robotShutdown
    purpose :	�رջ�е��(����������ʵ��е�ۣ�
    param   :	rshd �����ľ��

    return  :	true �ɹ� false ʧ��
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
    purpose :	���·����Ϣ
    param   :	wayPoint �������·��

    return  :	void
*********************************************************************/
void printRoadPoint(const aubo_robot_namespace::wayPoint_S* wayPoint)
{
    //�������
    cout << "pos.x=" << wayPoint->cartPos.position.x << endl;
    cout << "pos.y=" << wayPoint->cartPos.position.y << endl;
    cout << "pos.z=" << wayPoint->cartPos.position.z << endl;
    //�����������Ԫ����ʾ
    cout << "ori.w=" << wayPoint->orientation.w << endl;
    cout << "ori.x=" << wayPoint->orientation.x << endl;
    cout << "ori.y=" << wayPoint->orientation.y << endl;
    cout << "ori.z=" << wayPoint->orientation.z << endl;
    //�ؽڽǶ�
    cout << "joint_1=" << wayPoint->jointpos[0] * 180.0 / M_PI << endl;
    cout << "joint_2=" << wayPoint->jointpos[1] * 180.0 / M_PI << endl;
    cout << "joint_3=" << wayPoint->jointpos[2] * 180.0 / M_PI << endl;
    cout << "joint_4=" << wayPoint->jointpos[3] * 180.0 / M_PI << endl;
    cout << "joint_5=" << wayPoint->jointpos[4] * 180.0 / M_PI << endl;
    cout << "joint_6=" << wayPoint->jointpos[5] * 180.0 / M_PI << endl;
}

/********************************************************************
    function:	moveTest
    purpose :	�ƶ���е�۲���
    param   :	rshd �����ľ��

    return  :	void
*********************************************************************/
void moveTest(RSHD rshd, Pos biasPos, Rpy biasRpy)
{
    //���·��
    auto path = setWayPointVector(rshd,"detectpoint.txt");
    //�ƶ�������
    RobotRecongnitionParam param;
    rs_get_robot_recognition_param(rshd, 1, &param);

    for (auto basePoint: path) //���ݿ��ж����Ļ���·��
    {
        wayPoint_S finalPoint,currentPoint; //����Ŀ��·��,��ǰ·����Ϣ
        //��������λ��
        finalPoint.cartPos.position.y = basePoint.cartPos.position.y + biasPos.y;
        finalPoint.cartPos.position.x = basePoint.cartPos.position.x + biasPos.x;
        finalPoint.cartPos.position.z = basePoint.cartPos.position.z + biasPos.z;
        
        Rpy baseRpy,finalRpy; //������ŷ���ǣ����յ�ŷ����
        rs_quaternion_to_rpy(rshd, &basePoint.orientation, &baseRpy); //�ӻ�������Ԫ��ת��Ϊŷ����
        //����������̬
        finalRpy.rx = (baseRpy.rx + biasRpy.rx);
        finalRpy.ry = (baseRpy.ry + biasRpy.ry);
        finalRpy.rz = (baseRpy.rz + biasRpy.rz);
        rs_rpy_to_quaternion(rshd, &finalRpy, &finalPoint.orientation); //����ŷ����ת��Ϊ���յ���Ԫ��

        rs_get_current_waypoint(rshd, &currentPoint); //�õ���ǰλ��
        rs_inverse_kin(rshd, currentPoint.jointpos, 
            &finalPoint.cartPos.position, &finalPoint.orientation, 
            &finalPoint); //����ؽ���Ϣ

        //�ƶ������˵���Ӧ�Ĺؽ�λ��
        rs_move_joint(rshd, finalPoint.jointpos, true); //�ƶ�,�ƶ�������������
        //rs_move_joint(rshd, basePoint.jointpos, true); //�ƶ�,�ƶ�������������
        //cout << "ʵ��λ��\n";
        //printRoadPoint(&finalPoint); //�����ǰλ��
        rs_get_current_waypoint(rshd, &currentPoint); //�õ���ǰλ��
        cout << "��ǰλ��\n";
        printRoadPoint(&currentPoint); //�����ǰλ��
        Sleep(2000);
    }
}

/********************************************************************
    function:	setWayPointQueue
    purpose :	����·�����
    param   :	filepath ·���ļ�·��
                hasHeader �Ƿ�����˱�ͷ

    return  :	queue<wayPoint_S> һ��·�����
*********************************************************************/
vector<wayPoint_S> setWayPointVector(RSHD rshd,string filepath, bool hasHeader)
{
    //�ȽϽṹ
    struct cmp {
        bool operator()(const pair<int, wayPoint_S>& left, const pair<int, wayPoint_S>& right) {
            return left.first > right.first;
        }
    };
    priority_queue<pair<int, wayPoint_S>,vector<pair<int, wayPoint_S>>,cmp> que; //���ȶ���
    ifstream input(filepath); //�ļ�����
    string info; //�ݴ������
    //��ȡ���б�ͷ
    if (hasHeader)
    {
        getline(input, info);
    }
    //��ȡʣ��������
    while (!input.eof())
    {
        unsigned int id; //���
        wayPoint_S point; //·��
        Pos pos; //����
        Rpy rpy; //ŷ���Ǳ�ʾ
        Ori orientation; //��Ԫ����ʾ
        input >> id; //��һ�����õ�pointID
        input >> pos.x >> pos.y >> pos.z
            >> rpy.rx >> rpy.ry >> rpy.rz;
        for (size_t i = 0; i < ARM_DOF; i++)
        {
            input >> point.jointpos[i];
        }
        input >> id;//ʵ�ʵ�orderID
        rs_rpy_to_quaternion(rshd, &rpy, &orientation); //ʹ��-1��ִ��ʧ��
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
    purpose :	�õ���ǰ·��
    param   :	rshd �����ľ��

    return  :	wayPoint_S һ��·��ṹ��
*********************************************************************/
wayPoint_S getWayPoint(RSHD rshd)
{
    wayPoint_S currentWayPoint_S;
    rs_get_current_waypoint(rshd, &currentWayPoint_S); //�õ���ǰλ��
    return currentWayPoint_S;
}

/********************************************************************
    function:	getPos
    purpose :	�õ���ǰλ��
    param   :	rshd �����ľ��

    return  :	wayPoint_S һ��λ�ýṹ��
*********************************************************************/
Pos getPos(RSHD rshd)
{
    Pos pos;
    wayPoint_S currentWayPoint_S;
    rs_get_current_waypoint(rshd, &currentWayPoint_S); //�õ���ǰλ��
    pos = currentWayPoint_S.cartPos.position;
    return pos;
}

/********************************************************************
    function:	getOri
    purpose :	�õ���ǰ��Ԫ���Ƕ�
    param   :	rshd �����ľ��

    return  :	Ori һ����Ԫ���ṹ��
*********************************************************************/
Ori getOri(RSHD rshd)
{
    Ori ori;
    wayPoint_S currentWayPoint_S;
    rs_get_current_waypoint(rshd, &currentWayPoint_S); //�õ���ǰλ��
    ori = currentWayPoint_S.orientation;
    return ori;
}

/********************************************************************
    function:	getRpy
    purpose :	�õ���ǰŷ���ǽǶ�
    param   :	rshd �����ľ��

    return  :	Rpy һ��ŷ���ǽṹ��
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
    purpose :	�õ���ǰ����ؽڽǶ�
    param   :	rshd �����ľ��

    return  :	vector<double> һ���ؽ�vector
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
