#pragma once
#include <rsdef.h>
#include <queue>
#include <fstream>
#include <string>
#include <vector>
//#include <utility>
constexpr char ROBOT_ADDR[] = "192.168.1.10"; //������ip��ַ
constexpr unsigned int ROBOT_PORT = 8899; //�����˶˿�
constexpr double M_PI = 3.14159265358979323846; //��


bool login(RSHD& rshd, const char* addr, int port); //��¼
bool logout(RSHD rshd); //ע��
bool robotStartup(RSHD rshd); //����
bool robotShutdown(RSHD rshd);//ֹͣ
void moveTest(RSHD rshd, Pos biasPos = { 0.0,0.0,0.0 }, Rpy biasRpy = { 0.0,0.0,0.0 });
std::vector<wayPoint_S> setWayPointVector(RSHD rshd, std::string filepath, bool hasHeader = true);//�����ƶ�·��
wayPoint_S getWayPoint(RSHD rshd); //�õ�·��
Pos getPos(RSHD rshd); //�õ�����
Ori getOri(RSHD rshd); //�õ���Ԫ��
Rpy getRpy(RSHD rshd); //�õ�ŷ����
std::vector<double> getJoint(RSHD rshd); //�õ��ؽ�ֵ