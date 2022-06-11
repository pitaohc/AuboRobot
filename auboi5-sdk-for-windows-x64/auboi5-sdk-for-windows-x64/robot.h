#pragma once
#include <rsdef.h>
#include <queue>
#include <fstream>
#include <string>
#include <vector>
//#include <utility>
constexpr char ROBOT_ADDR[] = "192.168.1.10"; //机器人ip地址
constexpr unsigned int ROBOT_PORT = 8899; //机器人端口
constexpr double M_PI = 3.14159265358979323846; //派


bool login(RSHD& rshd, const char* addr, int port); //登录
bool logout(RSHD rshd); //注销
bool robotStartup(RSHD rshd); //启动
bool robotShutdown(RSHD rshd);//停止
void moveTest(RSHD rshd, Pos biasPos = { 0.0,0.0,0.0 }, Rpy biasRpy = { 0.0,0.0,0.0 });
std::vector<wayPoint_S> setWayPointVector(RSHD rshd, std::string filepath, bool hasHeader = true);//设置移动路径
wayPoint_S getWayPoint(RSHD rshd); //得到路点
Pos getPos(RSHD rshd); //得到坐标
Ori getOri(RSHD rshd); //得到四元数
Rpy getRpy(RSHD rshd); //得到欧拉角
std::vector<double> getJoint(RSHD rshd); //得到关节值