#include "stdafx.h"
//#include "example.h"
//
//#define M_PI 3.14159265358979323846
//
//#define ROBOT_ADDR "192.168.1.10"
//#define ROBOT_PORT 8899
//
////控制柜用户ＤＩ名称
//const char* USER_DI_00 = "U_DI_00";
//const char* USER_DI_01 = "U_DI_01";
//const char* USER_DI_02 = "U_DI_02";
//const char* USER_DI_03 = "U_DI_03";
//const char* USER_DI_04 = "U_DI_04";
//const char* USER_DI_05 = "U_DI_05";
//const char* USER_DI_06 = "U_DI_06";
//const char* USER_DI_07 = "U_DI_07";
//const char* USER_DI_10 = "U_DI_10";
//const char* USER_DI_11 = "U_DI_11";
//const char* USER_DI_12 = "U_DI_12";
//const char* USER_DI_13 = "U_DI_13";
//const char* USER_DI_14 = "U_DI_14";
//const char* USER_DI_15 = "U_DI_15";
//const char* USER_DI_16 = "U_DI_16";
//const char* USER_DI_17 = "U_DI_17";
//
////控制柜用户ＤＯ名称
//const char* USER_DO_00 = "U_DO_00";
//const char* USER_DO_01 = "U_DO_01";
//const char* USER_DO_02 = "U_DO_02";
//const char* USER_DO_03 = "U_DO_03";
//const char* USER_DO_04 = "U_DO_04";
//const char* USER_DO_05 = "U_DO_05";
//const char* USER_DO_06 = "U_DO_06";
//const char* USER_DO_07 = "U_DO_07";
//const char* USER_DO_10 = "U_DO_10";
//const char* USER_DO_11 = "U_DO_11";
//const char* USER_DO_12 = "U_DO_12";
//const char* USER_DO_13 = "U_DO_13";
//const char* USER_DO_14 = "U_DO_14";
//const char* USER_DO_15 = "U_DO_15";
//const char* USER_DO_16 = "U_DO_16";
//const char* USER_DO_17 = "U_DO_17";
//
//const char* TOOL_IO_0 = "T_DI/O_00";
//const char* TOOL_IO_1 = "T_DI/O_01"	;
//const char* TOOL_IO_2 = "T_DI/O_02"	;
//const char* TOOL_IO_3 = "T_DI/O_03"	;
//
///********************************************************************
//	function:	printRoadPoint
//	purpose :	输出路点信息到标准输出
//	param   :	wayPoint 路点对象指针
//	return  :	void
//*********************************************************************/
//void printRoadPoint(const aubo_robot_namespace::wayPoint_S  *wayPoint)
//{
//	//输出坐标
//	std::cout<<"pos.x="<<wayPoint->cartPos.position.x<<std::endl;
//	std::cout<<"pos.y="<<wayPoint->cartPos.position.y<<std::endl;
//	std::cout<<"pos.z="<<wayPoint->cartPos.position.z<<std::endl;
//	//输出方向，由四元数表示
//	std::cout<<"ori.w="<<wayPoint->orientation.w<<std::endl;
//	std::cout<<"ori.x="<<wayPoint->orientation.x<<std::endl;
//	std::cout<<"ori.y="<<wayPoint->orientation.y<<std::endl;
//	std::cout<<"ori.z="<<wayPoint->orientation.z<<std::endl;
//	//关节角度
//	std::cout<<"joint_1="<<wayPoint->jointpos[0]*180.0/M_PI<<std::endl;
//	std::cout<<"joint_2="<<wayPoint->jointpos[1]*180.0/M_PI<<std::endl;
//	std::cout<<"joint_3="<<wayPoint->jointpos[2]*180.0/M_PI<<std::endl;
//	std::cout<<"joint_4="<<wayPoint->jointpos[3]*180.0/M_PI<<std::endl;
//	std::cout<<"joint_5="<<wayPoint->jointpos[4]*180.0/M_PI<<std::endl;
//	std::cout<<"joint_6="<<wayPoint->jointpos[5]*180.0/M_PI<<std::endl;
//}
//
///********************************************************************
//	function:	callback_RealTimeRoadPoint
//	purpose :	实时输出路点的回调函数
//	param   :	wayPoint 输出上下文句柄
//				arg
//	return  :	void
//*********************************************************************/
//void callback_RealTimeRoadPoint(const aubo_robot_namespace::wayPoint_S  *wayPoint, void *arg)
//{
//	printRoadPoint(wayPoint);
//}
//
///************************************************************************/
///*
//    function:	move_to
//	purpose :	机器人移动到指定位置
//	param   :	rshd 机器人服务句柄
//				pos  坐标指针
//				joint6Angle
//	return  :	bool 操作是否成功
//   pos 目标位置x,y,z 单位米
//   joint6Angle 6轴角度(度)
//*/
///************************************************************************/
//bool move_to(RSHD rshd, const Pos *pos, double joint6Angle)
//{
//	bool result = false;
//
//	//首先获取当前路点信息
//	aubo_robot_namespace::wayPoint_S wayPoint;
//
//	//逆解位置信息
//	aubo_robot_namespace::wayPoint_S targetPoint;
//
//	//目标位置对应的关节角
//	double targetRadian[ARM_DOF] = {0};
//	
//	if (RS_SUCC == rs_get_current_waypoint(rshd, &wayPoint)) // 获取当前位置
//	{
//		//参考当前姿态逆解得到六个关节角
//		if (RS_SUCC == rs_inverse_kin(rshd, wayPoint.jointpos, pos, &wayPoint.orientation, &targetPoint))
//		{
//			//将得到目标位置,将6关节角度设置为用户给定的角度（必须在+-175度）
//			targetRadian[0] = targetPoint.jointpos[0];
//			targetRadian[1] = targetPoint.jointpos[1];
//			targetRadian[2] = targetPoint.jointpos[2];
//			targetRadian[3] = targetPoint.jointpos[3];
//			targetRadian[4] = targetPoint.jointpos[4];
//			//6轴使用用户给定的关节角度
//			targetRadian[5] = joint6Angle/180*M_PI;
//
//			//轴动到目标位置
//			if (RS_SUCC == rs_move_joint(rshd, targetRadian))
//			{
//				std::cout<<"到达目标位置"<<std::endl;
//
//				//获取当前关节角，进行验证
//				rs_get_current_waypoint(rshd, &wayPoint);
//				//输出位置信息
//				printRoadPoint(&wayPoint);
//			}
//			else
//			{
//				std::cerr<<"move joint error 移动关节错误"<<std::endl;
//			}
//		}
//		else
//		{
//			std::cerr<<"ik failed 逆解关节失败"<<std::endl;
//		}
//
//	}
//	else
//	{
//		std::cerr<<"get current waypoint error"<<std::endl;
//	}
//
//	return result;
//}
//
///********************************************************************
//	function:	example_login
//	purpose :	登陆机械臂
//	param   :	rshd 输出上下文句柄
//				addr 机械臂服务器地址
//				port 机械臂服务器端口
//	return  :	true 成功 false 失败
//*********************************************************************/
//bool example_login(RSHD &rshd, const char * addr, int port)
//{
//	bool result = false;
//
//	rshd = RS_FAILED;
//
//	//初始化接口库
//	if (rs_initialize() == RS_SUCC)
//	{
//		//创建上下文
//		if (rs_create_context(&rshd)  == RS_SUCC )
//		{
//			//登陆机械臂服务器
//			if (rs_login(rshd, addr, port) == RS_SUCC)
//			{
//				result = true;
//				//登陆成功
//				std::cout<<"login succ"<<std::endl;
//			}
//			else
//			{
//				//登陆失败
//				std::cerr<<"login failed"<<std::endl;				
//			}
//		}
//		else
//		{
//			//创建上下文失败
//			std::cerr<<"rs_create_context error"<<std::endl;
//		}
//	}
//	else
//	{
//		//初始化接口库失败
//		std::cerr<<"rs_initialize error"<<std::endl;
//	}
//
//	return result;
//}
//
///********************************************************************
//	function:	example_logout
//	purpose :	退出登陆
//	param   :	rshd 上下文句柄
//					
//	return  :	true 成功 false 失败
//*********************************************************************/
//bool example_logout(RSHD rshd)
//{
//	return rs_logout(rshd)==RS_SUCC ? true : false;
//}
//
///********************************************************************
//	function:	example_robotStartup
//	purpose :	启动机械臂(必须连接真实机械臂）
//	param   :	rshd 上下文句柄
//					
//	return  :	true 成功 false 失败
//*********************************************************************/
//bool example_robotStartup(RSHD rshd)
//{
//	bool result = false;
//
//	//工具的动力学参数和运动学参数
//	ToolDynamicsParam tool_dynamics = {0};
//	//机械臂碰撞等级
//	uint8 colli_class = 6;
//	//机械臂启动是否读取姿态（默认开启）
//	bool read_pos = true;
//	//机械臂静态碰撞检测（默认开启）
//	bool static_colli_detect = true;
//	//机械臂最大加速度（系统自动控制，默认为30000)
//	int board_maxacc = 30000;
//	//机械臂服务启动状态
//	ROBOT_SERVICE_STATE state = ROBOT_SERVICE_READY;
//
//	if (rs_robot_startup(rshd, &tool_dynamics, colli_class, read_pos, static_colli_detect, board_maxacc, &state)
//		== RS_SUCC)
//	{
//		result = true;
//		std::cout<<"call robot startup succ, robot state:"<<state<<std::endl;
//	}
//	else
//	{
//		std::cerr<<"robot startup failed"<<std::endl;
//	}
//
//	return result;
//}
//
///********************************************************************
//	function:	example_robotShutdown
//	purpose :	关闭机械臂（必须连接真实机械臂）
//	param   :	rshd 上下文句柄
//					
//	return  :	true 成功 false 失败
//*********************************************************************/
//bool example_robotShutdown(RSHD rshd)
//{
//	return rs_robot_shutdown(rshd)==RS_SUCC ? true : false;
//}
//
///********************************************************************
//	function:	example_moveJ
//	purpose :	机械臂轴动测试
//	param   :	rshd 上下文句柄
//					
//	return  :	true 成功 false 失败
//*********************************************************************/
//bool example_moveJ(RSHD rshd)
//{
//	bool result = false;
//
//	RobotRecongnitionParam param;
//	rs_get_robot_recognition_param(rshd, 1, &param);
//
//	//该位置为机械臂的初始位置（提供6个关节角的关节信息（单位：弧度））
//	double initPos[6]={
//		-0.000172/180*M_PI,
//		-7.291862/180*M_PI,
//		-75.694718/180*M_PI,
//		21.596727/180*M_PI,
//		-89.999982/180*M_PI,
//		-0.00458/180*M_PI};
//
//	//首先运动到初始位置
//	if (rs_move_joint(rshd, initPos) == RS_SUCC)
//	{
//		result = true;
//		std::cout<<"movej succ"<<std::endl;
//	}
//	else
//	{
//		std::cerr<<"movej failed!"<<std::endl;
//	}
//
//	return result;
//}
//
///********************************************************************
//	function:	example_moveL
//	purpose :	机械臂保持当前姿态直线运动测试
//	param   :	rshd 上下文句柄
//					
//	return  :	true 成功 false 失败
//*********************************************************************/
//bool example_moveL(RSHD rshd)
//{
//	bool result = false;
//
//	//首先移动到初始位置
//	example_moveJ(rshd);
//
//	//获取当前路点信息
//	aubo_robot_namespace::wayPoint_S wayPoint;
//
//	//逆解位置信息
//	aubo_robot_namespace::wayPoint_S targetPoint;
//
//	//目标位置对应的关节角
//	double targetRadian[ARM_DOF] = {0};
//
//	//目标位置
//	Pos pos = {-0.489605, -0.155672, 0.448430};
//
//	if (RS_SUCC == rs_get_current_waypoint(rshd, &wayPoint))
//	{
//		//参考当前姿态逆解得到六个关节角
//		if (RS_SUCC == rs_inverse_kin(rshd, wayPoint.jointpos, &pos, &wayPoint.orientation, &targetPoint))
//		{
//			//将得到目标位置,将6关节角度设置为用户给定的角度（必须在+-175度）
//			targetRadian[0] = targetPoint.jointpos[0];
//			targetRadian[1] = targetPoint.jointpos[1];
//			targetRadian[2] = targetPoint.jointpos[2];
//			targetRadian[3] = targetPoint.jointpos[3];
//			targetRadian[4] = targetPoint.jointpos[4];
//			targetRadian[5] = targetPoint.jointpos[5];
//
//			//轴动到目标位置
//			if (RS_SUCC == rs_move_line(rshd, targetRadian))
//			{
//				std::cout<<"at target"<<std::endl;
//			}
//			else
//			{
//				std::cerr<<"move joint error"<<std::endl;
//			}
//		}
//		else
//		{
//			std::cerr<<"ik failed"<<std::endl;
//		}
//
//	}
//	else
//	{
//		std::cerr<<"get current waypoint error"<<std::endl;
//	}
//
//
//	return result;
//}
//
///********************************************************************
//	function:	example_moveP
//	purpose :	机械臂轨迹运动测试
//	param   :	rshd 上下文句柄
//					
//	return  :	void
//*********************************************************************/
//void example_moveP(RSHD rshd)
//{
//	/** 模拟业务 **/
//	/** 接口调用: 初始化运动属性 ***/
//	rs_init_global_move_profile(rshd);
//
//	/** 接口调用: 设置关节型运动的最大加速度 ***/
//	aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
//	jointMaxAcc.jointPara[0] = 50.0/180.0*M_PI;
//	jointMaxAcc.jointPara[1] = 50.0/180.0*M_PI;
//	jointMaxAcc.jointPara[2] = 50.0/180.0*M_PI;
//	jointMaxAcc.jointPara[3] = 50.0/180.0*M_PI;
//	jointMaxAcc.jointPara[4] = 50.0/180.0*M_PI;
//	jointMaxAcc.jointPara[5] = 50.0/180.0*M_PI;   //接口要求单位是弧度
//	rs_set_global_joint_maxacc(rshd, &jointMaxAcc);
//
//	/** 接口调用: 设置关节型运动的最大速度 ***/
//	aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
//	jointMaxVelc.jointPara[0] = 50.0/180.0*M_PI;
//	jointMaxVelc.jointPara[1] = 50.0/180.0*M_PI;
//	jointMaxVelc.jointPara[2] = 50.0/180.0*M_PI;
//	jointMaxVelc.jointPara[3] = 50.0/180.0*M_PI;
//	jointMaxVelc.jointPara[4] = 50.0/180.0*M_PI;
//	jointMaxVelc.jointPara[5] = 50.0/180.0*M_PI;   //接口要求单位是弧度
//	rs_set_global_joint_maxvelc(rshd, &jointMaxVelc);
//
//
//	/** 接口调用: 初始化运动属性 ***/
//	rs_init_global_move_profile(rshd);
//
//	/** 接口调用: 设置末端型运动的最大加速度 　　直线运动属于末端型运动***/
//	double endMoveMaxAcc;
//	endMoveMaxAcc = 0.2;   //单位米每秒
//	rs_set_global_end_max_line_acc(rshd, endMoveMaxAcc);
//	rs_set_global_end_max_angle_acc(rshd, endMoveMaxAcc);
//
//
//	/** 接口调用: 设置末端型运动的最大速度 直线运动属于末端型运动***/
//	double endMoveMaxVelc;
//	endMoveMaxVelc = 0.2;   //单位米每秒
//	rs_set_global_end_max_line_velc(rshd, endMoveMaxVelc);
//	rs_set_global_end_max_angle_velc(rshd, endMoveMaxVelc);
//
//	double jointAngle[aubo_robot_namespace::ARM_DOF] = {0};
//
//	for(int i=0;i<2;i++)
//	{
//		//准备点  关节运动属于关节型运动
//		rs_init_global_move_profile(rshd);
//		rs_set_global_joint_maxacc(rshd, &jointMaxAcc);
//		rs_set_global_joint_maxvelc(rshd, &jointMaxVelc);
//
//		//关节运动至准备点
//		jointAngle[0] = -0.000003;
//		jointAngle[1] = -0.127267;
//		jointAngle[2] = -1.321122;
//		jointAngle[3] = 0.376934;
//		jointAngle[4] = -1.570796;
//		jointAngle[5] = -0.000008;
//
//		int ret = rs_move_joint(rshd, jointAngle);
//		if(ret != RS_SUCC)
//		{
//			std::cerr<<"JointMove失败.　ret:"<<ret<<std::endl;
//		}
//
//		//圆弧
//		rs_init_global_move_profile(rshd);
//
//		rs_set_global_end_max_line_acc(rshd, endMoveMaxAcc);
//		rs_set_global_end_max_angle_acc(rshd, endMoveMaxAcc);
//		rs_set_global_end_max_line_velc(rshd, endMoveMaxVelc);
//		rs_set_global_end_max_angle_velc(rshd, endMoveMaxVelc);
//
//		jointAngle[0] = -0.000003;
//		jointAngle[1] = -0.127267;
//		jointAngle[2] = -1.321122;
//		jointAngle[3] = 0.376934;
//		jointAngle[4] = -1.570796;
//		jointAngle[5] = -0.000008;
//		rs_add_waypoint(rshd, jointAngle);
//
//		jointAngle[0] = 0.200000;
//		jointAngle[1] = -0.127267;
//		jointAngle[2] = -1.321122;
//		jointAngle[3] = 0.376934;
//		jointAngle[4] = -1.570794;
//		jointAngle[5] = -0.000008;
//		rs_add_waypoint(rshd, jointAngle);
//
//		jointAngle[0] = 0.600000;
//		jointAngle[1] = -0.127267;
//		jointAngle[2] = -1.321122;
//		jointAngle[3] = 0.376934;
//		jointAngle[4] = -1.570796;
//		jointAngle[5] = -0.000008;
//		rs_add_waypoint(rshd, jointAngle);
//
//		rs_set_circular_loop_times(rshd, 0);
//		if(RS_SUCC !=rs_move_track(rshd, ARC_CIR))
//		{
//			std::cerr<<"TrackMove failed.　ret:"<<ret<<std::endl;
//		}
//
//		//准备点
//		rs_init_global_move_profile(rshd);
//		rs_set_global_joint_maxacc(rshd, &jointMaxAcc);
//		rs_set_global_joint_maxvelc(rshd, &jointMaxVelc);
//
//		jointAngle[0] = -0.000003;
//		jointAngle[1] = -0.127267;
//		jointAngle[2] = -1.321122;
//		jointAngle[3] = 0.376934;
//		jointAngle[4] = -1.570796;
//		jointAngle[5] = -0.000008;
//
//		//关节运动至准备点
//		ret = rs_move_joint(rshd, jointAngle);
//		if(RS_SUCC != ret)
//		{
//			std::cerr<<"JointMove失败.　ret:"<<ret<<std::endl;
//		}
//
//		//圆
//		rs_init_global_move_profile(rshd);
//
//		rs_set_global_end_max_line_acc(rshd, endMoveMaxAcc);
//		rs_set_global_end_max_angle_acc(rshd, endMoveMaxAcc);
//		rs_set_global_end_max_line_velc(rshd, endMoveMaxVelc);
//		rs_set_global_end_max_angle_velc(rshd, endMoveMaxVelc);
//
//		jointAngle[0] = -0.000003;
//		jointAngle[1] = -0.127267;
//		jointAngle[2] = -1.321122;
//		jointAngle[3] = 0.376934;
//		jointAngle[4] = -1.570796;
//		jointAngle[5] = -0.000008;
//		rs_add_waypoint(rshd, jointAngle);
//
//		jointAngle[0] = -0.211675;
//		jointAngle[1] = -0.325189;
//		jointAngle[2] = -1.466753;
//		jointAngle[3] = 0.429232;
//		jointAngle[4] = -1.570794;
//		jointAngle[5] = -0.211680;
//		rs_add_waypoint(rshd, jointAngle);
//
//		jointAngle[0] = -0.037186;
//		jointAngle[1] = -0.224307;
//		jointAngle[2] = -1.398285;
//		jointAngle[3] = 0.396819;
//		jointAngle[4] = -1.570796;
//		jointAngle[5] = -0.037191;
//		rs_add_waypoint(rshd, jointAngle);
//
//		//圆的圈数
//		rs_set_circular_loop_times(rshd, 1);
//		ret = rs_move_track(rshd, ARC_CIR);
//		if(RS_SUCC != ret)
//		{
//			std::cerr<<"TrackMove failed.　ret:"<<ret<<std::endl;
//		}
//
//
//		//准备点
//		rs_init_global_move_profile(rshd);
//
//		rs_set_global_joint_maxacc(rshd, &jointMaxAcc);
//		rs_set_global_joint_maxvelc(rshd, &jointMaxVelc);
//
//		jointAngle[0] = -0.000003;
//		jointAngle[1] = -0.127267;
//		jointAngle[2] = -1.321122;
//		jointAngle[3] = 0.376934;
//		jointAngle[4] = -1.570796;
//		jointAngle[5] = -0.000008;
//
//		//关节运动至准备点
//		if(RS_SUCC != rs_move_joint(rshd, jointAngle))
//		{
//			std::cerr<<"JointMove失败.　ret:"<<ret<<std::endl;
//		}
//
//		//MoveP
//		rs_init_global_move_profile(rshd);
//
//		rs_set_global_end_max_line_acc(rshd, endMoveMaxAcc);
//		rs_set_global_end_max_angle_acc(rshd, endMoveMaxAcc);
//		rs_set_global_end_max_line_velc(rshd, endMoveMaxVelc);
//		rs_set_global_end_max_angle_velc(rshd, endMoveMaxVelc);
//
//
//		jointAngle[0] = -0.000003;
//		jointAngle[1] = -0.127267;
//		jointAngle[2] = -1.321122;
//		jointAngle[3] = 0.376934;
//		jointAngle[4] = -1.570796;
//		jointAngle[5] = -0.000008;
//		rs_add_waypoint(rshd, jointAngle);
//
//		jointAngle[0] = 0.100000;
//		jointAngle[1] = -0.147267;
//		jointAngle[2] = -1.321122;
//		jointAngle[3] = 0.376934;
//		jointAngle[4] = -1.570794;
//		jointAngle[5] = -0.000008;
//		rs_add_waypoint(rshd, jointAngle);
//
//		jointAngle[0] = 0.200000;
//		jointAngle[1] = -0.167267;
//		jointAngle[2] = -1.321122;
//		jointAngle[3] = 0.376934;
//		jointAngle[4] = -1.570794;
//		jointAngle[5] = -0.000008;
//		rs_add_waypoint(rshd, jointAngle);
//
//		//交融半径
//		rs_set_blend_radius(rshd, 0.03);
//		rs_set_circular_loop_times(rshd, 1);
//		if(RS_SUCC !=rs_move_track(rshd, CARTESIAN_MOVEP))
//		{
//			std::cerr<<"TrackMove failed.　ret:"<<ret<<std::endl;
//		}
//	}
//}
//
///********************************************************************
//	function:	example_ik_fk
//	purpose :	机械臂正逆解测试
//	param   :	
//					
//	return  :	
//*********************************************************************/
//void example_ik_fk(RSHD rshd)
//{
//	aubo_robot_namespace::wayPoint_S wayPoint;
//
//	ik_solutions solutions;
//
//	double jointAngle[aubo_robot_namespace::ARM_DOF] = {-0.000003, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008};
//	
//	//正解
//	if (RS_SUCC == rs_forward_kin(rshd, jointAngle, &wayPoint))
//	{
//		std::cout<<"fk succ"<<std::endl;
//
//		printRoadPoint(&wayPoint);
//	}
//
//	//逆解
//	double startPointJointAngle[aubo_robot_namespace::ARM_DOF] = {0.0/180.0*M_PI,  0.0/180.0*M_PI,  0.0/180.0*M_PI, 0.0/180.0*M_PI, 0.0/180.0*M_PI,0.0/180.0*M_PI};
//
//	aubo_robot_namespace::Pos targetPosition;
//	targetPosition.x =-0.400;
//	targetPosition.y =-0.1215;
//	targetPosition.z = 0.5476;
//
//	aubo_robot_namespace::Rpy rpy;
//	aubo_robot_namespace::Ori targetOri;
//
//	rpy.rx = 180.0/180.0*M_PI;
//	rpy.ry = 0.0/180.0*M_PI;
//	rpy.rz = -90.0/180.0*M_PI;
//
//	rs_rpy_to_quaternion(rshd, &rpy, &targetOri);
//
//	if (RS_SUCC == rs_inverse_kin(rshd, startPointJointAngle, &targetPosition, &targetOri, &wayPoint))
//	{
//		std::cout<<"ik succ"<<std::endl;
//		printRoadPoint(&wayPoint);
//	}
//	else
//	{
//		std::cerr<<"ik failed"<<std::endl;
//	}
//
//	if (RS_SUCC == rs_inverse_kin_closed_form(rshd, &targetPosition, &targetOri, &solutions))
//	{
//		std::cout<<"ik succ"<<std::endl;
//		for (int i=0;i<solutions.solution_count;i++)
//		{
//			printRoadPoint(&solutions.waypoint[i]);
//		}
//	}
//	else
//	{
//		std::cerr<<"ik failed"<<std::endl;
//	}
//}
//
///********************************************************************
//	function:	example_boardIO
//	purpose :	机械臂控制柜IO测试(必须连接真实机械臂）
//	param   :	rshd 上下文句柄
//					
//	return  :	void
//*********************************************************************/
//void example_boardIO(RSHD rshd)
//{
//	double status = 0;
//
//	if (RS_SUCC == rs_set_board_io_status_by_name(rshd, RobotBoardUserDO, USER_DO_00, IO_STATUS_VALID))
//	{
//		std::cout<<"set "<<USER_DO_00<<" succ"<<std::endl;
//
//		if (RS_SUCC == rs_get_board_io_status_by_name(rshd, RobotBoardUserDO, USER_DO_00, &status))
//		{
//			std::cout<<"get "<<USER_DO_00<<"="<<status<<std::endl;
//		}
//		else
//		{
//			std::cerr<<"get "<<USER_DO_00<<" failed"<<std::endl;
//		}
//	}
//	else
//	{
//		std::cerr<<"set "<<USER_DO_00<<" failed"<<std::endl;
//	}
//}
//
////机械臂工具端IO测试(必须连接真实机械臂）
//void example_ToolIO(RSHD rshd)
//{
//	double status = 0;
//
//	//首先设置tool_io_0为数字输出
//	if (RS_SUCC == rs_set_tool_io_type(rshd, TOOL_DIGITAL_IO_0, IO_OUT))
//	{
//		//设置tool_io_0数字输出为有效
//		if (RS_SUCC == rs_set_tool_do_status(rshd, TOOL_IO_0, IO_STATUS_VALID))
//		{
//			std::cout<<"set "<<TOOL_IO_0<<" succ"<<std::endl;
//		}
//		else
//		{
//			std::cerr<<"set "<<TOOL_IO_0<<" failed"<<std::endl;
//		}
//		
//		//获取tool_io_0数字输出的状态
//		if (RS_SUCC == rs_get_tool_io_status(rshd, TOOL_IO_0, &status))
//		{
//			std::cout<<"get "<<TOOL_IO_0<<"="<<status<<std::endl;
//		}
//		else
//		{
//			std::cerr<<"get "<<TOOL_IO_0<<" failed"<<std::endl;
//		}
//	}
//}
//
///********************************************************************
//	function:	example_callbackRobotRoadPoint
//	purpose :	实时路点信息回调函数测试
//	param   :	rshd 上下文句柄
//					
//	return  :	true 成功 false 失败
//*********************************************************************/
//bool example_callbackRobotRoadPoint(RSHD rshd)
//{
//	bool result = false;
//
//	//允许实时路点信息推送
//	if (RS_SUCC == rs_enable_push_realtime_roadpoint(rshd, true))
//	{
//		if (RS_SUCC == rs_setcallback_realtime_roadpoint(rshd, callback_RealTimeRoadPoint, NULL))
//		{
//			result = true;
//		}
//		else
//		{
//			std::cerr<<"call rs_setcallback_realtime_roadpoint failed"<<std::endl;
//		}
//	}
//	else
//		std::cerr<<"call rs_enable_push_realtime_roadpoint failed!"<<std::endl;
//
//	return result;
//}
//
//void example_get_diagnosis(RSHD rshd)
//{
//	RobotDiagnosis *info = new RobotDiagnosis;  
//
//	while(1)
//	{
//		memset((void*)info, 0, sizeof(RobotDiagnosis));  
//
//		rs_get_diagnosis_info(rshd, info);
//
//		std::cout<<"info.macTargetPosBufferSize="<<info->macTargetPosBufferSize<<" info.macTargetPosDataSize="<<info->macTargetPosDataSize<<std::endl;
//
//		Sleep(500);
//	}
//}