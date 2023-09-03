#include <iostream>
#include <unistd.h>
#include <pthread.h>
#include <queue>
#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"


struct RobotData {
  /**
   * \f$^{O}T_{EE}\f$
   * //姿势以列优先格式表示为 4x4 矩阵
   * 
   */
	std::array<double, 16> EndPos{}; //O_T_EE{};  // NOLINT(readability-identifier-naming)
	std::array<double, 6> Pos{};   //方便转换
  /**
   * \f$\tau_{J}\f$
   * Measured link-side joint torque sensor signals. Unit: \f$[Nm]\f$
   */
	std::array<double, 7> Torque{}; //tau_J{};  // NOLINT(readability-identifier-naming)

  /**
   * \f$q\f$
   * //关节角度. Unit: \f$[rad]\f$
   */
	std::array<double, 7> JointPos{}; //q{};

  /**
   * \f$\dot{q}\f$
   * //关节速度 Unit: \f$[\frac{rad}{s}]\f$
   */
	std::array<double, 7> JointVel{};//dq{};

  /**
   * \f$\ddot{q}_d\f$
   * //加速度（非测量值） Unit: \f$[\frac{rad}{s^2}]\f$
   */
	std::array<double, 7> JointAcc{};//ddq_d{};

  /**
   * \f${^OddP_{EE}}_{c}\f$
   * //最后一个指令末端执行器加速度
   * Unit:
   * \f$[\frac{m}{s^2},\frac{m}{s^2},\frac{m}{s^2},\frac{rad}{s^2},\frac{rad}{s^2},\frac{rad}{s^2}]\f$.
   */
	std::array<double, 6> EndAcc{};//O_ddP_EE_c{};  // NOLINT(readability-identifier-naming)

  /**
   * \f$\theta\f$
   * //电机位置Unit: \f$[rad]\f$
   */
	std::array<double, 7> MotorPos{};//theta{};

//末端速度
	std::array<double, 6> EndVel{};

};

class FrankaRobot
{
public:
	FrankaRobot(const std::string& franka_address);
//	~FrankaRobot();

public:

//关节角控制
	franka::JointPositions JoinPos_control(
                            const franka::RobotState& robot_state, franka::Duration period);

	std::function<franka::JointPositions(const franka::RobotState&, franka::Duration)> 
                           JoinPos_control_callback =
                          std::bind(&FrankaRobot::JoinPos_control,this, std::placeholders::_1, std::placeholders::_2);

	void start_JointPos_pth(pthread_t* thread);  //

//力矩控制
	franka::Torques force_control(
                            const franka::RobotState& robot_state, franka::Duration period);
 
	std::function<franka::Torques(const franka::RobotState&, franka::Duration)> 
                            force_control_callback =
                            std::bind(&FrankaRobot::force_control,this, std::placeholders::_1, std::placeholders::_2);
 
	void start_Force_pth(pthread_t* thread);  //

//笛卡尔控制
	franka::CartesianPose CartPos_control(
                            const franka::RobotState& robot_state, franka::Duration period);

	std::function<franka::CartesianPose(const franka::RobotState&, franka::Duration)>
                            CartPos_control_callback =
                           std::bind(&FrankaRobot::CartPos_control,this, std::placeholders::_1, std::placeholders::_2);

	void start_Pos_pth(pthread_t* thread);  //

//末端速度控制
	franka::CartesianVelocities CartVel_control(
                            const franka::RobotState& robot_state, franka::Duration period);

	std::function<franka::CartesianVelocities(const franka::RobotState&, franka::Duration)>
                            CartVel_control_callback =
                          std::bind(&FrankaRobot::CartVel_control,this, std::placeholders::_1, std::placeholders::_2);

	void start_CarVel_pth(pthread_t* thread);  //

  //关节速度控制
	franka::JointVelocities JointVel_control(
                            const franka::RobotState& robot_state, franka::Duration period);

	std::function<franka::JointVelocities(const franka::RobotState&, franka::Duration)> 
                           JointVel_control_callback =
                          std::bind(&FrankaRobot::JointVel_control,this, std::placeholders::_1, std::placeholders::_2);

	void start_JointVel_pth(pthread_t* thread); //


	void start_ForceAndJointPos_pth(pthread_t* thread);  //

	void start_ForceAndJointVel_pth(pthread_t* thread);  

	void start_ForceAndPos_pth(pthread_t* thread);  

	void start_ForceAndCarVel_pth(pthread_t* thread); 


public:

	void robot_JoinPos_init();

	void robot_force_init();

	void robot_CartPos_init();

	void robot_CartVel_init();

	void robot_JointVel_init();
	

	void getfrankadate(franka::RobotState state);

	//void initialSendData(franka::RobotState state);

	void GetRead_pth();

	void robot_close();

	void setfrequency(float num);

	franka::Torques zero_torques{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
	franka::CartesianVelocities zero_carvel{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
	franka::JointVelocities zero_jointvel{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

	std::queue<double>joint_1;
	std::queue<double>force_1;
	std::queue<double>CartPos_1;
	std::queue<double>CartVel_1;
	std::queue<double>JointVel_1;

	int startGetPos;
	bool motion_finished;
	bool STATE;             //关闭状态 置1关闭
	double times;       //循环前需要置0；
	double times_1;

private:


	std::array<double, 7> goal_JointPos{};
	std::array<double, 7> initial_JointPos{};


	std::array<double, 7>goal_force{};
   
	std::array<double, 16> goal_Pos{};          //目标姿态矩阵
	std::array<double, 16> initial_Pos{};       //初始姿态矩阵


	std::array<double, 6> goal_CartVel{};   

	std::array<double, 7> goal_jointVel{};
 


	void RotMatrixToEuler(std::array<double,16> &Rot ,std::array<double,6> &Euler);
	void EulerToRotMatrix(std::array<double,16> &Rot ,std::array<double,6> &Euler);


public:

	int counter;		//循环计数；

	float FREQUENCY;	//自定义频率
	float CyclesNum;		//计数周期   1000/频率取整数

	RobotData m_robotdata;	//获取机器人端数据

	franka::Robot  m_robot;	//机器人API接口

};



