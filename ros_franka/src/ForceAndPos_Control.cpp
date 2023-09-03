#include <cmath>
#include <iostream>
#include <pthread.h>
#include"ros/ros.h"
#include"ros_franka/FrankaData.h"
#include"ros_franka/FrankaForceAndPos.h"
#include <franka/exception.h>
#include <franka/robot.h>
#include "examples_common.h"
#include "FrankaRobot.h"

class Panda
{
public:
	Panda(const std::string& franka_address):Robot1(franka_address)
	{
		//sub1和pub1的话题名称需要根据实际的话题进行更改，sub2的话题名称不能更改

		sub1 = n.subscribe("ForceAndPos", 10, &Panda::Callback,this);		//接收外部节点位置数据的订阅方
		sub2 = n.subscribe("FrankaFrequency", 10, &Panda::Callback1,this);	//接收自定义的发布频率
		pub1 = n.advertise<ros_franka::FrankaData>("DataFromFranka",10);	//发布Franka的节点位置数据的发布方
        
	};

	void Callback(const ros_franka::FrankaForceAndPos &data)				//接收外部数据的回调函数
	{
		Robot1.startGetPos=1;
		k = Robot1.CyclesNum;		//计入循环次数

		for(float i=1.0f; i<=k; i++)
		{
			for (int j = 0; j < 7; j++)
			{
					double tag = (1-i/k)*ForceArray[j]  + i/k * data.Torque[j];
           			Robot1.force_1.push(tag);
			}
			for (int j = 0; j < 16; j++)
			{
					double tag = (1-i/k)*CartPosArray[j]  + i/k * data.EndPos[j];
           			Robot1.CartPos_1.push(tag);
			}
		}

		for(int i=0;i<7; i++)
		{
			ForceArray[i] = data.Torque[i];
		}
		for(int i=0;i<16; i++)
		{
			CartPosArray[i] = data.EndPos[i];
		}
	   
	};   

	void Callback1(const ros_franka::FrankaData &data)				//以订阅的频率控制发布的频率
	{
        	//组织被发布的数据
		ros_franka::FrankaData PubData;

        	//将Franka自身数据传递给将要发布的消息
        	for (int i = 0; i < 7; i++)
        	{
        	      PubData.JointPos[i]=Robot1.m_robotdata.JointPos[i];
        	}
        	for (int i = 0; i < 7; i++)
        	{
         	     PubData.JointVel[i]=Robot1.m_robotdata.JointVel[i];
        	}
        	for (int i = 0; i < 7; i++)
        	{
        	      PubData.JointAcc[i]=Robot1.m_robotdata.JointAcc[i];
        	}
        	for (int i = 0; i < 7; i++)
       		{
       		      PubData.Torque[i]=Robot1.m_robotdata.Torque[i];
        	}
        	for (int i = 0; i < 16; i++)
        	{
          	    PubData.EndPos[i]=Robot1.m_robotdata.Pos[i];
        	}
        	for (int i = 0; i < 6; i++)
        	{
        	      PubData.EndAcc[i]=Robot1.m_robotdata.EndAcc[i];
        	}
        	for (int i = 0; i < 7; i++)
        	{
        	      PubData.MotorPos[i]=Robot1.m_robotdata.MotorPos[i];
        	}

        	pub1.publish(PubData);							//发布消息

        }
     
public:
	std::array<double, 16> CartPosArray{};
	std::array<double, 7> ForceArray{};
	float k;
	ros::NodeHandle n;  
	ros::Subscriber sub1; 
	ros::Subscriber sub2;
	ros::Publisher pub1; 
	FrankaRobot Robot1;
};
 


 
int main(int argc, char** argv)
{

	pthread_t thread;
	//初始化ros节点
	ros::init(argc, argv, "ForceAndPos_Control");

	//实例化,输入IP地址
	Panda pl("192.168.100.63");	

	//初始化数据
	pl.Robot1.robot_force_init();

	std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};

	MotionGenerator motion_generator(0.5, q_goal);

	pl.Robot1.m_robot.control(motion_generator);     


	float FREQ; 
	pl.n.param<float>("/freq",FREQ,1000);
	pl.Robot1.setfrequency(FREQ);//接收频率为10；该数据要与发送数据给机器人的频率要相同

	pl.Robot1.GetRead_pth();							//机器人内部参数更新位置

	std::cout  << "control ..." << std::endl;

	//更新为多线程调用	开启控制
	pl.Robot1.start_ForceAndPos_pth(&thread);

	ros::AsyncSpinner spinner(3); // Use 3 threads
	spinner.start();
	ros::waitForShutdown();

	pl.Robot1.STATE = true;								//结束运动

	pthread_join(thread,NULL);

	return 0;
}