#include <cmath>
#include <iostream>
#include <pthread.h>
#include"ros/ros.h"
#include"ros_haption/HaptionData.h"

#include <time.h>
#include <unistd.h>
//#include "VirtuoseAPI.h"

#include "HaptionRobot.h"

int num=0;

class Haption
{
public:
	Haption()
	{
                 //sub1和pub1的话题名称需要根据实际的话题进行更改，sub2的话题名称不能更改

		sub1 = n.subscribe("HaptionPosAndSpeed", 1000, &Haption::Callback,this); //接收外部节点位置数据的订阅方
		sub2 = n.subscribe("HaptionFrequency", 1000, &Haption::Callback1,this); //接收自定义的发布频率
		pub1 = n.advertise<ros_haption::HaptionData>("DataFromHaption",1000); //发布Franka的节点位置数据的发布方
        
	};

	void Callback(const ros_haption::HaptionData &data)	//接收外部数据的回调函数
	{
			VH.startGetPos=1;
			k = VH.CyclesNum;		//计入循环次数

			for(float i=1.0f; i<=k; i++)
			{
				for (int j = 0; j <7; j++)
				{
					double tag = (1-i/k)*PosArray[j]  + i/k * data.Position[j];
           			VH.EndPos_1.push(tag);
				}
                for (int j = 0; j <6; j++)
				{
					double tag1 = (1-i/k)*SpeedArray[j]  + i/k * data.Speed[j];
           			VH.EndVel_1.push(tag1);
				}
			}

		for(int i=0;i<7; i++)
		{
			PosArray[i] = data.Position[i];
		}
        for(int i=0;i<6; i++)
		{
			SpeedArray[i] = data.Speed[i];
		}

	};   

	void Callback1(const ros_haption::HaptionData &data)   //以订阅的频率控制发布的频率
	{

        	//Haption数据类型实例化
		ros_haption::HaptionData haptiondata;

        	//将Haption自身数据传递给将要发布的消息
       		for (int i = 0; i < 7; i++)
        	{
              		haptiondata.Position[i]=VH.M_Data.Position[i];
        	}
        	for (int i = 0; i < 6; i++)
        	{
              		haptiondata.Force[i]=VH.M_Data.Force[i];
        	}
        	for (int i = 0; i < 6; i++)
        	{
              		haptiondata.Speed[i]=VH.M_Data.Speed[i];
        	}
        	for (int i = 0; i < 6; i++)
        	{
              		haptiondata.ArticularPosition[i]=VH.M_Data.Position1[i];
        	}
        	for (int i = 0; i < 6; i++)
        	{
              		haptiondata.ArticularSpeed[i]=VH.M_Data.Speed1[i];
        	}
			for(int i=0;i<3;i++)
			{
				haptiondata.ButtonState[i] = VH.M_Data.Buttonstate[i];
			}

        	pub1.publish(haptiondata);//发布消息
        }
     
public:
	std::array<float, 7> PosArray{};
    std::array<float, 6> SpeedArray{};
	float k;
	ros::NodeHandle n;  
	ros::Subscriber sub1; 
	ros::Subscriber sub2;
	ros::Publisher pub1; 
	VirHaptions VH;
};
 


 
int main(int argc, char** argv)
 {

	//初始化ros节点
	ros::init(argc, argv, "HaptionPosAndSpeedControl");

	Haption ha;

	//初始化数据
	ha.VH.HaptionInit();

	//设置控制模式
    //ha.VH.setCommandtype(COMMAND_TYPE_VIRTMECH);
	virtSetCommandType(ha.VH.M_VC, COMMAND_TYPE_VIRTMECH);  

	ha.VH.setfrequency(1000);   //频率设置

	std::cout  << "   Press Enter to start Control...   " << std::endl;
	std::cin.ignore();

	ha.VH.start_admittance_pth();
  
	ros::spin();
	//关闭haption
        
	ha.VH.HaptionClose();

	std::cout  << "    control finish!    " << std::endl;

 	return 0;
 }
