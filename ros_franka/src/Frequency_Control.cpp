#include"ros/ros.h"
#include"ros_franka/FrankaData.h"





int main(int argc, char *argv[])
{
    	ros::init(argc,argv,"FrankaFrequency");
    	ros::NodeHandle nh;
	float FREQ;
	nh.param<float>("/freq",FREQ,1000);
    	ros::Publisher pub1=nh.advertise<ros_franka::FrankaData>("FrankaFrequency",1000);

    	//发布的消息内容是无意义，用作控制频率
    	ros_franka::FrankaData data;

	//主程序发布频率以该频率定义的
	ros::Rate rate(FREQ);
	ROS_INFO("FREQ:       %f",FREQ);

	while(ros::ok())
	{

		for (int i = 0; i < 7; i++)
    		{
           		data.JointPos[i]=0;
    		}
    		for (int i = 0; i < 7; i++)
    		{
        		data.JointVel[i]=0;
    		}
    		for (int i = 0; i < 7; i++)
    		{
        		data.JointAcc[i]=0;
    		}
    		for (int i = 0; i < 7; i++)
    		{
        		data.Torque[i]=0;
    		}
    		for (int i = 0; i < 16; i++)
    		{
        		data.EndPos[i]=0;
   			}
    		for (int i = 0; i < 6; i++)
    		{
        		data.EndAcc[i]=0;
    		}
    		for (int i = 0; i < 7; i++)
    		{
        		data.MotorPos[i]=0;
    		}


		

    
		pub1.publish(data);
		rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
