#include"ros/ros.h"
#include"ros_franka/FrankaData.h"
#include"ros_franka/FrankaJointPos.h"


int main(int argc, char *argv[])
{
    ros::init(argc,argv,"JointPub");
    ros::NodeHandle nh;
    ros::Publisher pub2=nh.advertise<ros_franka::FrankaJointPos>("JointPos",10);


    ros_franka::FrankaJointPos pub;
    for (int i = 0; i < 7; i++)
    {
           pub.JointPos[i]=0;
    }




	int num =0;
	double times = 0;
	ros::Rate rate(500);

	while(num <= 3000)
	{
		num++;

		if(num>=500)
		{
			times = (num-500) * 0.002;

        		double delta_angle = M_PI / 8.0 * (1 - std::cos(M_PI / 2.5 * times));

			//当前机器人7个角度值填入这里	
			pub.JointPos[0]=0+ delta_angle;		//第一轴
			pub.JointPos[1]=0;
			pub.JointPos[2]=0;
			pub.JointPos[3]=0;
			pub.JointPos[4]=0;
			pub.JointPos[5]=0;	//这是第六轴，角度值后+num，例如M_PI_2+num。
			pub.JointPos[6]=0 ;
		std::cout<<delta_angle<<std::endl;
		}


    
		pub2.publish(pub);
		rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
