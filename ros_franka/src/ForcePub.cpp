#include"ros/ros.h"
#include"ros_franka/FrankaData.h"
#include"ros_franka/FrankaForce.h"


int main(int argc, char *argv[])
{
    ros::init(argc,argv,"ForcePub");
    ros::NodeHandle nh;
    ros::Publisher pub2=nh.advertise<ros_franka::FrankaForce>("frocePos",10);


    ros_franka::FrankaForce pub;
    for (int i = 0; i < 7; i++)
    {
           pub.Torque[i]=0;
    }

	int num =0;
	double times = 0;
	ros::Rate rate(1000);

	while(num <= 5500)
	{
		num++;

		if(num>=500)
		{
			times = (num-500) * 0.001;

        		double delta_angle = 1 * (1 - std::cos(M_PI / 2.5 * times));

			//当前机器人7个角度值填入这里	
			pub.Torque[0]=0+ delta_angle;		//第一轴
			pub.Torque[1]=0;
			pub.Torque[2]=0;
			pub.Torque[3]=0;
			pub.Torque[4]=0;
			pub.Torque[5]=0;	
			pub.Torque[6]=0 ;
		}

    
		pub2.publish(pub);
		rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
