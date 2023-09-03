#include"ros/ros.h"
#include"ros_franka/FrankaData.h"
#include"ros_franka/FrankaForceAndJointVel.h"


int main(int argc, char *argv[])
{
    ros::init(argc,argv,"JointVelPub");
    ros::NodeHandle nh;
    ros::Publisher pub2=nh.advertise<ros_franka::FrankaForceAndJointVel>("ForceAndJointVel",10);


    ros_franka::FrankaForceAndJointVel pub;
    for (int i = 0; i < 7; i++)
    {
            pub.Torque[i]=0;     
           pub.JointVel[i]=0;
    }

    double time_max = 1.0;
    double omega_max = 1.0;
	int num =0;
	double times = 0;
	ros::Rate rate(1000);

	while(num <= 2500)
	{
		num++;
		if(num>=500)
		{
			times = (num-500) * 0.001;

        	double cycle = std::floor(std::pow(-1.0, (times - std::fmod(times, time_max)) / time_max));
          double omega = cycle * omega_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * times));

        		double delta_angle = 1 * (1 - std::cos(M_PI / 2.5 * times));

			//当前机器人7个角度值填入这里	
			pub.JointVel[0]=0;		//第一轴
			pub.JointVel[1]=0;
			pub.JointVel[2]=0;
			pub.JointVel[3]=0;
			pub.JointVel[4]=0;
			pub.JointVel[5]=0+ omega;	
			pub.JointVel[6]=0 ;

            ///
            pub.Torque[0]=delta_angle;
		}

    
		pub2.publish(pub);
		rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
