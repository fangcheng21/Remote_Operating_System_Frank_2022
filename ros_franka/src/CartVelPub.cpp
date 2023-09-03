#include"ros/ros.h"
#include"ros_franka/FrankaData.h"
#include"ros_franka/FrankaCartVel.h"


int main(int argc, char *argv[])
{
    ros::init(argc,argv,"CartVelPub");
    ros::NodeHandle nh;
    ros::Publisher pub2=nh.advertise<ros_franka::FrankaCartVel>("CartVelPos",10);


    ros_franka::FrankaCartVel pub;
    for (int i = 0; i < 6; i++)
    {
           pub.EndVel[i]=0;
    }

    double time_max = 4.0;
    double v_max = 0.1;
    double angle = M_PI / 4.0;
	int num =0;
	double times = 0;
	ros::Rate rate(1000);

	while(num <= 8500)
	{
		num++;
		if(num>=500)
		{
			times = (num-500) * 0.001;

		    double cycle = std::floor(pow(-1.0, (times - std::fmod(times, time_max)) / time_max));
            double v = cycle * v_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * times));
            double v_x = std::cos(angle) * v;
            double v_z = -std::sin(angle) * v;

			//当前机器人7个角度值填入这里	
			pub.EndVel[0]= v_x;
            pub.EndVel[2]= v_z;
		}

    
		pub2.publish(pub);
		rate.sleep();
		ros::spinOnce();
	}

	return 0;
}