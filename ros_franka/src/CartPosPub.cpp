#include"ros/ros.h"
#include"ros_franka/FrankaData.h"
#include"ros_franka/FrankaCartPos.h"


int main(int argc, char *argv[])
{
    ros::init(argc,argv,"CartPosPub");
    ros::NodeHandle nh;
    ros::Publisher pub2=nh.advertise<ros_franka::FrankaCartPos>("CartPos",10);


    ros_franka::FrankaCartPos pub;
    for (int i = 0; i < 16; i++)
    {
           pub.EndPos[i]=0;
    }


	int num =0;
	double times = 0;
	ros::Rate rate(1000);

	while(num <= 10500)
	{
		num++;
		if(num>=500)
		{
			times = (num-500) * 0.001;

		    constexpr double kRadius = 0.3;
            double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * times));
            double delta_x = kRadius * std::sin(angle);
            double delta_z = kRadius * (std::cos(angle) - 1);

			//当前机器人7个角度值填入这里	
			pub.EndPos[12]= delta_x;
            pub.EndPos[14]= delta_z;
		}

    
		pub2.publish(pub);
		rate.sleep();
		ros::spinOnce();
	}

	return 0;
}