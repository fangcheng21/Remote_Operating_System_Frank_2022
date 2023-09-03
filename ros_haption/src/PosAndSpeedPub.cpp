#include"ros/ros.h"
#include"ros_haption/HaptionData.h"


int main(int argc, char *argv[])
{
    ros::init(argc,argv,"PosAndSpeedPub");
    ros::NodeHandle nh;
    ros::Publisher pub2=nh.advertise<ros_haption::HaptionData>("HaptionPosAndSpeed",1000);

    ros_haption::HaptionData pub;
    for (int i = 0; i < 7; i++)
    {
           pub.Position[i]=0;
    }
    for (int i = 0; i < 6; i++)
    {
           pub.Speed[i]=0;
    }
    for (int i = 0; i < 6; i++)
    {
           pub.Force[i]=0;
    }
    for (int i = 0; i < 6; i++)
    {
           pub.ArticularPosition[i]=0;
    }
    for (int i = 0; i < 6; i++)
    {
           pub.ArticularSpeed[i]=0;
    }
    for (int i = 0; i < 3; i++)
    {
           pub.ButtonState[i]=0;
    }


	int num =0;
	double times = 0;
	//以1000频率发布角度信息
	ros::Rate rate(1000);

	while(num <= 10500)
	{

		num++;
              if(num>500)
              {
		    times = (num-500)* 0.001;
            constexpr double kRadius = 0.3;
            double angle = M_PI / 8 * (1 - std::cos(M_PI / 5.0 * times));
            double delta_x = kRadius * std::sin(angle);

            double Vx =  kRadius * std::cos(angle)*M_PI / 8*M_PI / 5.0*std::sin(M_PI / 5.0 * times);
            pub.Position[0] = delta_x;
	        pub.Speed[0]=Vx;		

              }


    
	    pub2.publish(pub);
	    rate.sleep();
	    ros::spinOnce();
	}

	return 0;
}
