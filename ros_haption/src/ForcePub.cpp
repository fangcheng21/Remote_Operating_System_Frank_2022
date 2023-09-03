#include"ros/ros.h"
#include"ros_haption/HaptionData.h"


int main(int argc, char *argv[])
{
    ros::init(argc,argv,"ForcePub");
    ros::NodeHandle nh;
    ros::Publisher pub2=nh.advertise<ros_haption::HaptionData>("HaptionForcePos",1000);

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

	while(num <= 5500)
	{

		num++;
              if(num>=500)
              {
		       times = (num-500) * 0.001;
		       //插补函数，5次插补
        	       double delta_angle = M_PI / 4.0 * (1 - std::cos(M_PI / 2.5 * times));


	              pub.Force[0]=delta_angle;		
	              pub.Force[1]=0;
	              pub.Force[2]=0;
	              pub.Force[3]=0;
	              pub.Force[4]=0;
	              pub.Force[5]=0;	
              }


    
	pub2.publish(pub);
	rate.sleep();
	ros::spinOnce();
	}

	return 0;
}
