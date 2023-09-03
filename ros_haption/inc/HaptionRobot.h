#include <iostream>
#include <cmath>
#include <unistd.h>
#include <pthread.h>
#include <array>
#include <queue>

#include "VirtuoseAPI.h"


struct HaptionData
{
    std::array<float, 7>  Position{};  //virtGetPosition – Read the current position of the VIRTUOSE, or of the object attached to the VIRTUOSE.

    std::array<float,6>  Speed{};  //virtGetSpeed(VirtContext VC, float *speed_p_env_r_ps); 

    std::array<float,6>  Force{}; //virtGetForce

    std::array<float, 6>  Position1{};  //virtGetArticularPosition  //关节位置

    std::array<float, 6>  Speed1{};	//virtGetArticularSpeed   //关节速度

    std::array<int,3> Buttonstate{};//virtGetButton(VirtContext VC, int button_number, int *state);//仅支持三个按钮，它们的相应的指标取决于产品

    // Position: meters (m)
    // Angles: radians (rad)
    // Speed: meters per second (m/s)
    // Force: Newtons (N)
    // Torque: Newton-meters (Nm)
    // Mass: kilograms (kg)
    // Inertia: kilograms per square meters (kg/m2)
    // Translation stiffness: Newtons per meter (N/m)
    // Rotation stiffness: Newton-meters per radian (Nm/rad)
    // Translation damping: Newtons per meter per second (N/(m/s))
    // Rotation damping: Newton-meters per radian per second (Nm/(rad/s))

    // All integers of standard "int" type
    // All floating-point of standard "float" type
    // All vectors and matrices of standard "float []" type
    // A position is a float[7]
    // A speed is a float[6]
    // 3 values for the position (X, Y, Z)
    // 4 values for the orientation as a quaternion (Qx, Qy, Qz, Qw)
    // 3 values for the velocity (Vx, Vy, Vy)
    // 3 values for the rotation speed (Rx, Ry, Rz)
    // A force is a float[6]
    // 3 values for the force itself (Fx, Fy, Fz)
    // 3 values for the torques (Tx, Ty, Tz)
};

class VirHaptions 
{
    public:
        VirHaptions();

    public:

    //VirtContext virtOpen(const char *nom);
    ////////////////////////////virtSetArticularForce(VirtContext VC, float *force);
    //virtSetForce(VirtContext VC, float *force);
    ///////////////////////////virtSetArticularPosition(VirtContext VC, float *pos);
    //virtSetPosition(VirtContext VC, float *pos);
    ///////////////////////////virtSetArticularSpeed(VirtContext VC, float *speed);
    //virtSetSpeed(VC, speed); // Write device speed

    int HaptionInit();

    void HaptionClose();

    void HaptionGetData();

    void HaptioninitData();

    void setfrequency(float num);

    void setCommandtype(VirtCommandType type);

    void setcommandtype_articular();

    void setcommandtype_impedance();

    void setcommandtype_virtmech();

    void setcommandtype_articular_impedance();


    /* ********数据获取******** */
    void start_data_pth();

    /* *********末端位置******* */
    void setposition();

    void start_Pos_pth();

    /* *******末端力矩********* */
    void setspeed();

        void start_Speed_pth();

    /* *******末端速度********* */
    void setforce();

    void start_Force_pth();

    /* *******关节位置********* */
    void setArticularPos();

    void start_ArticularPos_pth();

    /* *******关节力矩********* */
    void setArticularForce();

    void start_ArticularForce_pth();

    /* *******关节速度********* */
    void setArticularSpeed();

    void start_ArticularSpeed_pth();

    /* *******导纳控制********* */
    void setadmittance();

    void start_admittance_pth();

    public:

    //需要初始化若干点
        std::queue<float>force_1;
        std::queue<float>EndPos_1;
        std::queue<float>EndVel_1;
        std::queue<float>Jointpos_1;
        std::queue<float>Jointforce_1;
        std::queue<float>Jointspeed_1;

        std::array<float,7>initial_Pos{};
        std::array<float,7>goal_EndPos{};

        std::array<float,6>initial_force{};
        std::array<float,6>goal_force{};     

        std::array<float,6>initial_endvel{};
        std::array<float,6>goal_endvel{};

        std::array<float,6>initial_jointPos{};
        std::array<float,6>goal_jointPos{};

        std::array<float,6>initial_jointforce{};
        std::array<float,6>goal_jointforce{};

        std::array<float,6>initial_jointspeed{};
        std::array<float,6>goal_jointspeed{};

    public:

        bool STATE1;    //退出信号标志位
        bool EMPTYSTATE1;
        int startGetPos;
        float timestep;

        float FREQUENCY;	//自定义频率
	    float CyclesNum;		//计数周期   1000/频率取整数

        VirtContext M_VC;

        HaptionData M_Data;

        HaptionData SendData;


};
