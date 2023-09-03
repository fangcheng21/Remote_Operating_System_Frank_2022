#include "FrankaRobot.h"

FrankaRobot::FrankaRobot(const std::string& franka_address):m_robot(franka_address)
{

        STATE = false;
        counter = 0;
        times = 0;
	times_1 = 0;
	startGetPos = 0;
	motion_finished = false;




	franka::RobotState initial_state = m_robot.readOnce();

	getfrankadate(initial_state);

};


/* **************** */
//非实时性设置  力矩，阻抗， 刚度
void FrankaRobot::robot_JoinPos_init()
{
	/*
	**lower_torque_thresholds_acceleration
	**upper_torque_thresholds_acceleration
	**lower_torque_thresholds_nominal
	**upper_torque_thresholds_nominal
	**lower_force_thresholds_acceleration
	**upper_force_thresholds_acceleration
	**lower_force_thresholds_nominal
	**upper_force_thresholds_nominal
	*/

	m_robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
 
	//设置内部控制器中每个关节的阻抗
	m_robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}}); 

	//在内部 控制器中设置笛卡尔刚度/顺从性（x、y、z、滚动、俯仰、偏航）
	m_robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});     
}

void FrankaRobot::robot_force_init()
{

	m_robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
 

	//m_robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});

	//m_robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}}); 
}

void FrankaRobot::robot_CartPos_init()
{

	m_robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
 

	m_robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});

	m_robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}}); 
}

void FrankaRobot::robot_CartVel_init()
{

	m_robot.setCollisionBehavior(
        {{25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0}}, {{35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}},
        {{25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0}}, {{35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}},
        {{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}}, {{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}},
        {{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}}, {{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}});
 

	m_robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});

	//m_robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}}); 
}

void FrankaRobot::robot_JointVel_init()
{

	m_robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
 

	m_robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});

	m_robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}}); 
}



/* **************** */
//关节控制回调实现
franka::JointPositions FrankaRobot::JoinPos_control(const franka::RobotState& robot_state, franka::Duration period)
{
	times += period.toSec() ;                //时间周期，每次循环+0.001 s

      	if (times == 0.0) 
	{
	        initial_JointPos = robot_state.q_d;
      	}
	
	double dt = std::round(1000*times) - std::round(1000*times_1);
	
	times_1 = times;

	if( startGetPos )
	{
		if(!joint_1.empty())
		{
			for( int i =0; i < int(std::round(dt)); i++)
			{
				for(int j=0;j<7;j++)
				{
					goal_JointPos[j] = joint_1.front();
					joint_1.pop();
				};
			};
		}
		else
		{
			STATE = true;
			std::cout<<"The array is empty!"<<std::endl;
		}
		
		

	}


	
		
	
        getfrankadate(robot_state);

        franka::JointPositions output = {{initial_JointPos[0] + goal_JointPos[0] ,
                                                                                        initial_JointPos[1] + goal_JointPos[1] ,
                                                                                        initial_JointPos[2] + goal_JointPos[2] ,
                                                                                        initial_JointPos[3] + goal_JointPos[3] ,
                                                                                        initial_JointPos[4] + goal_JointPos[4] ,
                                                                                        initial_JointPos[5] + goal_JointPos[5] ,
                                                                                        initial_JointPos[6] + goal_JointPos[6] }};



        //判断结束指令
        if (STATE) {
        		motion_finished = true;
			std::cout<<"control finish"<<std::endl;
        };

        output.motion_finished = motion_finished;
        return output;
};

void* JoinPosControl_thread(void* __this)
{
	FrankaRobot* _this=(FrankaRobot*)__this;
	_this->m_robot.control(_this->JoinPos_control_callback);

}

void FrankaRobot::start_JointPos_pth(pthread_t* thread)
{
	pthread_t pthread;
	pthread_create(&pthread,NULL,JoinPosControl_thread,(void*)this);
	*thread=pthread;
};


/* **************** */
//力矩控制
 franka::Torques FrankaRobot::force_control(const franka::RobotState& robot_state, franka::Duration period)
{
         times += period.toSec();

          if (times == 0.0) {
          	return zero_torques;
          }

        double dt = std::round(1000*times) - std::round(1000*times_1);
	
	times_1 = times;


        if( startGetPos )
	{
		if(!force_1.empty())
		{
			for( int i =0; i < int(std::round(dt)); i++)
			{
				for(int j=0;j<7;j++)
				{
					goal_force[j] = force_1.front();
					force_1.pop();
				};
			};
		}
		else
		{
			STATE = true;
			std::cout<<"The array is empty!"<<std::endl;
		}
		

	}

	getfrankadate(robot_state);

        franka::Torques   output = {{goal_force[0] ,
                                                                                        goal_force[1]  ,
                                                                                        goal_force[2]  ,
                                                                                        goal_force[3]  ,
                                                                                        goal_force[4]  ,
                                                                                        goal_force[5]  ,
                                                                                        goal_force[6]  }};



        if (STATE) {
        		motion_finished = true;
				std::cout<<"control finish"<<std::endl;
        };
        output.motion_finished = motion_finished;
        return output;
};

void* ForceControl_thread(void* __this)
{
	FrankaRobot* _this=(FrankaRobot*)__this;
	_this->m_robot.control(_this->force_control_callback);

};
   
void FrankaRobot::start_Force_pth(pthread_t* thread)
{
	pthread_t pthread;
	pthread_create(&pthread,NULL,ForceControl_thread,(void*)this);
	*thread=pthread;
};


/* **************** */
//位置控制回调实现，
franka::CartesianPose FrankaRobot::CartPos_control(const franka::RobotState& robot_state, franka::Duration period)
{
	times += period.toSec();

	if (times == 0.0) {
		initial_Pos = robot_state.O_T_EE_c;
	}

        double dt = std::round(1000*times) - std::round(1000*times_1);
	
	times_1 = times;


        if( startGetPos )
	{
		if(!CartPos_1.empty())
		{
			for( int i =0; i < int(std::round(dt)); i++)
			{
				for(int j=0;j<16;j++)
				{
					goal_Pos[j] = CartPos_1.front();
					CartPos_1.pop();
				};
			};
		}
		else
		{
			STATE = true;
			std::cout<<"The array is empty!"<<std::endl;
		}
		

	}

	getfrankadate(robot_state);
        //改成16位变换矩阵
        franka::CartesianPose   output = {{initial_Pos[0] + goal_Pos[0], initial_Pos[1] + goal_Pos[1],    
						    initial_Pos[2] + goal_Pos[2],    initial_Pos[3] + goal_Pos[3],
			                            initial_Pos[4] + goal_Pos[4],    initial_Pos[5] + goal_Pos[5],
						    initial_Pos[6] + goal_Pos[6],    initial_Pos[7] + goal_Pos[7],    
			                            initial_Pos[8] + goal_Pos[8],    initial_Pos[9] + goal_Pos[9],
						    initial_Pos[10] + goal_Pos[10],   initial_Pos[11] + goal_Pos[11],
			                            initial_Pos[12] + goal_Pos[12],   initial_Pos[13] + goal_Pos[13],
						    initial_Pos[14] + goal_Pos[14],   initial_Pos[15] + goal_Pos[15]}};


        //判断结束指令
        if (STATE) {
        		motion_finished = true;
				std::cout<<"control finish"<<std::endl;
        };

        output.motion_finished = motion_finished;
        return output;
}

void* CartPosControl_thread(void* __this)
{
	FrankaRobot* _this=(FrankaRobot*)__this;
	_this->m_robot.control(_this->CartPos_control_callback);

}

void FrankaRobot::start_Pos_pth(pthread_t* thread)
{
	pthread_t pthread;
	pthread_create(&pthread,NULL,CartPosControl_thread,(void*)this);
	*thread=pthread;
};


/* **************** */
//末端速度
franka::CartesianVelocities FrankaRobot::CartVel_control(const franka::RobotState& robot_state, franka::Duration period)
{
	times += period.toSec();

	if (times == 0.0) {
		return zero_carvel;
	}

        double dt = std::round(1000*times) - std::round(1000*times_1);
	
	times_1 = times;


        if( startGetPos )
	{
		if(!CartVel_1.empty())
		{
			for( int i =0; i < int(std::round(dt)); i++)
			{
				for(int j=0;j<6;j++)
				{
					goal_CartVel[j] = CartVel_1.front();
					CartVel_1.pop();
				};
			};
		}
		else
		{
			STATE = true;
			std::cout<<"The array is empty!"<<std::endl;
		}
		

	}

	getfrankadate(robot_state);

        franka::CartesianVelocities   output = {{goal_CartVel[0],
                                                                                            goal_CartVel[1],
                                                                                            goal_CartVel[2],
                                                                                            goal_CartVel[3],
                                                                                            goal_CartVel[4],
                                                                                            goal_CartVel[5]}};


        if (STATE) {
        		motion_finished = true;
				std::cout<<"control finish"<<std::endl;
        };
        output.motion_finished = motion_finished;
        return output;
}

void* CartVelControl_thread(void* __this)
{
	FrankaRobot* _this=(FrankaRobot*)__this;
	_this->m_robot.control(_this->CartVel_control_callback);

}

void FrankaRobot::start_CarVel_pth(pthread_t* thread)
{
	pthread_t pthread;
	pthread_create(&pthread,NULL,CartVelControl_thread,(void*)this);
	*thread=pthread;
};


/* **************** */
//关节速度
franka::JointVelocities FrankaRobot::JointVel_control(const franka::RobotState& robot_state, franka::Duration period)
{
	times += period.toSec();

	if (times == 0.0) {
		return zero_jointvel;
	}

        double dt = std::round(1000*times) - std::round(1000*times_1);
	
	times_1 = times;


        if( startGetPos )
	{
		if(!JointVel_1.empty())
		{
			for( int i =0; i < int(std::round(dt)); i++)
			{
				for(int j=0;j<7;j++)
				{
					goal_jointVel[j] = JointVel_1.front();
					JointVel_1.pop();
				};
			};
		}
		else
		{
			STATE = true;
			std::cout<<"The array is empty!"<<std::endl;
		}
		

	}

	getfrankadate(robot_state);

        franka::JointVelocities   output = {{goal_jointVel[0],
                                                             goal_jointVel[1],
                                                             goal_jointVel[2],
                                                             goal_jointVel[3],
                                                             goal_jointVel[4],
                                                             goal_jointVel[5],
                                                             goal_jointVel[6]}};


        if (STATE) {
        		motion_finished = true;
				std::cout<<"control finish"<<std::endl;
        };
        output.motion_finished = motion_finished;
        return output;
}

void* JointVelControl_thread(void* __this)
{
	FrankaRobot* _this=(FrankaRobot*)__this;
	_this->m_robot.control(_this->JointVel_control_callback);
}

void FrankaRobot::start_JointVel_pth(pthread_t* thread)
{
	pthread_t pthread;
	pthread_create(&pthread,NULL,JointVelControl_thread,(void*)this);
	*thread=pthread;
};


///
void* ForceAndJointPos_Control_thread(void* __this)
{
	FrankaRobot* _this=(FrankaRobot*)__this;
	_this->m_robot.control(_this->force_control_callback,_this->JoinPos_control_callback);

}

void FrankaRobot::start_ForceAndJointPos_pth(pthread_t* thread)
{
	pthread_t pthread;
	pthread_create(&pthread,NULL,ForceAndJointPos_Control_thread,(void*)this);
	*thread=pthread;
};

///
void* ForceAndJointVel_Control_thread(void* __this)
{
	FrankaRobot* _this=(FrankaRobot*)__this;
	_this->m_robot.control(_this->force_control_callback,_this->JointVel_control_callback);
}

void FrankaRobot::start_ForceAndJointVel_pth(pthread_t* thread)
{
	pthread_t pthread;
	pthread_create(&pthread,NULL,ForceAndJointVel_Control_thread,(void*)this);
	*thread=pthread;
};

///
void* ForceAndPos_Control_thread(void* __this)
{
	FrankaRobot* _this=(FrankaRobot*)__this;
	_this->m_robot.control(_this->force_control_callback,_this->CartPos_control_callback);

}

void FrankaRobot::start_ForceAndPos_pth(pthread_t* thread)
{
	pthread_t pthread;
	pthread_create(&pthread,NULL,ForceAndPos_Control_thread,(void*)this);
	*thread=pthread;
};

///
void* ForceAndCarVel_Control_thread(void* __this)
{
	FrankaRobot* _this=(FrankaRobot*)__this;
	_this->m_robot.control(_this->force_control_callback,_this->CartVel_control_callback);

}

void FrankaRobot::start_ForceAndCarVel_pth(pthread_t* thread)
{
	pthread_t pthread;
	pthread_create(&pthread,NULL,ForceAndCarVel_Control_thread,(void*)this);
	*thread=pthread;
};



////////////////////////////////////////////////////////////////////////
void FrankaRobot::getfrankadate(franka::RobotState state)
{

    this->m_robotdata.EndPos=state.O_T_EE;
    this->m_robotdata.Torque=state.tau_J;
    this->m_robotdata.JointPos=state.q;
    this->m_robotdata.JointVel=state.dq;
    this->m_robotdata.JointAcc=state.ddq_d;
    this->m_robotdata.EndAcc=state.O_ddP_EE_c;
    this->m_robotdata.MotorPos=state.theta;

}


void FrankaRobot::GetRead_pth()
{
	franka::RobotState initial_state = m_robot.readOnce();
	getfrankadate(initial_state);
};

void FrankaRobot::robot_close()
{
	STATE =true;
};

void FrankaRobot::setfrequency(float num)
{

	FREQUENCY =num;

	CyclesNum = std::round(1000/ FREQUENCY );

	//预先压入插值间隔五倍的点
	for(int i=0;i<(5*CyclesNum);i++)
	{
		for(int j=0; j<7; j++)
		{
			joint_1.push(0);
			force_1.push(0);
			JointVel_1.push(0);
		};
		for(int j=0; j<16; j++)
		{
			CartPos_1.push(0);
		}
		for(int j=0; j<6; j++)
		{
			CartVel_1.push(0);
		}
	}

};

void FrankaRobot::RotMatrixToEuler(std::array<double,16> &Rot ,std::array<double,6> &Euler)
{
	Euler[0] = atan2(Rot[6],Rot[10]);   //Rx
	Euler[1] = atan2(-Rot[2],std::sqrt(Rot[6]*Rot[6]+Rot[10]*Rot[10]));     //Ry
	Euler[2] = atan2(Rot[1],Rot[0]);        //Rz
	Euler[3] = Rot[12];         //x
	Euler[4] = Rot[13];         //y
	Euler[5] = Rot[14];         //z
}

void FrankaRobot::EulerToRotMatrix(std::array<double,16> &Rot ,std::array<double,6> &Euler)
{
	Rot[0] =std::cos(Euler[2])*std::cos(Euler[1]);
	Rot[1] =std::sin(Euler[2])*std::cos(Euler[1]);
	Rot[2] =- std::sin(Euler[1]);
	Rot[3] =0;
	Rot[4] =-std::sin(Euler[2])*std::cos(Euler[0])+std::cos(Euler[2])*std::sin(Euler[1])*std::sin(Euler[0]);
	Rot[5] =std::cos(Euler[2])*std::cos(Euler[0])+std::sin(Euler[2])*std::sin(Euler[1])*std::sin(Euler[0]);
	Rot[6] =std::cos(Euler[1])*std::sin(Euler[0]);
	Rot[7] =0;
	Rot[8] =std::sin(Euler[2])*std::sin(Euler[0])+std::cos(Euler[2])*std::sin(Euler[1])*std::cos(Euler[0]);
	Rot[9] =-std::cos(Euler[2])*std::sin(Euler[0])+std::sin(Euler[2])*std::sin(Euler[1])*std::cos(Euler[0]);
	Rot[10] =std::cos(Euler[1])*std::cos(Euler[0]);
	Rot[11] =0;
	Rot[12] =Euler[3];
	Rot[13] =Euler[4];
	Rot[14] =Euler[5];
	Rot[15] =1;
}
