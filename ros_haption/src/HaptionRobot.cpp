#include "HaptionRobot.h"

VirHaptions::VirHaptions()
{
        STATE1 = false;
        EMPTYSTATE1 = true;
        startGetPos= 0;
                initial_Pos={0,0,0,0,0,0,0};
                goal_EndPos={0,0,0,0,0,0,0};

                initial_force={0,0,0,0,0,0};
                goal_force={0,0,0,0,0,0};     
                initial_endvel={0,0,0,0,0,0};
                goal_endvel={0,0,0,0,0,0};
                initial_jointPos={0,0,0,0,0,0};
                goal_jointPos={0,0,0,0,0,0};
                initial_jointforce={0,0,0,0,0,0};
                goal_jointforce={0,0,0,0,0,0};
                initial_jointspeed={0,0,0,0,0,0};
                goal_jointspeed={0,0,0,0,0,0};

};


int VirHaptions::HaptionInit()
{
        M_VC = virtOpen("127.0.0.1#53210");  // Open connection
        if (M_VC == NULL)
	{
		fprintf(stderr, "Erreur dans virtOpen: %s\n",virtGetErrorMessage(virtGetErrorCode(NULL)));
		return -1;
	}

	//常见设置
	//virtSetIndexingMode(VC, INDEXING_ALL);
		//设置比例因子
	virtSetForceFactor(M_VC, 1.0f);
	virtSetSpeedFactor(M_VC, 1.0f);
	//virtSetTimeStep(M_VC, 0.003f);

	//float identity[7] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,1.0f};
	//virtSetBaseFrame(M_VC, identity); //修改虚拟机构基架的位置
	//virtSetObservationFrame(M_VC, identity)； //移动观察框

        //按1khz 需要测试是否可以调整频率
        timestep=0.001f;                                    //设置循环频率
        
        //virtSetTimeStep(M_VC, timestep);                          // Specify cycle time
        //virtAttachVO(M_VC, mass, inertia);                    // Attach to process

        virtSetPowerOn(M_VC, 1);                                        // Enable force-feedback

        //数据初始化
        HaptionGetData();
        HaptioninitData();
}


	//COMMAND_TYPE_NONE 无法移动
	//COMMAND_TYPE_IMPEDANCE 力/位置控制
	//COMMAND_TYPE_VIRTMECH  虚拟机构的位置/力控制
	//COMMAND_TYPE_ARTICULAR 关节位置控制
	//COMMAND_TYPE_ARTICULAR_IMPEDANCE 关节力控制
	//virtSetCommandType(VC, COMMAND_TYPE_VIRTMECH); //设置所需的控制模式
void VirHaptions::setCommandtype(VirtCommandType type)
{
        virtSetCommandType(M_VC, type);  //Choose control mode
}

void VirHaptions::setcommandtype_articular()
{
        virtSetCommandType(M_VC, COMMAND_TYPE_ARTICULAR);  
}
void VirHaptions::setcommandtype_impedance()
{
        virtSetCommandType(M_VC, COMMAND_TYPE_IMPEDANCE); 
}
 void VirHaptions::setcommandtype_virtmech()
{
        virtSetCommandType(M_VC, COMMAND_TYPE_VIRTMECH);  
}
void VirHaptions::setcommandtype_articular_impedance()
{
        virtSetCommandType(M_VC, COMMAND_TYPE_ARTICULAR_IMPEDANCE);  
}


void VirHaptions::HaptionClose()
{

        //此处需考虑加入判断是否有循环
        virtStopLoop(M_VC); 					// Stop update

        virtSetPowerOn(M_VC, 0);                                    // Disable force-feedback
        virtClose(M_VC);                                                        // Close connection
}

////////////////////////////////////////////////////////////////

void VirHaptions::HaptionGetData()
{

		
	int state1, state2, state3;

        virtGetPosition(M_VC, M_Data.Position.data());

        virtGetSpeed(M_VC, M_Data.Speed.data());

        virtGetForce(M_VC, M_Data.Force.data());

	virtGetArticularPosition(M_VC, M_Data.Position1.data());

	virtGetArticularSpeed(M_VC,M_Data.Speed1.data());

	virtGetButton(M_VC, 1, &state1);		
		M_Data.Buttonstate[0]= state1;

        virtGetButton(M_VC, 2, &state2);		
		M_Data.Buttonstate[1]= state2;

        virtGetButton(M_VC, 3, &state3);		
		M_Data.Buttonstate[2]= state3;
}

void VirHaptions::HaptioninitData()
{
        virtGetPosition(M_VC, initial_Pos.data());
        virtGetArticularPosition(M_VC, initial_jointPos.data());
}

void VirHaptions::setfrequency(float num)
{
        FREQUENCY =num;
	CyclesNum = std::round(1000/ FREQUENCY );

	//预先压入插值间隔五倍的点
	for(int i=0;i<(5*CyclesNum);i++)
	{
                for(int j=0;j<7;j++)
                {
                        EndPos_1.push(0);
                }
                for(int j=0;j<6;j++)
                {
                        force_1.push(0);
                        EndVel_1.push(0);
                        Jointpos_1.push(0);
                        Jointforce_1.push(0);
                        Jointspeed_1.push(0);
                }
	}
}

/* ********数据获取******** */
void virtGetDataCallback(VirtContext VC,void *__this)
{
        VirHaptions* _this = (VirHaptions*) __this;
        _this->HaptionGetData();
};
void VirHaptions::start_data_pth()
{
        virtSetPeriodicFunction(M_VC,virtGetDataCallback , &timestep, (void*)this); // Setup callback
        virtStartLoop(M_VC);
};


/* *********末端位置******* */
void virtSetPositionCallback(VirtContext VC,void *__this)
{
        VirHaptions* _this = (VirHaptions*) __this;
        _this->HaptionGetData();
        _this->setposition();        
};

void VirHaptions::setposition()
{
        std::array<float,7>array_1{};
         //加入初始点设置
        if( startGetPos )
	{
		if((EndPos_1.size())>=7 )
		{
			for(int i=0;i<7;i++)
                        {
                                goal_EndPos[i]=EndPos_1.front();
                                EndPos_1.pop();
                        }
                        EMPTYSTATE1 = true;
		}
		else
		{
			//加入退出指令
                        STATE1 = true;
                        if(EMPTYSTATE1)
                        {
                                std::cout<<"The array is empty!"<<std::endl;
                                EMPTYSTATE1=false;
                        }

		}
	}
	for(int i=0;i<7;i++)
        {
                array_1[i] = initial_Pos[i]+goal_EndPos[i];
        }

        virtSetPosition(M_VC, array_1.data());    
};

void VirHaptions::start_Pos_pth()
{
        virtSetPeriodicFunction(M_VC,virtSetPositionCallback , &timestep, (void*)this); // Setup callback
        virtStartLoop(M_VC);
};

/* *******末端力矩********* */
    //此处需要测试 1.是否需要加入循环 2.是否加入时间片轮询减小运算量
void virtSetForceCallback(VirtContext VC,void *__this)
{
        VirHaptions* _this = (VirHaptions*) __this;
        _this->HaptionGetData();
        _this->setforce();        
};  

void VirHaptions::setforce()
{
        std::array<float,6>array_1{};//放到类中

        if( startGetPos )
	{
		if((force_1.size())>=6)
		{
			for(int i=0;i<6;i++)
                        {
                                goal_force[i] = force_1.front();
                                force_1.pop();
                        }
                        EMPTYSTATE1 = true;
		}
		else
		{
			//加入退出指令
                        STATE1 = true;
			if(EMPTYSTATE1)
                        {
                                std::cout<<"The array is empty!"<<std::endl;
                                EMPTYSTATE1=false;
                        }
		}
	}

        	for(int i=0;i<6;i++)
                {
                        array_1[i] = initial_force[i]+goal_force[i];
                }

            virtSetForce(M_VC, array_1.data());    
};

void VirHaptions::start_Force_pth()
{
        virtSetPeriodicFunction(M_VC,virtSetForceCallback , &timestep, (void*)this); // Setup callback
        virtStartLoop(M_VC);
}

/* *******末端速度********* */
    //此处需要测试 1.是否需要加入循环 2.是否加入时间片轮询减小运算量
void virtSetSpeedCallback(VirtContext VC,void *__this)
{
        VirHaptions* _this = (VirHaptions*) __this;
        _this->HaptionGetData();
        _this->setspeed();        
}; 

void VirHaptions::setspeed()
{
        std::array<float,6>array_1{};   //放到类中
            //加入初始点设置
        if( startGetPos )
	{
		if((EndVel_1.size())>=6)
		{
			for(int i=0;i<6;i++)
                        {
                                goal_endvel[i] = EndVel_1.front();
                                EndVel_1.pop();
                        }
                        EMPTYSTATE1 = true;
		}
		else
		{
			//加入退出指令
                        STATE1 = true;
			if(EMPTYSTATE1)
                        {
                                std::cout<<"The array is empty!"<<std::endl;
                                EMPTYSTATE1=false;
                        }
		}
	}
                for(int i=0;i<6;i++)
                {
                        array_1[i] = initial_endvel[i]+goal_endvel[i];
                }
             virtSetSpeed(M_VC, array_1.data());    
};

void VirHaptions::start_Speed_pth()
{
        virtSetPeriodicFunction(M_VC,virtSetSpeedCallback , &timestep, (void*)this); // Setup callback
        virtStartLoop(M_VC);
}

/* *******关节位置********* */
//此处需要测试 1.是否需要加入循环 2.是否加入时间片轮询减小运算量
void virtSetArticularPosCallback(VirtContext VC,void *__this)
{
        VirHaptions* _this = (VirHaptions*) __this;
        _this->HaptionGetData();
        _this->setArticularPos();        
}; 

void VirHaptions::setArticularPos()
{
        std::array<float,6>array_1{};   //放到类中
            //加入初始点设置
        if( startGetPos )
	{
		if((Jointpos_1.size())>=6)
		{

			for(int i=0;i<6;i++)
                        {
                                goal_jointPos[i] = Jointpos_1.front();
                                Jointpos_1.pop();
                        }
                        EMPTYSTATE1=true;
		}
		else
		{
			//加入退出指令
                        STATE1 = true;
			if(EMPTYSTATE1)
                        {
                                std::cout<<"The array is empty!"<<std::endl;
                                EMPTYSTATE1=false;
                        }
		}
        }
                for(int i=0;i<6;i++)
                {
                        array_1[i] = initial_jointPos[i]+goal_jointPos[i];
                }
                     virtSetArticularPosition(M_VC, array_1.data());    
};

        void VirHaptions::start_ArticularPos_pth()
        {
                virtSetPeriodicFunction(M_VC,virtSetArticularPosCallback , &timestep, (void*)this); // Setup callback
                virtStartLoop(M_VC);
        }

/* *******关节力矩********* */
    //此处需要测试 1.是否需要加入循环 2.是否加入时间片轮询减小运算量
        void virtSetArticularForceCallback(VirtContext VC,void *__this)
    {
            VirHaptions* _this = (VirHaptions*) __this;
             _this->HaptionGetData();
             _this->setArticularForce();        
    }; 

    void VirHaptions::setArticularForce()
        {
                std::array<float,6>array_1{};   //放到类中
            //加入初始点设置
        if( startGetPos )
	{
		if((Jointforce_1.size())>=6)
		{
			for(int i=0;i<6;i++)
                        {
                                goal_jointforce[i] = Jointforce_1.front();
                                Jointforce_1.pop();
                        }
                        EMPTYSTATE1 = true;
		}
		else
		{
			//加入退出指令
                        STATE1 = true;
			if(EMPTYSTATE1)
                        {
                                std::cout<<"The array is empty!"<<std::endl;
                                EMPTYSTATE1=false;
                        }
		}
        }
                for(int i=0;i<6;i++)
                {
                        array_1[i] = initial_jointforce[i]+goal_jointforce[i];
                }

                virtSetArticularForce(M_VC, array_1.data());    
        };

        void VirHaptions::start_ArticularForce_pth()
        {
                virtSetPeriodicFunction(M_VC,virtSetArticularForceCallback , &timestep, (void*)this); // Setup callback
                virtStartLoop(M_VC);
        }

/* *******关节速度********* */
    //此处需要测试 1.是否需要加入循环 2.是否加入时间片轮询减小运算量
void virtSetArticularSpeedCallback(VirtContext VC,void *__this)
    {
            VirHaptions* _this = (VirHaptions*) __this;
             _this->HaptionGetData();
             _this->setArticularSpeed();        
    }; 

void VirHaptions::setArticularSpeed()
{
        std::array<float,6>array_1{};   //放到类中
            //加入初始点设置
        if( startGetPos )
	{
		if((Jointspeed_1.size())>=6)
		{
			for(int i=0;i<6;i++)
                        {
                                array_1[i] = Jointspeed_1.front();
                                Jointspeed_1.pop();
                        }
                        EMPTYSTATE1 = true;
		}
		else
		{
			//加入退出指令
                        STATE1 = true;
			if(EMPTYSTATE1)
                        {
                                std::cout<<"The array is empty!"<<std::endl;
                                EMPTYSTATE1=false;
                        }
		}

        }
                for(int i=0;i<6;i++)
                {
                        array_1[i] = initial_jointspeed[i]+goal_jointspeed[i];
                }
             virtSetArticularSpeed(M_VC, array_1.data());    
};

        void VirHaptions::start_ArticularSpeed_pth()
        {
                virtSetPeriodicFunction(M_VC,virtSetArticularSpeedCallback , &timestep, (void*)this); // Setup callback
                virtStartLoop(M_VC);
        }


/* *******导纳控制********* */
    //此处需要测试 1.是否需要加入循环 2.是否加入时间片轮询减小运算量
void virtSetadmittanceCallback(VirtContext VC,void *__this)
    {
            VirHaptions* _this = (VirHaptions*) __this;
             _this->HaptionGetData();
                _this->setposition();  
                _this->setspeed();       
    }; 


        void VirHaptions::start_admittance_pth()
        {
                virtSetPeriodicFunction(M_VC,virtSetadmittanceCallback , &timestep, (void*)this); // Setup callback
                virtStartLoop(M_VC);
        }