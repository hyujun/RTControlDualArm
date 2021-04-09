
#ifndef __XENO__
#define __XENO__
#endif

#include "RTClient.h"

//Modify this number to indicate the actual number of motor on the network
#define ELMO_TOTAL 16
#define DUAL_ARM_DOF 16

hyuEcat::Master ecatmaster;
hyuEcat::EcatElmo ecat_elmo[ELMO_TOTAL];

// When all slaves or drives reach OP mode,
// system_ready becomes 1.
int system_ready = 0;
int break_flag = 0;
// Global time (beginning from zero)
double double_gt=0; //real global time
double double_dt=0;

// EtherCAT Data (Dual-Arm)
UINT16	        StatusWord[DUAL_ARM_DOF] = {0,};
INT32 	        ActualPos[DUAL_ARM_DOF] = {0,};
INT32 	        ActualVel[DUAL_ARM_DOF] = {0,};
INT16 	        ActualTor[DUAL_ARM_DOF] = {0,};
INT8	        ModeOfOperationDisplay[DUAL_ARM_DOF] = {0,};
std::string 	DeviceState[DUAL_ARM_DOF];
INT16 	        TargetTor[DUAL_ARM_DOF] = {0,};		//100.0 persentage
/****************************************************************************/
// Xenomai RT tasks
RT_TASK RTArm_task;
RT_TASK print_task;
RT_TASK tcpip_task;
RT_TASK event_task;

RT_QUEUE msg_tcpip;
RT_QUEUE msg_event;

void signal_handler(int signum);

// For RT thread management
unsigned long fault_count=0;
unsigned long ethercat_time=0;
unsigned long worst_time=0;

VectorXd ActualPos_Rad;
VectorXd ActualVel_Rad;
VectorXd TargetPos_Rad;
VectorXd TargetVel_Rad;
VectorXd TargetAcc_Rad;
VectorXd TargetToq;

VectorXd TargetPos_Task;
VectorXd TargetVel_Task;
VectorXd TargetAcc_Task;

static int hand_motion;
static int hand_state;

int isSlaveInit()
{
#if defined(_ECAT_ON_)
	int elmo_count = 0;
	int slave_count = 0;

	for(int i=0; i<ELMO_TOTAL; ++i)
	{
		if(ecat_elmo[i].initialized())
		{
			elmo_count++;
		}
	}

	for(int j=0; j<((int)ecatmaster.GetConnectedSlaves()-1); j++)
	{
		if(ecatmaster.GetSlaveState(j) == 0x08)
		{
			slave_count++;
		}
	}

	if((elmo_count == ELMO_TOTAL) && (slave_count == ((int)ecatmaster.GetConnectedSlaves()-1)))
		return 1;
	else
		return 0;
#endif
}


Vector3d ForwardPos[2];
Vector3d ForwardOri[2];
Vector3d ForwardAxis[2];
int NumChain;

// RTArm_task
void RTRArm_run( void *arg )
{
#if defined(_PLOT_ON_)
	int sampling_time 	= 20;	// Data is sampled every 10 cycles.
	int sampling_tick 	= sampling_time;

	void *msg;
	LOGGING_PACK logBuff;
	int len = sizeof(LOGGING_PACK);
#endif
	RTIME now, previous;
	RTIME start;
	RTIME p1 = 0;
	RTIME p3 = 0;

	short MaxTor = 1200;
	hand_motion = 0x00;

	uint16_t ControlMotion = SYSTEM_BEGIN;
	uint16_t JointState = SYSTEM_BEGIN;

    ActualPos_Rad.setZero(DUAL_ARM_DOF);
    ActualVel_Rad.setZero(DUAL_ARM_DOF);
    TargetPos_Rad.setZero(DUAL_ARM_DOF);
    TargetVel_Rad.setZero(DUAL_ARM_DOF);
    TargetAcc_Rad.setZero(DUAL_ARM_DOF);
    TargetToq.setZero(DUAL_ARM_DOF);

    TargetPos_Task.setZero(12);
    TargetVel_Task.setZero(12);
    TargetAcc_Task.setZero(12);
	VectorXd finPos = VectorXd::Zero(DUAL_ARM_DOF);

	std::shared_ptr<SerialManipulator> DualArm = std::make_shared<SerialManipulator>();
	std::unique_ptr<HYUControl::Controller> Control = std::make_unique<HYUControl::Controller>(DualArm);
    std::unique_ptr<HYUControl::Motion> motion = std::make_unique<HYUControl::Motion>(DualArm);

	DualArm->UpdateManipulatorParam();

	/* Arguments: &task (NULL=self),
	 *            start time,
	 *            period
	 */
	rt_task_set_periodic(nullptr, TM_NOW, cycle_ns);

	while (true)
	{
		rt_task_wait_period(nullptr); 	//wait for next cycle
        if(break_flag == 1)
            break;

		previous = rt_timer_read();
#if defined(_ECAT_ON_)
        ecatmaster.RxUpdate();
#endif
		for(int k=0; k < DUAL_ARM_DOF; k++)
		{
			DeviceState[k] = 			ecat_elmo[k].GetDevState();
			StatusWord[k] = 			ecat_elmo[k].status_word_;
			ModeOfOperationDisplay[k] = ecat_elmo[k].mode_of_operation_display_;
			ActualPos[k] =				ecat_elmo[k].position_;
			ActualVel[k] =				ecat_elmo[k].velocity_;
			ActualTor[k] =				ecat_elmo[k].torque_;
		}

        DualArm->ENCtoRAD(ActualPos, ActualPos_Rad);
        DualArm->VelocityConvert(ActualVel, ActualVel_Rad);

		if( system_ready )
		{
			DualArm->pKin->PrepareJacobian(ActualPos_Rad);
            DualArm->pDyn->PrepareDynamics(ActualPos_Rad, ActualVel_Rad);
			DualArm->pKin->GetForwardKinematics( ForwardPos, ForwardOri, NumChain );

			DualArm->StateMachine( ActualPos_Rad, ActualVel_Rad, finPos, JointState, ControlMotion );
			motion->JointMotion( TargetPos_Rad, TargetVel_Rad, TargetAcc_Rad,finPos,
                        ActualPos_Rad, ActualVel_Rad, double_gt, JointState, ControlMotion );

			//if(JointState == MOVE_CUSTOMIZE1)
            //    hand_motion == 0x11;
            //else if(JointState == MOVE_CUSTOMIZE9)
            //    hand_motion == 0x00;

			if( ControlMotion == MOVE_FRICTION )
            {
                Control->FrictionIdentification( ActualPos_Rad, ActualVel_Rad, TargetPos_Rad, TargetVel_Rad, TargetAcc_Rad, TargetToq, double_gt );
            }
			else if( ControlMotion == MOVE_CLIK_JOINT )
            {
                Control->CLIKTaskController( ActualPos_Rad, ActualVel_Rad, TargetPos_Task, TargetVel_Task, TargetToq, double_dt, 6 );
            }
			else
			{
				Control->InvDynController( ActualPos_Rad, ActualVel_Rad, TargetPos_Rad, TargetVel_Rad, TargetAcc_Rad, TargetToq, double_dt );
			}

			DualArm->TorqueConvert(TargetToq, TargetTor, MaxTor);

			//write the motor data
			for(int j=0; j < DUAL_ARM_DOF; ++j)
			{
				if(double_gt >= 1.0 && JointState != SYSTEM_BEGIN)
				{
					//ecat_elmo[j].writeTorque(TargetTor[j]);
				}
				else
				{
                    ecat_elmo[j].writeTorque(0);
				}
			}
		}
#if defined(_ECAT_ON_)
        ecatmaster.TxUpdate();
#endif
		// For EtherCAT performance statistics
		p1 = p3;
		p3 = rt_timer_read();
		now = rt_timer_read();

		if ( isSlaveInit() == 1 )
		{
            double_dt = (static_cast<double>(p3 - p1))*1e-3; 	// us
			double_gt = (static_cast<double>(p3 - start))*1e-9; // s
			ethercat_time = (long)(now - previous);

            system_ready=1;	//all drives have been done

            if ( worst_time<ethercat_time )
                worst_time=ethercat_time;

            if( ethercat_time >= cycle_ns )
            {
                fault_count++;
                worst_time=0;
            }
		}
		else
		{
			if(ecatmaster.GetConnectedSlaves() < ELMO_TOTAL)
			{
				//signal_handler(1);
			}

			system_ready = 0;
			double_gt = 0;
			worst_time = 0;
			ethercat_time = 0;
			start = rt_timer_read();
		}
	}
}

void print_run(void *arg)
{
	long stick=0;
	int count=0;

	rt_printf("\nPlease WAIT at least %i (s) until the system getting ready...\n", WAKEUP_TIME);
	
	/* Arguments: &task (NULL=self),
	 *            start time,
	 *            period (here: 100ms = 0.1s)
	 */
	RTIME PrintPeriod = 500e6;  //ms
	rt_task_set_periodic(nullptr, TM_NOW, PrintPeriod);
	
	while (true)
	{
		rt_task_wait_period(nullptr); //wait for next cycle
        if(break_flag==1)
            break;

		if ( system_ready )
		{
			rt_printf("Time=%0.2fs\n", double_gt);
			rt_printf("Calculation= %0.2fus, DesisredTask_dt=%0.2fus, WorstCalculation= %0.2fus, RealTimeFault=%d\n",
					static_cast<double>(ethercat_time)*1e-3, double_dt, static_cast<double>(worst_time)*1e-3, fault_count);

			for(int j=0; j<DUAL_ARM_DOF; ++j)
			{
				rt_printf("\t \nID: %d,", j+1);

#if defined(_DEBUG_)
				//rt_printf(" StatWord: 0x%04X, ",	StatusWord[j]);
				//rt_printf(" DeviceState: %d, ",	DeviceState[j]);
				rt_printf(" ModeOfOp: %d,",			ModeOfOperationDisplay[j]);
				//rt_printf("\n");
#endif
				rt_printf("\tActPos(Deg): %0.2lf,", 	ActualPos_Rad(j)*RADtoDEG);
				//rt_printf("\tTarPos(Deg): %0.2lf,",	TargetPos_Rad(j))*RADtoDEG);
				rt_printf("\tActPos(inc): %d,", 		ActualPos[j]);
				//rt_printf("\n");
				rt_printf("\tActVel(Deg/s): %0.1lf,", 	ActualVel_Rad(j)*RADtoDEG);
				//rt_printf("\tTarVel(Deg/s): %0.1lf,",	TargetVel_Rad(j)*RADtoDEG);
				//rt_printf("\tActVel(inc/s): %d,", 	ActualVel[j]);
				//rt_printf("\n");
				rt_printf("\tActTor(%): %d,",			ActualTor[j]);
				rt_printf("\tCtrlTor(Nm): %0.1lf", 	TargetToq(j));
				//rt_printf("\tTarTor(%): %d", 			TargetTor[j]);
				//rt_printf("\n");
			}

			rt_printf("\nForward Kinematics -->");
			for(int cNum = 0; cNum < NumChain; cNum++)
			{
				rt_printf("\n Num:%d: x:%0.3lf, y:%0.3lf, z:%0.3lf, u:%0.3lf, v:%0.3lf, w:%0.3lf",cNum, ForwardPos[cNum](0), ForwardPos[cNum](1), ForwardPos[cNum](2),
						ForwardOri[cNum](0)*RADtoDEG, ForwardOri[cNum](1)*RADtoDEG, ForwardOri[cNum](2)*RADtoDEG);
				//rt_printf("\n Manipulability: Task:%0.2lf, Orient:%0.2lf", TaskCondNumber[cNum], OrientCondNumber[cNum]);
			}
			rt_printf("\n\n");
		}
		else
		{
            if ( ++count >= roundl(static_cast<double>(NSEC_PER_SEC)/static_cast<double>(PrintPeriod)) )
            {
                ++stick;
                count=0;
            }

			if ( count==0 )
			{
				rt_printf("\nReady Time: %i sec", stick);
				rt_printf("\nMaster State: %s, AL state: 0x%02X, ConnectedSlaves : %d",
                          ecatmaster.GetEcatMasterLinkState().c_str(), ecatmaster.GetEcatMasterState(), ecatmaster.GetConnectedSlaves());
				for(int i=0; i<((int)ecatmaster.GetConnectedSlaves()-1); i++)
				{
					rt_printf("\nID: %d , SlaveState: 0x%02X, SlaveConnection: %s, SlaveNMT: %s ", i,
                              ecatmaster.GetSlaveState(i), ecatmaster.GetSlaveConnected(i).c_str(), ecatmaster.GetSlaveNMT(i).c_str());
					rt_printf(" SlaveStatus : %s,", DeviceState[i].c_str());
					rt_printf(" StatWord: 0x%04X, ", StatusWord[i]);

				}
				rt_printf("\n");
			}
		}
	}
}

void tcpip_run(void *arg)
{
	Poco::Net::SocketAddress server_addr(SERVER_PORT);
    Poco::Net::ServerSocket server_sock(server_addr);
    SocketHandler server(server_sock);

	rt_task_set_periodic(nullptr, TM_NOW, 10e6); //ms
	while(true)
	{
		rt_task_wait_period(nullptr);
		if(break_flag==1)
		    break;

        server.TXRXUpdate();
	}
}

int kbhit()
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if(ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }
    return 0;
}

void event_run(void *arg)
{
    int key_event=0;

    rt_task_set_periodic(nullptr, TM_NOW, 1000e3); //us
    while(true)
    {
        rt_task_wait_period(nullptr);
        if(break_flag)
            break;

        if(system_ready)
        {
            if(kbhit())
            {
                key_event = getchar();
                rt_printf("\nReceived Data %c\n", key_event);
                switch(key_event)
                {
                    case 't':
                    case 'k':
                        break;
                    default:
                        break;
                }
            }
        }
    }
}

/****************************************************************************/
void signal_handler(int signum)
{
	rt_printf("\nSignal Interrupt: %d", signum);

    rt_printf("\nConsolPrint RTTask Closing....");
    rt_task_delete(&print_task);
    rt_printf("\nConsolPrint RTTask Closing Success....");

#if defined(_TCPIP_ON_)
	rt_printf("\nTCPIP RTTask Closing....");
    rt_task_delete(&tcpip_task);
    rt_printf("\nTCPIP RTTask Closing Success....");
#endif

#if defined(_ECAT_ON_)
	rt_printf("\nEtherCAT RTTask Closing....");
	rt_task_delete(&RTArm_task);
	rt_printf("\nEtherCAT RTTask Closing Success....");
#endif

    rt_printf("\nEvent RTTask Closing....");
    rt_task_delete(&event_task);
    rt_printf("\nEvent RTTask Closing Success....");

    rt_printf("\n\n\t !!RT Arm Client System Stopped!! \n");
    break_flag=1;
}

/****************************************************************************/
int main(int argc, char **argv)
{
    // Perform auto-init of rt_print buffers if the task doesn't do so
    rt_print_auto_init(1);

    signal(SIGHUP, signal_handler);
	signal(SIGINT, signal_handler);
    signal(SIGQUIT, signal_handler);
    signal(SIGIOT, signal_handler);
    signal(SIGFPE, signal_handler);
    signal(SIGKILL, signal_handler);
    signal(SIGSEGV, signal_handler);
	signal(SIGTERM, signal_handler);

	/* Avoids memory swapping for this program */
	mlockall( MCL_CURRENT | MCL_FUTURE );

	// TO DO: Specify the cycle period (cycle_ns) here, or use default value
	//cycle_ns = 250e3; // nanosecond -> 4kHz
	//cycle_ns = 500e3; // nanosecond -> 2kHz
	cycle_ns = 1000e3; // nanosecond -> 1kHz
	//cycle_ns = 1250e3; // nanosecond -> 800Hz
	//cycle_ns = 2000ee; // nanosecond -> 500Hz

#if defined(_ECAT_ON_)
	for(int SlaveNum=0; SlaveNum < ELMO_TOTAL; SlaveNum++)
	{
		ecatmaster.addSlave(0, SlaveNum, &ecat_elmo[SlaveNum]);
	}
    ecatmaster.activateWithDC(0, cycle_ns);
#endif

	// RTArm_task: create and start
	rt_printf("\n-- Now running rt tasks ...\n");

    rt_task_create(&print_task, "Console_proc", 0, 70, 0);
    rt_task_start(&print_task, &print_run, nullptr);

#if defined(_ECAT_ON_)
	rt_task_create(&RTArm_task, "Control_proc", 1024*1024*5, 95, 0); // MUST SET at least 4MB stack-size (MAXIMUM Stack-size ; 8192 kbytes)
	rt_task_start(&RTArm_task, &RTRArm_run, nullptr);
#endif

#if defined(_TCPIP_ON_)
	rt_task_create(&tcpip_task, "TCPIP_proc", 0, 85, 0);
	rt_task_start(&tcpip_task, &tcpip_run, nullptr);
#endif

    rt_task_create(&event_task, "Event_proc", 0, 80, 0);
    rt_task_start(&event_task, &event_run, nullptr);

	// Must pause here
	pause();

    ecatmaster.deactivate();

    return 0;
}