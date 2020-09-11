
#ifndef __XENO__
#define __XENO__
#endif

#include "RTClient.h"

//Modify this number to indicate the actual number of motor on the network
#if defined(_WITH_KIST_HAND_)

#define ELMO_TOTAL 			17
#define DUAL_ARM_DOF 		15
#define HAND_DOF			2

std::map<int, int> ElmoDualArmMap = {
		{0,0}, {1,1}, {2,2}, {3,3}, {4,4}, {5,5}, {6,6}, {7,7}, {8,10}, {9,11}, {10,12}, {11,13}, {12,14}, {13,15}, {14,16}
};

std::map<int, int> ElmoHandMap = {
		{0, 8}, {1, 9}
};

#else
#define ELMO_TOTAL 16
#define DUAL_ARM_DOF 16
#endif

hyuEcat::Master ecatmaster;
hyuEcat::EcatElmo ecat_elmo[ELMO_TOTAL];

struct LOGGING_PACK
{
	double 	Time;						/**< Global Time*/
	double 	ActualPos[DUAL_ARM_DOF]; 	/**< Actual Position in Radian*/
	double 	ActualVel[DUAL_ARM_DOF];	/**< Actual Velocity in Radian/second*/
	short  	ActualToq[DUAL_ARM_DOF];
	double 	DesiredPos[DUAL_ARM_DOF];
	double  DesiredVel[DUAL_ARM_DOF];
	short  	DesiredToq[DUAL_ARM_DOF];
};

// NRMKDataSocket for plotting axes data in Data Scope
EcatDataSocket datasocket;

// When all slaves or drives reach OP mode,
// system_ready becomes 1.
int system_ready = 0;

// Global time (beginning from zero)
double double_gt=0; //real global time
float float_dt=0;

// EtherCAT Data (Dual-Arm)
UINT16	StatusWord[DUAL_ARM_DOF] = {0,};
INT32 	ActualPos[DUAL_ARM_DOF] = {0,};
INT32 	ActualVel[DUAL_ARM_DOF] = {0,};
INT16 	ActualTor[DUAL_ARM_DOF] = {0,};
INT8	ModeOfOperationDisplay[DUAL_ARM_DOF] = {0,};
INT8	DeviceState[DUAL_ARM_DOF] = {0,};
INT16 	TargetTor[DUAL_ARM_DOF] = {0,};		//100.0 persentage

#if defined(_WITH_KIST_HAND_)
// EthercCAT Data (Hand)
UINT16	Hand_StatusWord[HAND_DOF] = {0,};
INT32 	Hand_ActualPos[HAND_DOF] = {0,};
INT32 	Hand_ActualVel[HAND_DOF] = {0,};
INT16 	Hand_ActualTor[HAND_DOF] = {0,};
INT8	Hand_ModeOfOperationDisplay[HAND_DOF] = {0,};
INT8	Hand_DeviceState[HAND_DOF] = {0,};

INT32 	Hand_TargetPos[HAND_DOF] = {0,};		//inc
INT32 	Hand_TargetVel[HAND_DOF] = {0,};		//inc/sec
INT16 	Hand_TargetTor[HAND_DOF] = {0,};		//100.0 persentage
INT8 	Hand_ModeOfOperation[HAND_DOF] = {0,};
UINT16	Hand_ControlWord[HAND_DOF] = {0,};

#endif
/****************************************************************************/
// Xenomai RT tasks
RT_TASK RTArm_task;
RT_TASK print_task;
RT_TASK plot_task;
RT_TASK tcpip_task;

RT_QUEUE msg_plot;
RT_QUEUE msg_tcpip;

void signal_handler(int signum);
int isSlaveInit(void);

// For RT thread management
unsigned long fault_count=0;
unsigned long ethercat_time=0;
unsigned long worst_time=0;

static double ActualPos_Rad[DUAL_ARM_DOF] = {0.0,};
static double ActualVel_Rad[DUAL_ARM_DOF] = {0.0,};
static double TargetPos_Rad[DUAL_ARM_DOF] = {0.0,};
static double TargetVel_Rad[DUAL_ARM_DOF] = {0.0,};
static double TargetAcc_Rad[DUAL_ARM_DOF] = {0.0,};
static double TargetToq[DUAL_ARM_DOF] = {0.0,};

#if defined(_WITH_KIST_HAND_)
static double Hand_ActualPos_Rad[HAND_DOF] = {0.0,};
static double Hand_ActualVel_Rad[HAND_DOF] = {0.0,};
#endif


static double best_manipulatorpower=0;

#if defined(_ECAT_ON_)
int isSlaveInit(void)
{
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
}
#endif

Vector3d ForwardPos[2];
Vector3d ForwardOri[2];
Vector3d ForwardAxis[2];
int NumChain;

// RTArm_task
void RTRArm_run(void *arg)
{
#if defined(_PLOT_ON_)
	int sampling_time 	= 20;	// Data is sampled every 10 cycles.
	int sampling_tick 	= sampling_time;

	void *msg;
	LOGGING_PACK logBuff;
	int len = sizeof(LOGGING_PACK);
#endif
	RTIME now, previous;
	RTIME p1 = 0;
	RTIME p3 = 0;

	int once_flag = 0;
	short MaxTor = 1200;

	int k=0;

	uint16_t ControlMotion = SYSTEM_BEGIN;
	uint16_t JointState = SYSTEM_BEGIN;

	VectorXd finPos(DUAL_ARM_DOF);
	finPos.setZero();

	SerialManipulator DualArm;
	HYUControl::Controller Control(&DualArm);
	HYUControl::Motion motion(&DualArm);

	DualArm.UpdateManipulatorParam();

	//HYUControl::KistHand kisthand;

	/* Arguments: &task (NULL=self),
	 *            start time,
	 *            period
	 */
	rt_task_set_periodic(NULL, TM_NOW, cycle_ns);

	while (1)
	{
		rt_task_wait_period(NULL); 	//wait for next cycle

		previous = rt_timer_read();

		ecatmaster.RxUpdate();

#if defined(_WITH_KIST_HAND_)
		//read the motor data
		for(k=0; k < DUAL_ARM_DOF; k++)
		{
			DeviceState[k] = 			ecat_elmo[ElmoDualArmMap[k]].Elmo_DeviceState();
			StatusWord[k] = 			ecat_elmo[ElmoDualArmMap[k]].status_word_;
			ModeOfOperationDisplay[k] = ecat_elmo[ElmoDualArmMap[k]].mode_of_operation_display_;
			ControlWord[k] = 			ecat_elmo[ElmoDualArmMap[k]].control_word_;
			ActualPos[k] = 				ecat_elmo[ElmoDualArmMap[k]].position_;
			ActualVel[k] = 				ecat_elmo[ElmoDualArmMap[k]].velocity_;
			ActualTor[k] = 				ecat_elmo[ElmoDualArmMap[k]].torque_;
		}

		for(k=0; k < HAND_DOF; k++)
		{
			Hand_DeviceState[k] = 				ecat_elmo[ElmoHandMap[k]].Elmo_DeviceState();
			Hand_StatusWord[k] = 				ecat_elmo[ElmoHandMap[k]].status_word_;
			Hand_ModeOfOperationDisplay[k] = 	ecat_elmo[ElmoHandMap[k]].mode_of_operation_display_;
			Hand_ControlWord[k] = 				ecat_elmo[ElmoHandMap[k]].control_word_;
			Hand_ActualPos[k] = 				ecat_elmo[ElmoHandMap[k]].position_;
			Hand_ActualVel[k] = 				ecat_elmo[ElmoHandMap[k]].velocity_;
			Hand_ActualTor[k] = 				ecat_elmo[ElmoHandMap[k]].torque_;
		}

#else
		for(k=0; k < DUAL_ARM_DOF; k++)
		{
			DeviceState[k] = 			ecat_elmo[k].Elmo_DeviceState();
			StatusWord[k] = 			ecat_elmo[k].status_word_;
			ModeOfOperationDisplay[k] = ecat_elmo[k].mode_of_operation_display_;
			ActualPos[k] = 				ecat_elmo[k].position_;
			ActualVel[k] = 				ecat_elmo[k].velocity_;
			ActualTor[k] = 				ecat_elmo[k].torque_;
		}
#endif

		if( system_ready )
		{
			if( once_flag == 0)
			{
				//DualArm.ENCtoRAD(ActualPos, TargetPos_Rad);
				once_flag = 1;
			}

#if defined(_WITH_KIST_HAND_)
			kisthand.HandEnctoRad(Hand_ActualPos, Hand_ActualPos_Rad);
			kisthand.HandVelocityConvert(Hand_ActualVel, Hand_ActualVel_Rad);

			if(double_gt >= 1)
				HandMotion=1;

			kisthand.HandControl(HandMotion, Hand_ActualPos_Rad, Hand_TargetVel, double_gt);
#endif

			DualArm.ENCtoRAD(ActualPos, ActualPos_Rad);
			DualArm.VelocityConvert(ActualVel, ActualVel_Rad);

			DualArm.pKin->PrepareJacobian(ActualPos_Rad);
			DualArm.pDyn->PrepareDynamics(ActualPos_Rad, ActualVel_Rad);

			//DualArm.pKin->GetManipulability( TaskCondNumber, OrientCondNumber );
			DualArm.pKin->GetForwardKinematics( ForwardPos, ForwardOri, NumChain );

			DualArm.StateMachine( ActualPos_Rad, ActualVel_Rad, finPos, JointState, ControlMotion );
			motion.JointMotion( TargetPos_Rad, TargetVel_Rad, TargetAcc_Rad, finPos, ActualPos_Rad, ActualVel_Rad, double_gt, JointState, ControlMotion );

			//if( ControlMotion == MOVE_FRICTION )
			//	Control.FrictionIdentification( ActualPos_Rad, ActualVel_Rad, TargetPos_Rad, TargetVel_Rad, TargetAcc_Rad, TargetToq, double_gt );
			//else if( ControlMotion == MOVE_CLIK_JOINT )
			//	Control.CLIKTaskController( ActualPos_Rad, ActualVel_Rad, TargetPos_Rad, TargetVel_Rad, TargetPos_Task, TargetVel_Task, nullMotion, TargetToq, float_dt );
			//else
			//{
				//Control.PDGravController(ActualPos_Rad, ActualVel_Rad, TargetPos_Rad, TargetVel_Rad, TargetToq );
				//Control.InvDynController( ActualPos_Rad, ActualVel_Rad, TargetPos_Rad, TargetVel_Rad, TargetAcc_Rad, TargetToq, float_dt );
			//}

			DualArm.TorqueConvert(TargetToq, TargetTor, MaxTor);

			//manipulatorpower = DualArm.PowerComsumption(ActualTor);

#if defined(_WITH_KIST_HAND_)
			//write the motor data
			for(OutputCommandCount=0; OutputCommandCount < DUAL_ARM_DOF; ++OutputCommandCount)
			{
				if(double_gt >= 1.0)
				{
					ecat_elmo[ElmoDualArmMap[OutputCommandCount]].writeTorque(TargetTor[OutputCommandCount]);
				}
				else
				{
					ecat_elmo[ElmoDualArmMap[OutputCommandCount]].writeTorque(0);
				}
			}

			for(OutputCommandCount=0; OutputCommandCount < HAND_DOF; ++OutputCommandCount)
			{
				if(double_gt >= 1.0)
				{
					ecat_elmo[ElmoHandMap[OutputCommandCount]].writeVelocity(Hand_TargetVel[OutputCommandCount]);
				}
				else
				{
					ecat_elmo[ElmoHandMap[OutputCommandCount]].writeVelocity(0);
				}
			}

#else
			//write the motor data
			for(int j=0; j < DUAL_ARM_DOF; ++j)
			{

				if(double_gt >= 1.0)
				{
					ecat_elmo[j].writeTorque(TargetTor[j]);
				}
				else
				{
					ecat_elmo[j].writeTorque(0);
				}
			}
#endif
		}

		ecatmaster.TxUpdate();

#if defined(_USE_DC_MODE_)
		ecatmaster.SyncEcatMaster(rt_timer_read());
#endif

		// For EtherCAT performance statistics
		p1 = p3;
		p3 = rt_timer_read();
		now = rt_timer_read();

		if ( isSlaveInit() == 1 )
		{
			float_dt = ((float)(long)(p3 - p1))*1e-3; 		// us
			double_gt += ((double)(long)(p3 - p1))*1e-9; 	// s
			ethercat_time = (long) now - previous;

			if( double_gt >= 0.5 )
			{
				system_ready=1;	//all drives have been done

				if ( worst_time<ethercat_time )
					worst_time=ethercat_time;

				//if(best_manipulatorpower < manipulatorpower)
				//	best_manipulatorpower = manipulatorpower;

				if( ethercat_time > (unsigned long)cycle_ns )
				{
					fault_count++;
					worst_time=0;
				}
			}

#if defined(_PLOT_ON_)
			if ( (system_ready==1) && datasocket.hasConnection() && (sampling_tick-- == 0) )
			{
				sampling_tick = sampling_time - 1; // 'minus one' is necessary for intended operation

				logBuff.Time = double_gt;
				for(int k=0; k<DUAL_ARM_DOF; k++)
				{
					logBuff.ActualPos[k] = ActualPos_Rad[k]*RADtoDEG;
					logBuff.ActualVel[k] = ActualVel_Rad[k]*RADtoDEG;
					logBuff.ActualToq[k] = ActualTor[k];

					logBuff.DesiredPos[k] = TargetPos_Rad[k]*RADtoDEG;
					logBuff.DesiredVel[k] = TargetVel_Rad[k]*RADtoDEG;
					logBuff.DesiredToq[k] = TargetTor[k];
				}

				msg = rt_queue_alloc(&msg_plot, len);
				if(msg == NULL)
					rt_printf("rt_queue_alloc Failed to allocate, NULL pointer received\n");

				memcpy(msg, &logBuff, len);
				rt_queue_send(&msg_plot, msg, len, Q_NORMAL);
			}
#endif
		}
		else
		{
			system_ready = 0;
			double_gt = 0;
			worst_time = 0;
			ethercat_time = 0;
			best_manipulatorpower=0;
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
	RTIME PrintPeriod = 5e8;
	rt_task_set_periodic(NULL, TM_NOW, PrintPeriod);
	
	while (1)
	{
		rt_task_wait_period(NULL); //wait for next cycle

		if ( ++count >= roundl(NSEC_PER_SEC/PrintPeriod) )
		{
			++stick;
			count=0;
		}



		if ( system_ready )
		{
			rt_printf("Time=%0.2fs\n", double_gt);
			rt_printf("actTask_dt= %lius, desTask_dt=%0.1fus, Worst_dt= %lius, Fault=%d, PowerComsumption=%0.2lfW\n",
					ethercat_time/1000, float_dt, worst_time/1000, fault_count, best_manipulatorpower);

			for(int j=0; j<DUAL_ARM_DOF; ++j)
			{
				rt_printf("\t \nID: %d,", j+1);

#if defined(_DEBUG_)
				//rt_printf(" StatWord: 0x%04X, ",	StatusWord[j]);
				//rt_printf(" DeviceState: %d, ",	DeviceState[j]);
				rt_printf(" ModeOfOp: %d,",			ModeOfOperationDisplay[j]);
				//rt_printf("\n");
#endif
				rt_printf("\tActPos(Deg): %0.2lf,", 	ActualPos_Rad[j]*RADtoDEG);
				//rt_printf("\tTarPos(Deg): %0.2lf,",	TargetPos_Rad[j]*RADtoDEG);
				rt_printf("\tActPos(inc): %d,", 		ActualPos[j]);
				//rt_printf("\n");
				rt_printf("\tActVel(Deg/s): %0.1lf,", 	ActualVel_Rad[j]*RADtoDEG);
				//rt_printf("\tTarVel(Deg/s): %0.1lf,",	TargetVel_Rad[j]*RADtoDEG);
				//rt_printf("\tActVel(inc/s): %d,", 	ActualVel[j]);
				//rt_printf("\n");
				rt_printf("\tActTor(%): %d,",			ActualTor[j]);
				rt_printf("\tCtrlTor(Nm): %0.1lf", 	TargetToq[j]);
				//rt_printf("\tTarTor(%): %d", 			TargetTor[j]);
				//rt_printf("\n");
			}

			rt_printf("\nForward Kinematics -->");
			for(int cNum = 0; cNum < NumChain; cNum++)
			{
				rt_printf("\n Num:%d: x:%0.3lf, y:%0.3lf, z:%0.3lf, u:%0.3lf, v:%0.3lf, w:%0.3lf ",cNum, ForwardPos[cNum](0), ForwardPos[cNum](1), ForwardPos[cNum](2),
						ForwardOri[cNum](0)*RADtoDEG, ForwardOri[cNum](1)*RADtoDEG, ForwardOri[cNum](2)*RADtoDEG);
				//rt_printf("\n Manipulability: Task:%0.2lf, Orient:%0.2lf", TaskCondNumber[cNum], OrientCondNumber[cNum]);
				rt_printf("\n");
			}

#if defined(_WITH_KIST_HAND_)
			for(int Hand=0; Hand < HAND_DOF; Hand++)
			{
				rt_printf("\nID: %d, ", Hand+1);
				if(Hand < 2)
				{
					rt_printf("Right Index & Middle: ");
					rt_printf("\tModeOfOp: %d,",			Hand_ModeOfOperationDisplay[Hand]);
					//rt_printf("\tActPos(inc): %d,", 		Hand_ActualPos[Hand]);
					rt_printf("\tActPos(Deg): %0.2lf,", 	Hand_ActualPos_Rad[Hand]*RADtoDEG);
					rt_printf("\tActVel(Deg/s): %0.2lf,", 	Hand_ActualVel_Rad[Hand]*RADtoDEG);
					rt_printf("\tActVel(inc/s): %d,", 		Hand_ActualVel[Hand]);
					rt_printf("\tTarVel(inc): %d,", 		Hand_TargetVel[Hand]);
					rt_printf("\tActTor(%): %d,",			Hand_ActualTor[Hand]);
				}
			}
#endif
			rt_printf("\n");
		}
		else
		{
			if ( count==0 )
			{
				rt_printf("\nReady Time: %i sec", stick);

				rt_printf("\nMaster State: %s, AL state: 0x%02X, ConnectedSlaves : %d",
						ecatmaster.GetEcatMasterLinkState().c_str(), ecatmaster.GetEcatMasterState(), ecatmaster.GetConnectedSlaves());
				for(int i=0; i<((int)ecatmaster.GetConnectedSlaves()-1); i++)
				{
					rt_printf("\nID: %d , SlaveState: 0x%02X, SlaveConnection: %s, SlaveNMT: %s ", i,
							ecatmaster.GetSlaveState(i), ecatmaster.GetSlaveConnected(i).c_str(), ecatmaster.GetSlaveNMT(i).c_str());

					rt_printf(" SlaveStatus : %s,", ecat_elmo[i].GetDevState().c_str());
					rt_printf(" StatWord: 0x%04X, ", ecat_elmo[i].status_word_);

				}
				rt_printf("\n");
			}
		}
	}
}


void plot_run(void *arg)
{
	ssize_t len;
	void *msg;
	LOGGING_PACK logBuff;
	memset(&logBuff, 0, sizeof(LOGGING_PACK));

	int err = rt_queue_bind(&msg_plot, "PLOT_QUEUE", TM_INFINITE);
	if(err)
		fprintf(stderr, "Failed to queue bind, code %d\n", err);

	rt_task_set_periodic(NULL, TM_NOW, 1e7);

	while (1)
	{
		rt_task_wait_period(NULL);

		if ( (len = rt_queue_receive(&msg_plot, &msg, TM_INFINITE)) > 0 )
		{
			memcpy(&logBuff, msg, sizeof(LOGGING_PACK));

			datasocket.updateControlData( logBuff.ActualPos, logBuff.DesiredPos, logBuff.ActualVel, logBuff.DesiredVel, logBuff.ActualToq, logBuff.DesiredToq );
			datasocket.update( logBuff.Time );

			rt_queue_free(&msg_plot, msg);
		}
	}
}


void tcpip_run(void *arg)
{
	Poco::Net::ServerSocket sock(SERVER_PORT);
	auto pParams = new Poco::Net::TCPServerParams;
	pParams->setMaxThreads(3);
	pParams->setMaxQueued(3);

	static Poco::Net::TCPServer server(new SessionFactory(), sock, pParams);


	std::cout << "\n-- DualArm TCP Server Application." << std::endl;
	//std::cout << "maxThreads: " << server.maxThreads() << std::endl;
	std::cout << "-- maxConcurrentConnections: " << server.maxConcurrentConnections() << std::endl;

	server.start();

	rt_task_set_periodic(NULL, TM_NOW, 1000e6);

	while(1)
	{
		rt_task_wait_period(NULL);
		PrintServerStatus(server);
	}
}

/****************************************************************************/
void signal_handler(int signum)
{
	rt_printf("\nSignal Interrupt: %d", signum);
#if defined(_PLOT_ON_)
	rt_queue_unbind(&msg_plot);
	rt_printf("\nPlotting RTTask Closing....");
	rt_task_delete(&plot_task);
#endif

	rt_printf("\nTCPIP RTTask Closing....");
	rt_task_delete(&tcpip_task);
	rt_printf("\nTCPIP RTTask Closing Success....");

	rt_printf("\nConsolPrint RTTask Closing....");
	rt_task_delete(&print_task);
	rt_printf("\nConsolPrint RTTask Closing Success....");

#if defined(_ECAT_ON_)
	rt_printf("\nEtherCAT RTTask Closing....");
	rt_task_delete(&RTArm_task);
	rt_printf("\nEtherCAT RTTask Closing Success....");
#endif

	ecatmaster.deactivate();

    rt_printf("\n\n\t !!RT Arm Client System Stopped!! \n");
    exit(signum);
}

/****************************************************************************/
int main(int argc, char **argv)
{
	// Perform auto-init of rt_print buffers if the task doesn't do so
    rt_print_auto_init(1);

    signal(SIGHUP, signal_handler);
	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);
	signal(SIGKILL, signal_handler);
	signal(SIGFPE, signal_handler);
	signal(SIGQUIT, signal_handler);

	/* Avoids memory swapping for this program */
	mlockall( MCL_CURRENT | MCL_FUTURE );

	// TO DO: Specify the cycle period (cycle_ns) here, or use default value
	//cycle_ns = 250000; // nanosecond -> 4kHz
	//cycle_ns = 500000; // nanosecond -> 2kHz
	//cycle_ns = 1000000; // nanosecond -> 1kHz
	//cycle_ns = 1250000; // nanosecond -> 800Hz
	cycle_ns = 2e6; // nanosecond -> 500Hz
	period = ((double) cycle_ns)/((double) NSEC_PER_SEC);	//period in second unit


#if defined(_ECAT_ON_)

#if defined(_WITH_KIST_HAND_)
	int CheckElmoHand=0;
	int SlaveNum;
	for(SlaveNum=0; SlaveNum < ELMO_TOTAL; SlaveNum++)
	{
		ecatmaster.addSlave(0, SlaveNum, &ecat_elmo[SlaveNum]);
		if(ElmoHandMap[CheckElmoHand] == SlaveNum)
		{
			ecat_elmo[SlaveNum].mode_of_operation_ = 9;
			//ecat_elmo[SlaveNum].writeTorque(-10);
			CheckElmoHand++;
		}
	}
#else
	int SlaveNum;
	for(SlaveNum=0; SlaveNum < ELMO_TOTAL; SlaveNum++)
	{
		ecatmaster.addSlave(0, SlaveNum, &ecat_elmo[SlaveNum]);
	}
#endif


#if defined(_USE_DC_MODE_)
	ecatmaster.activateWithDC(0, cycle_ns);  //is a first arg DC location of MotorDriver?
#else
	ecatmaster.activate();
#endif
#endif

#if defined(_PLOT_ON_)
	// TO DO: Create data socket server
	datasocket.setPeriod(period);

	if (datasocket.startServer(SOCK_TCP, NRMK_PORT_DATA))
		rt_printf("Data server started at IP of : %s on Port: %d\n", datasocket.getAddress(), NRMK_PORT_DATA);

	rt_printf("Waiting for Data Scope to connect...\n");
	datasocket.waitForConnection(0);
#endif

	// RTArm_task: create and start
	rt_printf("\n-- Now running rt task ...\n");


#if defined(_PLOT_ON_)
	rt_queue_create(&msg_plot, "PLOT_QUEUE", sizeof(LOGGING_PACK)*20, 20, Q_FIFO|Q_SHARED);

	rt_task_create(&plot_task, "PLOT_PROC_Task", 0, 80, T_FPU);
	rt_task_start(&plot_task, &plot_run, NULL);
#endif

#if defined(_ECAT_ON_)
	rt_task_create(&RTArm_task, "CONTROL_PROC_Task", 1024*1024*4, 99, T_FPU); // MUST SET at least 4MB stack-size (MAXIMUM Stack-size ; 8192 kbytes)
	rt_task_start(&RTArm_task, &RTRArm_run, NULL);
#endif

	rt_task_create(&print_task, "CONSOLE_PROC_Task", 0, 70, T_FPU);
	rt_task_start(&print_task, &print_run, NULL);

	rt_task_create(&tcpip_task, "TCPIP_PROC_Task", 0, 80, T_FPU);
	rt_task_start(&tcpip_task, &tcpip_run, NULL);


	// Must pause here
	pause();

	// Finalize
	signal_handler(SIGTERM);

    return 0;
}



