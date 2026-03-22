

#ifndef __XENO__
#define __XENO__
#endif

#include "RTClient.h"
#include <array>
#include <thread>
#include <rtdm/ipc.h>
#include <sys/socket.h>

//Modify this number to indicate the actual number of motor on the network
inline constexpr int ELMO_TOTAL = 16;
inline constexpr int DUAL_ARM_DOF = 16;

hyuEcat::Master ecatmaster;
std::array<hyuEcat::EcatElmo, ELMO_TOTAL> ecat_elmo{};

// When all slaves or drives reach OP mode,
// system_ready becomes 1.
int system_ready = 0;
bool break_flag = false;
// Global time (beginning from zero)
double double_gt=0.0; //real global time
double double_dt=0.0;

// For RT thread management
unsigned long fault_count=0;
unsigned long down_count=0;
unsigned long calculation_time=0;
unsigned long worst_time=0;

double double_dt_tcp=0.0;
unsigned long fault_count_tcp=0;
unsigned long calculation_time_tcp=0;
unsigned long worst_time_tcp=0;

// EtherCAT Data (Dual-Arm)
std::array<UINT16, DUAL_ARM_DOF> StatusWord{};
std::array<INT32,  DUAL_ARM_DOF> ActualPos{};
std::array<INT32,  DUAL_ARM_DOF> ActualVel{};
std::array<INT16,  DUAL_ARM_DOF> ActualTor{};
std::array<INT8,   DUAL_ARM_DOF> ModeOfOperationDisplay{};
std::array<const char*, DUAL_ARM_DOF> DeviceState{};
std::array<INT16,  DUAL_ARM_DOF> TargetTor{};		//100.0 persentage
/****************************************************************************/
// Xenomai RT tasks
RT_TASK RTArm_task;
RT_TASK print_task;
std::thread tcpip_thread;
RT_TASK event_task;


RT_QUEUE msg_event;

void signal_handler(int signum);

// Fixed-size vectors: avoid dynamic allocation in RT loop (C++17 inline)
inline Eigen::Matrix<double, 16, 1> ActualPos_Rad  = Eigen::Matrix<double, 16, 1>::Zero();
inline Eigen::Matrix<double, 16, 1> ActualVel_Rad  = Eigen::Matrix<double, 16, 1>::Zero();
inline Eigen::Matrix<double, 16, 1> TargetPos_Rad  = Eigen::Matrix<double, 16, 1>::Zero();
inline Eigen::Matrix<double, 16, 1> TargetVel_Rad  = Eigen::Matrix<double, 16, 1>::Zero();
inline Eigen::Matrix<double, 16, 1> TargetAcc_Rad  = Eigen::Matrix<double, 16, 1>::Zero();
inline Eigen::Matrix<double, 16, 1> TargetToq      = Eigen::Matrix<double, 16, 1>::Zero();

inline Eigen::Matrix<double, 12, 1> TargetPos_Task = Eigen::Matrix<double, 12, 1>::Zero();
inline Eigen::Matrix<double, 12, 1> TargetVel_Task = Eigen::Matrix<double, 12, 1>::Zero();
inline Eigen::Matrix<double, 12, 1> TargetAcc_Task = Eigen::Matrix<double, 12, 1>::Zero();
inline Eigen::Matrix<double, 12, 1> ActualPos_Task = Eigen::Matrix<double, 12, 1>::Zero();
inline Eigen::Matrix<double, 12, 1> ExternalForce  = Eigen::Matrix<double, 12, 1>::Zero();

inline Eigen::Matrix<double, 12, 1> ErrorPos_Task  = Eigen::Matrix<double, 12, 1>::Zero();
inline Eigen::Matrix<double, 16, 1> finPos         = Eigen::Matrix<double, 16, 1>::Zero();
inline Eigen::Matrix<double, 12, 1> findPos_Task   = Eigen::Matrix<double, 12, 1>::Zero();

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
#else
	return 1;
#endif
}


std::array<Vector3d, 2> ForwardPos;
std::array<Vector3d, 2> ForwardOri;
std::array<Vector3d, 2> ForwardAxis;
int NumChain;

static unsigned char ControlIndex1 = CTRLMODE_IDY_JOINT;
static unsigned char ControlIndex2 = 3;
static unsigned char ControlSubIndex = 1;

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
	RTIME start = rt_timer_read();
	RTIME p1 = 0;
	RTIME p3 = 0;

	short MaxTor = 1200;


    unsigned char JointState = ControlSubIndex;

    // Fixed-size vectors are already zero-initialized at definition; just reset
    ActualPos_Rad.setZero();
    ActualVel_Rad.setZero();
    TargetPos_Rad.setZero();
    TargetVel_Rad.setZero();
    TargetAcc_Rad.setZero();
    finPos.setZero();
    TargetToq.setZero();

    TargetPos_Task.setZero();
    TargetVel_Task.setZero();
    TargetAcc_Task.setZero();
    ActualPos_Task.setZero();
    ExternalForce.setZero();
    ErrorPos_Task.setZero();
    findPos_Task.setZero();

	std::shared_ptr<SerialManipulator> DualArm = std::make_shared<SerialManipulator>();
	std::unique_ptr<HYUControl::Controller> Control = std::make_unique<HYUControl::Controller>(DualArm);
    std::unique_ptr<HYUControl::Motion> motion = std::make_unique<HYUControl::Motion>(DualArm);

    Eigen::Matrix<double, 2, 1> des_mass = Eigen::Matrix<double, 2, 1>::Constant(5.0);
    Eigen::Matrix<double, 12, 1> KpTask = Eigen::Matrix<double, 12, 1>::Zero();
    Eigen::Matrix<double, 12, 1> KdTask = Eigen::Matrix<double, 12, 1>::Zero();
    Eigen::Matrix<double, 16, 1> KpNull = Eigen::Matrix<double, 16, 1>::Constant(0.001);
    Eigen::Matrix<double, 16, 1> KdNull = Eigen::Matrix<double, 16, 1>::Constant(3.0);

    KpTask.segment(0,3).setConstant(100.0);
    KpTask.segment(3,3).setConstant(1300.0);
    KpTask.segment(6,3).setConstant(100.0);
    KpTask.segment(9,3).setConstant(1300.0);

    KdTask.segment(0,3).setConstant(5.0);
    KdTask.segment(3,3).setConstant(55.0);
    KdTask.segment(6,3).setConstant(5.0);
    KdTask.segment(9,3).setConstant(55.0);

    Control->SetImpedanceGain(KpTask, KdTask, KpNull, KdNull, des_mass);

	DualArm->UpdateManipulatorParam();
    void *msg;

    TCP_Packet_Task packet_task;
    rt_task_set_mode(0, T_WARNSW, nullptr);

    int xddp_s = socket(AF_RTIPC, SOCK_DGRAM, IPCPROTO_XDDP);
    if (xddp_s < 0) {
        rt_printf("Failed to create XDDP socket\n");
    } else {
        size_t poolsz = 16384;
        setsockopt(xddp_s, SOL_RTIPC, XDDP_POOLSZ, &poolsz, sizeof(poolsz));
        struct timeval tv = {0, 0};
        setsockopt(xddp_s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        struct sockaddr_ipc saddr;
        memset(&saddr, 0, sizeof(saddr));
        saddr.sipc_family = AF_RTIPC;
        saddr.sipc_port = 0; // /dev/rtp0
        if (bind(xddp_s, (struct sockaddr *)&saddr, sizeof(saddr)) < 0) {
            rt_printf("Failed to bind XDDP\n");
        }
    }

	/* Arguments: &task (NULL=self),
	 *            start time,
	 *            period
	 */
	rt_task_set_periodic(nullptr, TM_NOW, cycle_ns);

	while (true)
	{
		rt_task_wait_period(nullptr); 	//wait for next cycle
        if(break_flag)
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

        DualArm->ENCtoRAD(ActualPos.data(), ActualPos_Rad);
        DualArm->VelocityConvert(ActualVel.data(), ActualVel_Rad);

		if( system_ready )
		{
			DualArm->pKin->PrepareJacobian( ActualPos_Rad );
            DualArm->pDyn->PrepareDynamics( ActualPos_Rad, ActualVel_Rad );
			DualArm->pKin->GetForwardKinematics( ForwardPos.data(), ForwardOri.data(), NumChain );


            if(xddp_s >= 0) {
                ssize_t len = recvfrom(xddp_s, &packet_task.data, sizeof(TCP_Packet_Task), MSG_DONTWAIT, NULL, 0);
                if(len > 0) {
                    rt_printf("received message> len=%d bytes, index1=0x%02X, index2=0x%02X, subindex=0x%02X\n",
                           (int)len, packet_task.info.index1, packet_task.info.index2, packet_task.info.subindex);
                    ControlIndex1 = packet_task.info.index1;
                    ControlIndex2 = packet_task.info.index2;
                    ControlSubIndex = packet_task.info.subindex;
                    JointState = ControlSubIndex;
                }
            }

			if( ControlIndex1 == CTRLMODE_FRICTIONID )
            {
                Control->FrictionIdentification( ActualPos_Rad, ActualVel_Rad, TargetPos_Rad, TargetVel_Rad, TargetAcc_Rad, TargetToq, double_gt );
            }
			else if( ControlIndex1 == CTRLMODE_CLIK )
            {
                if( ControlIndex2 == 7 )
                {
                    DualArm->pKin->GetForwardKinematicsWithRelative(ActualPos_Task);
                }
                else
                {
                    DualArm->pKin->GetForwardKinematics(ActualPos_Task);
                }
                motion->TaskMotion( TargetPos_Task, TargetVel_Task, TargetAcc_Task, findPos_Task, ActualPos_Task, ActualVel_Rad, double_gt, JointState, ControlSubIndex );
                Control->CLIKTaskController( ActualPos_Rad, ActualVel_Rad, TargetPos_Task, TargetVel_Task,TargetToq, double_dt, ControlIndex2 );
            }
			else if( ControlIndex1 == CTRLMODE_TASK )
            {
                DualArm->pKin->GetForwardKinematics(ActualPos_Task);
                motion->TaskMotion( TargetPos_Task, TargetVel_Task, TargetAcc_Task, findPos_Task, ActualPos_Task, ActualVel_Rad, double_gt, JointState, ControlSubIndex );
			    Control->TaskInvDynController(TargetPos_Task, TargetVel_Task, TargetAcc_Task, ActualPos_Rad, ActualVel_Rad, TargetToq, double_dt, ControlIndex2 );
                Control->GetControllerStates(TargetPos_Rad, TargetVel_Rad, ErrorPos_Task );

            }
			else if( ControlIndex1 == CTRLMODE_IMPEDANCE_TASK )
            {
			    if( ControlIndex2 == 3 )
                {
                    DualArm->pKin->GetForwardKinematicsWithRelative(ActualPos_Task);
                }
			    else
                {
                    DualArm->pKin->GetForwardKinematics(ActualPos_Task);
                }
			    motion->TaskMotion(TargetPos_Task, TargetVel_Task, TargetAcc_Task, findPos_Task, ActualPos_Task, ActualVel_Rad, double_gt, JointState, ControlSubIndex );
			    Control->TaskImpedanceController(ActualPos_Rad, ActualVel_Rad, TargetPos_Task, TargetVel_Task, TargetAcc_Task, ExternalForce, TargetToq, ControlIndex2 );
			    Control->GetControllerStates(TargetPos_Rad, TargetVel_Rad, ErrorPos_Task );
            }
			else
			{
                motion->JointMotion( TargetPos_Rad, TargetVel_Rad, TargetAcc_Rad, finPos, ActualPos_Rad, ActualVel_Rad, double_gt, JointState, ControlSubIndex );
				Control->InvDynController( ActualPos_Rad, ActualVel_Rad, TargetPos_Rad, TargetVel_Rad, TargetAcc_Rad, TargetToq, double_dt );
			}

			DualArm->TorqueConvert(TargetToq, TargetTor.data(), MaxTor);

			//write the motor data
			for(int j=0; j < DUAL_ARM_DOF; ++j)
			{
				if( double_gt >= 0.1 )
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
        ecatmaster.TxUpdate(0, rt_timer_read());
#endif
		// For EtherCAT performance statistics
		p1 = p3;
		p3 = rt_timer_read();
		now = rt_timer_read();

		if ( isSlaveInit() )
		{
            double_dt = (static_cast<double>(p3 - p1))*1e-3; 	// us
			double_gt = (static_cast<double>(p3 - start))*1e-9; // s
            calculation_time = (long)(now - previous);

            system_ready = 1;	//all drives have been done

            if ( worst_time < calculation_time )
                worst_time = calculation_time;

            if( calculation_time >= cycle_ns )
            {
                fault_count++;
                worst_time = 0;
            }
		}
		else
		{
			if(ecatmaster.GetConnectedSlaves() < ELMO_TOTAL)
			{
				//signal_handler(1);
			}

			if(system_ready)
			    down_count++;

			system_ready = 0;
			double_gt = 0;
			worst_time = 0;
            calculation_time = 0;
			start = rt_timer_read();
		}
	}
    if(xddp_s >= 0) close(xddp_s);
}

void tcpip_run()
{
    RTIME p1, p2, p3;

    PacketHandler packet;
    Poco::Net::SocketAddress server_addr(SERVER_PORT);
    Poco::Net::ServerSocket server_sock(server_addr);

    Poco::Net::Socket::SocketList connectedSockList;
    connectedSockList.push_back(server_sock);

    TCP_Packet_Task packet_task;
    TCP_Packet_Task packet_task_send;

    int nrt_fd = open("/dev/rtp0", O_RDWR);
    if(nrt_fd < 0) {
        printf("Failed to open NRT /dev/rtp0\n");
    }
    while(true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(8));
        if(break_flag)
            break;

        //p1 = rt_timer_read();
        Poco::Net::Socket::SocketList readList(connectedSockList.begin(), connectedSockList.end());
        Poco::Net::Socket::SocketList writeList(connectedSockList.begin(), connectedSockList.end());
        Poco::Net::Socket::SocketList exceptList(connectedSockList.begin(), connectedSockList.end());

        Poco::Timespan timeout;
        if( Poco::Net::Socket::select(readList, writeList, exceptList, timeout) != 0 && system_ready )
        {
            Poco::Net::Socket::SocketList delSockList;

            for (auto& readSock : readList)
            {
                if (server_sock == readSock)
                {
                    auto newSock = server_sock.acceptConnection();
                    connectedSockList.push_back(newSock);
                    //std::cout << "New Client connected" << std::endl;
                }
                else
                {
                    auto n = ((Poco::Net::StreamSocket*)&readSock)->receiveBytes(packet_task.data, sizeof(TCP_Packet_Task));
                    if (n > 0)
                    {
                        packet_task_send = packet_task;
                        if(nrt_fd >= 0) {
                            write(nrt_fd, &packet_task.data, sizeof(TCP_Packet_Task));
                        }

                        ((Poco::Net::StreamSocket*)&readSock)->sendBytes(packet_task_send.data, sizeof(TCP_Packet_Task));
                    }
                    else
                    {
                        //std::cout << "Client Disconnected" << std::endl;
                        delSockList.push_back(readSock);
                    }
                }
            }

            for (auto& delSock : delSockList)
            {
                auto delIter = std::find_if(connectedSockList.begin(),connectedSockList.end(),[&delSock](auto& sock){return delSock == sock ? true : false;});
                if (delIter != connectedSockList.end())
                {
                    connectedSockList.erase(delIter);
                    //std::cout << "Remove the Client from connectedSockList" << std::endl;
                }
            }
        }
        p3 = p2;
        p2 = rt_timer_read();
        calculation_time_tcp = (long)(p2 - p1);
        double_dt_tcp = (static_cast<double>(p2 - p3))*1e-3; 	// us

        if ( worst_time_tcp < calculation_time_tcp )
            worst_time_tcp = calculation_time_tcp;

        if( calculation_time_tcp >= tcp_cycle_ns )
        {
            fault_count_tcp++;
            worst_time_tcp = 0;
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
        if(break_flag)
            break;

		if ( system_ready )
		{
			rt_printf("Time=%0.2fs\n", double_gt);
			rt_printf("DesiredTask=%0.2fus, Calculation= %0.2fus, WorstCalculation= %0.2fus, RTFault=%d, EcatDown=%d\n",
                      double_dt, static_cast<double>(calculation_time)*1e-3, static_cast<double>(worst_time)*1e-3, fault_count, down_count);
#if defined(_TCPIP_ON_)
            rt_printf("DesiredTask(tcp)=%0.2fus, Calculation(tcp)= %0.2fus, WorstCalculation(tcp)= %0.2fus, RTFault(tcp)=%d\n",
                      double_dt_tcp, static_cast<double>(calculation_time_tcp)*1e-3, static_cast<double>(worst_time_tcp)*1e-3, fault_count_tcp);
#endif
            rt_printf("\nIndex1:0x%02X, Index2:0x%02X, SubIndex:0x%02X", ControlIndex1, ControlIndex2, ControlSubIndex);
			for(int j=0; j<DUAL_ARM_DOF; ++j)
			{
				rt_printf("\t \nID: %d,", j+1);

#if defined(_DEBUG_)
				//rt_printf(" StatWord: 0x%04X, ",	StatusWord[j]);
				//rt_printf(" DeviceState: %d, ",	DeviceState[j]);
				rt_printf(" ModeOfOp: %d,",			ModeOfOperationDisplay[j]);
				//rt_printf("\n");
#endif
				rt_printf("\tActPos(Deg): %0.2lf,", ActualPos_Rad(j)*RADtoDEG);
				rt_printf("\tTarPos(Deg): %0.2lf,", TargetPos_Rad(j)*RADtoDEG);
				//rt_printf("\tActPos(inc): %d,", ActualPos[j]);
				//rt_printf("\n");
				rt_printf("\tActVel(Deg/s): %0.1lf,", ActualVel_Rad(j)*RADtoDEG);
				rt_printf("\tTarVel(Deg/s): %0.1lf,", TargetVel_Rad(j)*RADtoDEG);
				//rt_printf("\tActVel(inc/s): %d,", ActualVel[j]);
				//rt_printf("\n");
				rt_printf("\tActTor(%): %d,", ActualTor[j]);
				rt_printf("\tCtrlTor(Nm): %0.1lf", TargetToq(j));
				//rt_printf("\tTarTor(%): %d", TargetTor[j]);
				//rt_printf("\n");
			}
            rt_printf("\n");
			rt_printf("\nForward Kinematics -->");
			for(int cNum = 0; cNum < NumChain; cNum++)
			{
				rt_printf("\n Num:%d: x:%0.3lf, y:%0.3lf, z:%0.3lf, u:%0.3lf, v:%0.3lf, w:%0.3lf",cNum,
              ForwardPos[cNum](0), ForwardPos[cNum](1), ForwardPos[cNum](2),
              ForwardOri[cNum](0)*RADtoDEG, ForwardOri[cNum](1)*RADtoDEG, ForwardOri[cNum](2)*RADtoDEG);
                rt_printf("\n Num:%d: dx:%0.3lf, dy:%0.3lf, dz:%0.3lf, du:%0.3lf, dv:%0.3lf, dw:%0.3lf",cNum,
                          TargetPos_Task(6*cNum+3), TargetPos_Task(6*cNum+4), TargetPos_Task(6*cNum+5),
                          TargetPos_Task(6*cNum)*RADtoDEG, TargetPos_Task(6*cNum+1)*RADtoDEG, TargetPos_Task(6*cNum+2)*RADtoDEG);
                rt_printf("\n Num:%d: e_x:%0.3lf, e_y:%0.3lf, e_z:%0.3lf, e_u:%0.3lf, e_v:%0.3lf, e_w:%0.3lf\n",cNum,
                          ErrorPos_Task(6*cNum+3), ErrorPos_Task(6*cNum+4), ErrorPos_Task(6*cNum+5),
                          ErrorPos_Task(6*cNum)*RADtoDEG, ErrorPos_Task(6*cNum+1)*RADtoDEG, ErrorPos_Task(6*cNum+2)*RADtoDEG);
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
                          ecatmaster.GetEcatMasterLinkState(), ecatmaster.GetEcatMasterState(), ecatmaster.GetConnectedSlaves());
				for(int i=0; i<((int)ecatmaster.GetConnectedSlaves()-1); i++)
				{
					rt_printf("\nID: %d , SlaveState: 0x%02X, SlaveConnection: %s, SlaveNMT: %s ", i,
                              ecatmaster.GetSlaveState(i), ecatmaster.GetSlaveConnected(i), ecatmaster.GetSlaveNMT(i));
					rt_printf(" SlaveStatus : %s,", DeviceState[i]);
					rt_printf(" StatWord: 0x%04X, ", StatusWord[i]);

				}
				rt_printf("\n");
			}
		}
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
        std::this_thread::sleep_for(std::chrono::milliseconds(8));
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
	    if (signum == SIGXCPU) {
        rt_printf("\n[WARNING] Mode switch detected! (SIGXCPU)\n");
        return;
    }
rt_printf("\nSignal Interrupt: %d", signum);
    break_flag=true;

#if defined(_KEYBOARD_ON_)
    rt_printf("\nEvent RTTask Closing....");
    rt_task_delete(&event_task);
    rt_printf("\nEvent RTTask Closing Success....");
#endif

#if defined(_TCPIP_ON_)
    rt_printf("\nTCPIP RTTask Closing....");
    if(tcpip_thread.joinable()) tcpip_thread.join();
    rt_printf("\nTCPIP RTTask Closing Success....");
#endif

#if defined(_PRINT_ON_)
    rt_printf("\nConsolPrint RTTask Closing....");
    rt_task_delete(&print_task);
    rt_printf("\nConsolPrint RTTask Closing Success....");
#endif

	rt_printf("\nControl RTTask Closing....");
	rt_task_delete(&RTArm_task);
	rt_printf("\nControl RTTask Closing Success....");

    rt_printf("\n\n\t !!RT-DualArm Client System Stopped!! \n");
    ecatmaster.deactivate();
}

/****************************************************************************/
int main(int argc, char **argv)
{
    signal(SIGHUP, signal_handler);
	signal(SIGINT, signal_handler);
    signal(SIGQUIT, signal_handler);
    signal(SIGIOT, signal_handler);
    signal(SIGFPE, signal_handler);
    signal(SIGKILL, signal_handler);
    signal(SIGXCPU, signal_handler);
    signal(SIGSEGV, signal_handler);
	signal(SIGTERM, signal_handler);

	/* Avoids memory swapping for this program */
	mlockall( MCL_CURRENT | MCL_FUTURE );

    // Perform auto-init of rt_print buffers if the task doesn't do so
    rt_print_auto_init(1);

	// TO DO: Specify the cycle period (cycle_ns) here, or use default value
	//cycle_ns = 500e3; // nanosecond -> 2kHz
	cycle_ns = 1000e3; // nanosecond -> 1kHz
	//cycle_ns = 1250e3; // nanosecond -> 800Hz
	//cycle_ns = 2000e3; // nanosecond -> 500Hz

#if defined(_ECAT_ON_)
	for(int SlaveNum=0; SlaveNum < ELMO_TOTAL; ++SlaveNum)
	{
		ecatmaster.addSlave(0, SlaveNum, &ecat_elmo[static_cast<size_t>(SlaveNum)]);
	}
    ecatmaster.activateWithDC(0, cycle_ns);
#endif

	// RTArm_task: create and start
	rt_printf("\n-- Now running rt tasks ...\n");

#if defined(_TCPIP_ON_)
    tcpip_thread = std::thread(tcpip_run);
#endif

#if defined(_PRINT_ON_)
    rt_task_create(&print_task, "Console_proc", 0, 20, 0);
    rt_task_start(&print_task, &print_run, nullptr);
#endif

	rt_task_create(&RTArm_task, "Control_proc", 1024*1024*4, 99, 0); // MUST SET at least 4MB stack-size (MAXIMUM Stack-size ; 8192 kbytes)
	rt_task_start(&RTArm_task, &RTRArm_run, nullptr);

#if defined(_KEYBOARD_ON_)
    rt_task_create(&event_task, "Event_proc", 0, 80, 0);
    rt_task_start(&event_task, &event_run, nullptr);
#endif
	// Must pause here
	pause();
    // task delete check
    ecatmaster.deactivate();

    return 0;
}