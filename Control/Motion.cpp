/*
 * Motion.cpp
 *
 *  Created on: 2019. 8. 1.
 *      Author: Administrator
 */

#include "Motion.h"

namespace HYUControl {

Motion::Motion() {

	this->pManipulator = NULL;
	TotalDoF=0;
	TotalChain=0;
	MotionProcess=0;
}

Motion::Motion(SerialManipulator *_pManipulator)
{
	this->pManipulator = _pManipulator;

	TotalDoF = pManipulator->GetTotalDoF();
	TotalChain = pManipulator->GetTotalChain();

	MotionProcess=0;

	dq.resize(TotalDoF);
	dqdot.resize(TotalDoF);
	dqddot.resize(TotalDoF);

	TargetPos.resize(TotalDoF);
	TargetPosTask.resize(7, TotalChain);
}

Motion::~Motion() {

}

uint16_t Motion::JointMotion(double *_dq, double *_dqdot, double *_dqddot, VectorXd &_Target, double *_q, double *_qdot, double &_Time, uint16_t &_StatusWord, uint16_t &_MotionType)
{
	this->MotionCommand = _MotionType;

	if(_Time >= 1.5 && _StatusWord == SYSTEM_BEGIN)
	{
		MotionCommand = MOVE_ZERO;
		//MotionCommand = MOVE_CLIK_JOINT;
		_MotionType = MotionCommand;
	}
	else if( MotionCommand == MOVE_ZERO && MotionProcess == MOVE_ZERO && _StatusWord == TARGET_ACHIEVED )
	{
		MotionCommand = MOVE_CUSTOMIZE;
		//MotionCommand = MOVE_JOB;
		//MotionCommand = MOVE_FRICTION;
		_MotionType = MotionCommand;
	}
	else if( MotionCommand == MOVE_JOB && MotionProcess == MOVE_JOB && _StatusWord == TARGET_ACHIEVED )
	{
		MotionCommand = MOVE_ZERO;
		//MotionCommand = MOVE_JOB;
		_MotionType = MotionCommand;
	}
	else if( MotionCommand == MOVE_CUSTOMIZE && MotionProcess == MOVE_CUSTOMIZE && _StatusWord == TARGET_ACHIEVED )
	{
		MotionCommand = MOVE_JOB;
		_MotionType = MotionCommand;
	}
	else if( MotionCommand == MOVE_JOINT_CYCLIC && MotionProcess == MOVE_JOINT_CYCLIC && (_Time >= MotionInitTime+20.0))
	{
		MotionCommand = MOVE_JOB;
		_MotionType = MotionCommand;
	}

	q = Map<VectorXd>(_q, pManipulator->GetTotalDoF());
	qdot = Map<VectorXd>(_qdot, pManipulator->GetTotalDoF());

	dq = Map<VectorXd>(_dq, pManipulator->GetTotalDoF());
	dqdot = Map<VectorXd>(_dqdot, pManipulator->GetTotalDoF());
	dqddot = Map<VectorXd>(_dqddot, pManipulator->GetTotalDoF());

	if( MotionCommand == MOVE_ZERO ) //home posture
	{
		if( MotionCommand == MotionCommand_p )
		{
			if(JointPoly5th.isReady() == 0 && NewTarget==1)
			{
				JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
				NewTarget=0;
			}
			else
				JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);

			MotionProcess = MOVE_ZERO;
		}
		else
		{
			TargetPos.setZero();

			_Target = TargetPos;

			TrajectoryTime=5.0;
			NewTarget=1;
		}
	}
	else if( MotionCommand == MOVE_JOB ) //job posture
	{
		if( MotionCommand == MotionCommand_p )
		{
			if( JointPoly5th.isReady() == 0 && NewTarget==1 )
			{
				JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
				NewTarget=0;
			}
			else
				JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);

			MotionProcess = MOVE_JOB;
		}
		else
		{
			TargetPos.setZero();
			TargetPos(1) = -0.0*DEGtoRAD;

			TargetPos(3) = -15.0*DEGtoRAD;
			TargetPos(3+7) = 15.0*DEGtoRAD;
			TargetPos(4) = -20.0*DEGtoRAD;
			TargetPos(4+7) = -TargetPos(4);
			TargetPos(6) = -45.0*DEGtoRAD;
			TargetPos(6+7) = -TargetPos(6);
			TargetPos(7) = 20.0*DEGtoRAD;
			TargetPos(7+7) = -TargetPos(7);

			_Target = TargetPos;

			TrajectoryTime=5.0;
			NewTarget=1;
		}
	}
	else if( MotionCommand == MOVE_CUSTOMIZE ) //custom
	{
		if( MotionCommand == MotionCommand_p )
		{
			if( JointPoly5th.isReady() == 0 && NewTarget==1 )
			{
				JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
				NewTarget=0;
			}
			else
				JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);

			MotionProcess = MOVE_CUSTOMIZE;
		}
		else
		{
			TargetPos.setZero();
			TargetPos(0) = 2.0*DEGtoRAD;
			TargetPos(1) = 0.0*DEGtoRAD;

			TargetPos(2) = -20.0*DEGtoRAD;
			TargetPos(2+7) = -TargetPos(2);
			TargetPos(3) = -30.0*DEGtoRAD;
			TargetPos(3+7) = -TargetPos(3);
			TargetPos(4) = -30.0*DEGtoRAD;
			TargetPos(4+7) = -TargetPos(4);
			TargetPos(5) = -45.0*DEGtoRAD;
			TargetPos(5+7) = -TargetPos(5);
			TargetPos(6) = -60.0*DEGtoRAD;
			TargetPos(6+7) = -TargetPos(6);
			TargetPos(7) = -20.0*DEGtoRAD;
			TargetPos(7+7) = -TargetPos(7);

			_Target = TargetPos;

			TrajectoryTime=5.0;
			NewTarget=1;
		}
	}
	else if( MotionCommand == MOVE_CUSTOMIZE1) //bow
	{
		if( MotionCommand == MotionCommand_p )
		{
			if( JointPoly5th.isReady() == 0 && NewTarget==1 )
			{
				JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
				NewTarget=0;
			}
			else
				JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);

			MotionProcess = MOVE_CUSTOMIZE1;
		}
		else
		{
			TargetPos.setZero();
			TargetPos(0) = 20.0*DEGtoRAD;
			TargetPos(1) = -5.0*DEGtoRAD;

			TargetPos(2) = -20.0*DEGtoRAD;
			TargetPos(2+7) = -TargetPos(2);
			TargetPos(3) = -30.0*DEGtoRAD;
			TargetPos(3+7) = -TargetPos(3);
			TargetPos(4) = -30.0*DEGtoRAD;
			TargetPos(4+7) = -TargetPos(4);
			TargetPos(5) = -45.0*DEGtoRAD;
			TargetPos(5+7) = -TargetPos(5);
			TargetPos(6) = -60.0*DEGtoRAD;
			TargetPos(6+7) = -TargetPos(6);
			TargetPos(7) = -20.0*DEGtoRAD;
			TargetPos(7+7) = -TargetPos(7);

			_Target = TargetPos;

			TrajectoryTime=5.0;
			NewTarget=1;
		}
	}
	else if( MotionCommand == MOVE_CUSTOMIZE2) //gym1
	{
		if( MotionCommand == MotionCommand_p )
		{
			if( JointPoly5th.isReady() == 0 && NewTarget==1 )
			{
				JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
				NewTarget=0;
			}
			else
				JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);

			MotionProcess = MOVE_CUSTOMIZE2;
		}
		else
		{
			TargetPos.setZero();
			TargetPos(0) = 20.0*DEGtoRAD;
			TargetPos(1) = -5.0*DEGtoRAD;

			TargetPos(2) = -20.0*DEGtoRAD;
			TargetPos(2+7) = -TargetPos(2);
			TargetPos(3) = -30.0*DEGtoRAD;
			TargetPos(3+7) = -TargetPos(3);
			TargetPos(4) = -30.0*DEGtoRAD;
			TargetPos(4+7) = -TargetPos(4);
			TargetPos(5) = -45.0*DEGtoRAD;
			TargetPos(5+7) = -TargetPos(5);
			TargetPos(6) = -60.0*DEGtoRAD;
			TargetPos(6+7) = -TargetPos(6);
			TargetPos(7) = -20.0*DEGtoRAD;
			TargetPos(7+7) = -TargetPos(7);

			_Target = TargetPos;

			TrajectoryTime=5.0;
			NewTarget=1;
		}
	}
	else if( MotionCommand == MOVE_CUSTOMIZE3) //gym2
	{
		if( MotionCommand == MotionCommand_p )
		{
			if( JointPoly5th.isReady() == 0 && NewTarget==1 )
			{
				JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
				NewTarget=0;
			}
			else
				JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);

			MotionProcess = MOVE_CUSTOMIZE3;
		}
		else
		{
			TargetPos.setZero();
			TargetPos(0) = 20.0*DEGtoRAD;
			TargetPos(1) = -5.0*DEGtoRAD;

			TargetPos(2) = -20.0*DEGtoRAD;
			TargetPos(2+7) = -TargetPos(2);
			TargetPos(3) = -30.0*DEGtoRAD;
			TargetPos(3+7) = -TargetPos(3);
			TargetPos(4) = -30.0*DEGtoRAD;
			TargetPos(4+7) = -TargetPos(4);
			TargetPos(5) = -45.0*DEGtoRAD;
			TargetPos(5+7) = -TargetPos(5);
			TargetPos(6) = -60.0*DEGtoRAD;
			TargetPos(6+7) = -TargetPos(6);
			TargetPos(7) = -20.0*DEGtoRAD;
			TargetPos(7+7) = -TargetPos(7);

			_Target = TargetPos;

			TrajectoryTime=5.0;
			NewTarget=1;
		}
	}
	else if( MotionCommand == MOVE_JOINT_CYCLIC )
	{
		if( MotionCommand != MotionCommand_p )
		{
			MotionInitTime = _Time;
		}
		else
		{
			_T = 18.0;
			_omega = 2.0*M_PI/_T;
			_amp = 70;

			dq(0) = _amp*M_PI/180.0*sin(_omega*(_Time-MotionInitTime));
			dqdot(0) = _amp*M_PI/180.0*_omega*cos(_omega*(_Time-MotionInitTime));
			dqddot(0) = -_amp*M_PI/180*pow(_omega,2)*sin(_omega*(_Time-MotionInitTime));

			_T = 10.0;
			_omega = 2.0*M_PI/_T;
			_amp = 20;

			dq(1) = _amp*M_PI/180.0*sin(_omega*(_Time-MotionInitTime)+0.8481) - 15*M_PI/180.0;
			dqdot(1) = _amp*M_PI/180.0*_omega*cos(_omega*(_Time-MotionInitTime)+0.8481);
			dqddot(1) = -_amp*M_PI/180*pow(_omega,2)*sin(_omega*(_Time-MotionInitTime)+0.8481);

			_T = 7.0;
			_omega = 2.0*M_PI/_T;
			_amp = 50;

			dq(2) = _amp*M_PI/180.0*sin(_omega*(_Time-MotionInitTime)+0.6435) - 30*M_PI/180.0;
			dqdot(2) = _amp*M_PI/180.0*_omega*cos(_omega*(_Time-MotionInitTime)+0.6435);
			dqddot(2) = -_amp*M_PI/180*pow(_omega,2)*sin(_omega*(_Time-MotionInitTime)+0.6435);

			dq(2+6) = -dq(2);
			dqdot(2+6) = -dqdot(2);
			dqddot(2+6) = -dqddot(2);

			_T = 7.0;
			_omega = 2.0*M_PI/_T;
			_amp = 50;

			dq(3) = _amp*M_PI/180.0*sin(_omega*(_Time-MotionInitTime)+0.4115) - 20*M_PI/180.0;
			dqdot(3) = _amp*M_PI/180.0*_omega*cos(_omega*(_Time-MotionInitTime)+0.4115);
			dqddot(3) = -_amp*M_PI/180*pow(_omega,2)*sin(_omega*(_Time-MotionInitTime)+0.4115);

			dq(3+6) = -dq(3);
			dqdot(3+6) = -dqdot(3);
			dqddot(3+6) = -dqddot(3);

			_T = 7.0;
			_omega = 2.0*M_PI/_T;
			_amp = 45;

			dq(4) = _amp*M_PI/180.0*sin(_omega*(_Time-MotionInitTime)+0.729) - 30*M_PI/180.0;
			dqdot(4) = _amp*M_PI/180.0*_omega*cos(_omega*(_Time-MotionInitTime)+0.729);
			dqddot(4) = -_amp*M_PI/180*pow(_omega,2)*sin(_omega*(_Time-MotionInitTime)+0.729);

			dq(4+6) = -dq(4);
			dqdot(4+6) = -dqdot(4);
			dqddot(4+6) = -dqddot(4);

			_T = 7.0;
			_omega = 2.0*M_PI/_T;
			_amp = 70.0;

			dq(5) = _amp*M_PI/180.0*sin(_omega*(_Time-MotionInitTime));
			dqdot(5) = _amp*M_PI/180.0*_omega*cos(_omega*(_Time-MotionInitTime));
			dqddot(5) = -_amp*M_PI/180*pow(_omega,2)*sin(_omega*(_Time-MotionInitTime));

			dq(5+6) = -dq(5);
			dqdot(5+6) = -dqdot(5);
			dqddot(5+6) = -dqddot(5);

			_T = 7.0;
			_omega = 2.0*M_PI/_T;
			_amp = 40.0;

			dq(6) = _amp*M_PI/180.0*sin(_omega*(_Time-MotionInitTime));
			dqdot(6) = _amp*M_PI/180.0*_omega*cos(_omega*(_Time-MotionInitTime));
			dqddot(6) = -_amp*M_PI/180*pow(_omega,2)*sin(_omega*(_Time-MotionInitTime));

			dq(6+6) = -dq(6);
			dqdot(6+6) = -dqdot(6);
			dqddot(6+6) = -dqddot(6);

			_T = 7.0;
			_omega = 2.0*M_PI/_T;
			_amp = 40.0;

			dq(7) = _amp*M_PI/180.0*sin(_omega*(_Time-MotionInitTime));
			dqdot(7) = _amp*M_PI/180.0*_omega*cos(_omega*(_Time-MotionInitTime));
			dqddot(7) = -_amp*M_PI/180*pow(_omega,2)*sin(_omega*(_Time-MotionInitTime));

			dq(7+6) = dq(7);
			dqdot(7+6) = dqdot(7);
			dqddot(7+6) = dqddot(7);

			MotionProcess = MOVE_JOINT_CYCLIC;
		}
	}

	MotionCommand_p = MotionCommand;

	Map<VectorXd>(_dq, pManipulator->GetTotalDoF()) = dq;
	Map<VectorXd>(_dqdot, pManipulator->GetTotalDoF()) = dqdot;
	Map<VectorXd>(_dqddot, pManipulator->GetTotalDoF()) = dqddot;

	return MotionProcess;
}

uint16_t Motion::TaskMotion( VectorXd *_dx, VectorXd *_dxdot, VectorXd *_dxddot, double &_Time, uint16_t &_StatusWord, uint16_t &_MotionType )
{
	MotionCommandTask = _MotionType;

	//int TotalTask = 4*TotalChain;

	if( MotionCommandTask == MOVE_CLIK_JOINT )
	{
		if( MotionCommandTask == MotionCommandTask_p )
		{
			//if( TaskPoly5th.isReady()==0 && NewTarget==1 )
			//{
			//	TaskPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalTask);
			//	NewTarget=0;
			//}
			//else
			//{
			//	TaskPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
			//}

			for(int i=0; i<TotalChain; i++)
			{
				//_dx[i].resize(7);
				_dx[i].head(3) = TargetPosTask.block(0,i,3,1);
				_dx[i](3) = TargetPosTask(3,i);
				_dx[i].tail(3) = TargetPosTask.block(4,i,3,1);

				_dxdot[i].setZero();
				_dxddot[i].setZero();
			}

			MotionProcess = MOVE_CLIK_JOINT;
		}
		else
		{
			TargetPosTask.setZero();
			TargetPosTask.block(0,0,3,1) << 0, -1, 0;
			TargetPosTask.block(3,0,1,1) << 90.0*DEGtoRAD;
			TargetPosTask.block(4,0,3,1) << 0.310, -0.310, 0.420;

			TargetPosTask.block(0,1,3,1) << 0, -1, 0;
			TargetPosTask.block(3,1,1,1) << 90.0*DEGtoRAD;
			TargetPosTask.block(4,1,3,1) << 0.310, 0.310, 0.420;

			TrajectoryTime=5.0;
			NewTarget=1;
		}
	}

	MotionCommandTask_p = MotionCommandTask;

	return MotionProcess;
}

} /* namespace hyuCtrl */
