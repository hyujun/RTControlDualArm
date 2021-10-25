
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

Motion::Motion(std::shared_ptr<SerialManipulator> Manipulator)
{
	this->pManipulator = Manipulator;

	TotalDoF = pManipulator->GetTotalDoF();
	TotalChain = pManipulator->GetTotalChain();

	MotionProcess=0;
    MotionCommand_p = 0;

	TargetPos.setZero(TotalDoF);
    TargetPos_p.setZero(TotalDoF);

	TargetPosTask.setZero(12);
    TargetPosTask_p.setZero(12);

	TargetPos_Linear.setZero(6);
}

Motion::~Motion() {

}

uint16_t Motion::JointMotion(VectorXd &dq, VectorXd &dqdot, VectorXd &dqddot,
                             VectorXd &_Target, const VectorXd &q, const VectorXd &qdot,
                             double &_Time, unsigned char &_StatusWord, unsigned char &_MotionType)
{
	this->MotionCommand = _MotionType;

	dq.setZero(16);
    dqdot.setZero(16);
    dqddot.setZero(16);

	if( MotionCommand == MOVE_ZERO ) //home posture
	{
		if( _StatusWord != MotionCommand )
		{
			if( NewTarget==1 )
			{
				JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
				NewTarget=0;
			}
			else
				JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);

			MotionProcess = MOVE_ZERO;
		}
		else
		{
			TargetPos.setZero(16);
//            TargetPos(0) = -0.0*DEGtoRAD;
//            TargetPos(7) = -20.0*DEGtoRAD;
//            TargetPos(8) = 15.0*DEGtoRAD;
//            TargetPos(8+7) = -15.0*DEGtoRAD;

//            TargetPos(5) = -10.0*DEGtoRAD;
//            TargetPos(5+7)=-TargetPos(5);
//            TargetPos(6) = -30.00*DEGtoRAD;
//            TargetPos(6+7)=-TargetPos(6);
			TrajectoryTime=6.0;
			NewTarget=1;
            _Target = TargetPos;
			_StatusWord = 0;
		}
	}
	else if( MotionCommand == MOVE_JOB ) //job posture
	{
        if( _StatusWord != MotionCommand )
		{
			if( NewTarget==1 )
			{
				JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
				NewTarget=0;
			}
			else
				JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);

			MotionProcess = MOVE_JOB;
		}
		else
		{
			TargetPos.setZero(16);

//            TargetPos(5) = -30.0*DEGtoRAD;
//            TargetPos(5+7) =-TargetPos(5);

            TargetPos(6) = -45.0*DEGtoRAD;
            TargetPos(6+7) =-TargetPos(6);


            TargetPos(6) = -30.0*DEGtoRAD;
            TargetPos(6+7) =-TargetPos(6);

            _Target = TargetPos;

			TrajectoryTime=6.0;
			NewTarget=1;
			_StatusWord = 0;
		}
	}
    else if( MotionCommand == MOVE_CUSTOMIZE1 ) //job posture
    {
        if( _StatusWord != MotionCommand )
        {
            if( NewTarget==1 )
            {
                JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
                NewTarget=0;
            }
            else
                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);

            MotionProcess = MOVE_CUSTOMIZE1;
        }
        else
        {
            TargetPos.setZero(16);




            TargetPos(6) = -45.0*DEGtoRAD;
            TargetPos(6+7) =-TargetPos(6);

            TargetPos(8) = -45.0*DEGtoRAD;
            TargetPos(8+7) =-TargetPos(8);



            _Target = TargetPos;

            TrajectoryTime=7.0;
            NewTarget=1;
            _StatusWord = 0;
        }
    }
    else if( MotionCommand == MOVE_CUSTOMIZE2 )
    {
        if( _StatusWord != MotionCommand )
        {
            if( NewTarget==1 )
            {
                JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
                NewTarget=0;
            }
            else
                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);

            MotionProcess = MOVE_CUSTOMIZE2;
        }
        else
        {
            TargetPos.setZero(16);


            TargetPos(2) = -1.26*DEGtoRAD;
            TargetPos(3) = -3.15*DEGtoRAD;
            TargetPos(4) = 0.04*DEGtoRAD;
            TargetPos(5) = -4.02*DEGtoRAD;
            TargetPos(6) = -39.09*DEGtoRAD;
            TargetPos(7) = 0.09*DEGtoRAD;
            TargetPos(8) = -49.31*DEGtoRAD;

            TargetPos(9) = 0.05*DEGtoRAD;
            TargetPos(10) = 8.07*DEGtoRAD;
            TargetPos(11) = 1.03*DEGtoRAD;
            TargetPos(12) = -0.08*DEGtoRAD;
            TargetPos(13) = 37.77*DEGtoRAD;
            TargetPos(14) = 6.72*DEGtoRAD;
            TargetPos(15) = 45.25*DEGtoRAD;


            _Target = TargetPos;

            TrajectoryTime=7.0;
            NewTarget=1;
            _StatusWord = 0;
        }
    }
    else if( MotionCommand == MOVE_CUSTOMIZE3 )
    {
        if( _StatusWord != MotionCommand )
        {
            if( NewTarget==1 )
            {
                JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
                NewTarget=0;
            }
            else
                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);

            MotionProcess = MOVE_CUSTOMIZE3;
        }
        else
        {
            TargetPos.setZero(16);

            TargetPos(2) = -1.28*DEGtoRAD;
            TargetPos(3) = -8.20*DEGtoRAD;
            TargetPos(4) = 8.04*DEGtoRAD;
            TargetPos(5) = 1.28*DEGtoRAD;
            TargetPos(6) = -36.43*DEGtoRAD;
            TargetPos(7) = 6.10*DEGtoRAD;
            TargetPos(8) = -45.53*DEGtoRAD;

            TargetPos(9) = 0.06*DEGtoRAD;
            TargetPos(10) = 11.04*DEGtoRAD;
            TargetPos(11) = -12.82*DEGtoRAD;
            TargetPos(12) = -0.08*DEGtoRAD;
            TargetPos(13) = 40.23*DEGtoRAD;
            TargetPos(14) = 3.00*DEGtoRAD;
            TargetPos(15) = 44.72*DEGtoRAD;


            _Target = TargetPos;

            TrajectoryTime=6.0;
            NewTarget=1;
            _StatusWord = 0;
        }
    }
    else if( MotionCommand == MOVE_CUSTOMIZE4 )
    {
        if( _StatusWord != MotionCommand )
        {
            if( NewTarget==1 )
            {
                JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
                NewTarget=0;
            }
            else
                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);

            MotionProcess = MOVE_CUSTOMIZE4;
        }
        else
        {
            TargetPos.setZero(16);

            TargetPos(2) = -1.27*DEGtoRAD;
            TargetPos(3) = -7.88*DEGtoRAD;
            TargetPos(4) = 9.63*DEGtoRAD;
            TargetPos(5) = 15.76*DEGtoRAD;
            TargetPos(6) = -44.12*DEGtoRAD;
            TargetPos(7) = 4.30*DEGtoRAD;
            TargetPos(8) = -18.37*DEGtoRAD;

            TargetPos(9) = 0.06*DEGtoRAD;
            TargetPos(10) = 10.62*DEGtoRAD;
            TargetPos(11) = -19.53*DEGtoRAD;
            TargetPos(12) = -0.08*DEGtoRAD;
            TargetPos(13) = 26.68*DEGtoRAD;
            TargetPos(14) = -11.86*DEGtoRAD;
            TargetPos(15) = 54.39*DEGtoRAD;


            _Target = TargetPos;

            TrajectoryTime=10.0;
            NewTarget=1;
            _StatusWord = 0;
        }
    }
    else if( MotionCommand == MOVE_CUSTOMIZE5 )
    {
        if( _StatusWord != MotionCommand )
        {
            if( NewTarget==1 )
            {
                JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
                NewTarget=0;
            }
            else
                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);

            MotionProcess = MOVE_CUSTOMIZE5;
        }
        else
        {
            TargetPos.setZero(16);

            TargetPos(3) = 20.0*DEGtoRAD;
            TargetPos(3+7) =-TargetPos(3);

            TargetPos(6) =-70.0*DEGtoRAD;
            TargetPos(6+7) =70.0*DEGtoRAD;

            TargetPos(7) =-90.0*DEGtoRAD;






            _Target = TargetPos;

            TrajectoryTime=8.0;
            NewTarget=1;
            _StatusWord = 0;
        }
    }
    else if( MotionCommand == MOVE_CUSTOMIZE6)
    {
        if( _StatusWord != MotionCommand )
        {
            if( NewTarget==1 )
            {
                JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
                NewTarget=0;
            }
            else
                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);

            MotionProcess = MOVE_CUSTOMIZE6;
        }
        else
        {
            TargetPos.setZero(16);

            TargetPos(2) = 1.11*DEGtoRAD;
            TargetPos(3) = 13.26*DEGtoRAD;
            TargetPos(4) = -1.78*DEGtoRAD;
            TargetPos(5) = -4.45*DEGtoRAD;
            TargetPos(6) = -58.74*DEGtoRAD;
            TargetPos(7) = -90.23*DEGtoRAD;
            TargetPos(8) = 3.52*DEGtoRAD;

            TargetPos(3+7) =-13.26*DEGtoRAD;
            TargetPos(6+7) = 58.74*DEGtoRAD;


            _Target = TargetPos;

            TrajectoryTime=7.0;
            NewTarget=1;
            _StatusWord = 0;
        }
    }
    else if( MotionCommand == MOVE_CUSTOMIZE7 )
    {
        if( _StatusWord != MotionCommand )
        {
            if( NewTarget==1 )
            {
                JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
                NewTarget=0;
            }
            else
                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);

            MotionProcess = MOVE_CUSTOMIZE7;
        }
        else
        {
            TargetPos.setZero(16);

            TargetPos(2) = 0.04*DEGtoRAD;
            TargetPos(3) = 4.36*DEGtoRAD;
            TargetPos(4) = -4.66*DEGtoRAD;
            TargetPos(5) = -8.86*DEGtoRAD;
            TargetPos(6) = -48.34*DEGtoRAD;
            TargetPos(7) = -89.77*DEGtoRAD;
            TargetPos(8) = 8.57*DEGtoRAD;

            TargetPos(3+7) =-4.36*DEGtoRAD;
            TargetPos(6+7) = 48.34*DEGtoRAD;

            _Target = TargetPos;

            TrajectoryTime=6.0;
            NewTarget=1;
            _StatusWord = 0;
        }
    }
    else if( MotionCommand == MOVE_CUSTOMIZE8 )
    {
        if( _StatusWord != MotionCommand )
        {
            if( NewTarget==1 )
            {
                JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
                NewTarget=0;
            }
            else
                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);

            MotionProcess = MOVE_CUSTOMIZE8;
        }
        else
        {
            TargetPos.setZero(16);

            TargetPos(2) = 0.03*DEGtoRAD;
            TargetPos(3) = -2.68*DEGtoRAD;
            TargetPos(4) = -4.66*DEGtoRAD;
            TargetPos(5) = -8.86*DEGtoRAD;
            TargetPos(6) = -38.31*DEGtoRAD;
            TargetPos(7) = -89.77*DEGtoRAD;
            TargetPos(8) = 8.55*DEGtoRAD;

            TargetPos(3+7) =2.68*DEGtoRAD;
            TargetPos(6+7) = 38.31*DEGtoRAD;



            _Target = TargetPos;

            TrajectoryTime=10.0;
            NewTarget=1;
            _StatusWord = 0;
        }
    }
    else if( MotionCommand == MOVE_CUSTOMIZE9 )
    {
        if( _StatusWord != MotionCommand )
        {
            if( NewTarget==1 )
            {
                JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
                NewTarget=0;
            }
            else
                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);

            MotionProcess = MOVE_CUSTOMIZE9;
        }
        else
        {
            TargetPos.setZero(16);

            TargetPos(2) = 0.65*DEGtoRAD;
            TargetPos(3) = 1.32*DEGtoRAD;
            TargetPos(4) = -4.66*DEGtoRAD;
            TargetPos(5) = -8.77*DEGtoRAD;
            TargetPos(6) = -44.27*DEGtoRAD;
            TargetPos(7) = -89.77*DEGtoRAD;
            TargetPos(8) = 9.09*DEGtoRAD;

            TargetPos(3+7) = -1.32*DEGtoRAD;
            TargetPos(6+7) = 44.27*DEGtoRAD;


            _Target = TargetPos;

            TrajectoryTime=5.0;
            NewTarget=1;
            _StatusWord = 0;
        }
    }
    else if( MotionCommand == MOVE_CUSTOMIZE10 )
    {
        if( _StatusWord != MotionCommand )
        {
            if( NewTarget==1 )
            {
                JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
                NewTarget=0;
            }
            else
                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);

            MotionProcess = MOVE_CUSTOMIZE10;
        }
        else
        {
            TargetPos.setZero(16);

            TargetPos(2) = 1.10*DEGtoRAD;
            TargetPos(3) = 6.86*DEGtoRAD;
            TargetPos(4) = 0.72*DEGtoRAD;
            TargetPos(5) = 1.27*DEGtoRAD;
            TargetPos(6) = -52.79*DEGtoRAD;
            TargetPos(7) = -89.95*DEGtoRAD;
            TargetPos(8) = -2.63*DEGtoRAD;

            TargetPos(3+7) = -6.86*DEGtoRAD;
            TargetPos(6+7) = 52.79*DEGtoRAD;


            _Target = TargetPos;

            TrajectoryTime=6.0;
            NewTarget=1;
            _StatusWord = 0;
        }
    }
    else if( MotionCommand == MOVE_CUSTOMIZE11 )
    {
        if( _StatusWord != MotionCommand )
        {
            if( NewTarget==1 )
            {
                JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
                NewTarget=0;
            }
            else
                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);

            MotionProcess = MOVE_CUSTOMIZE11;
        }
        else
        {
            TargetPos.setZero(16);

            TargetPos(2) = 1.10*DEGtoRAD;
            TargetPos(3) = -2.87*DEGtoRAD;
            TargetPos(4) = 0.72*DEGtoRAD;
            TargetPos(5) = 1.27*DEGtoRAD;
            TargetPos(6) = -39.09*DEGtoRAD;
            TargetPos(7) = -89.95*DEGtoRAD;
            TargetPos(8) = -2.62*DEGtoRAD;

            TargetPos(3+7) = 2.87*DEGtoRAD;
            TargetPos(6+7) = 39.09*DEGtoRAD;


            _Target = TargetPos;

            TrajectoryTime=10.0;
            NewTarget=1;
            _StatusWord = 0;
        }
    }
    else if( MotionCommand == MOVE_CUSTOMIZE12 )
    {
        if( _StatusWord != MotionCommand )
        {
            if( NewTarget==1 )
            {
                JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
                NewTarget=0;
            }
            else
                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);

            MotionProcess = MOVE_CUSTOMIZE12;
        }
        else
        {
            TargetPos.setZero(16);

            TargetPos(2) = 1.10*DEGtoRAD;
            TargetPos(3) = 1.88*DEGtoRAD;
            TargetPos(4) = 0.72*DEGtoRAD;
            TargetPos(5) = 1.27*DEGtoRAD;
            TargetPos(6) = -45.40*DEGtoRAD;
            TargetPos(7) = -89.95*DEGtoRAD;
            TargetPos(8) = -2.62*DEGtoRAD;

            TargetPos(3+7) = -1.88*DEGtoRAD;
            TargetPos(6+7) = 45.40*DEGtoRAD;


            _Target = TargetPos;

            TrajectoryTime=5.0;
            NewTarget=1;
            _StatusWord = 0;
        }
    }
//	else if( MotionCommand == MOVE_CUSTOMIZE )
//	{
//        if( _StatusWord != MotionCommand )
//		{
//			if( NewTarget == 1 )
//			{
//				JointPoly5th.SetPoly5th(_Time, q, qdot, TargetPos, TrajectoryTime, TotalDoF);
//                JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
//				NewTarget=0;
//			}
//			else
//				JointPoly5th.Poly5th(_Time, dq, dqdot, dqddot);
//
//			MotionProcess = MOVE_CUSTOMIZE;
//		}
//		else
//		{
//			TargetPos.setZero();
//            TargetPos = _Target;
//            TargetPos_p = TargetPos;
//			TrajectoryTime=5.0;
//			NewTarget=1;
//			_StatusWord = 0;
//		}
//	}
//	else if( MotionCommand == MOVE_JOINT_CYCLIC )
//	{
//        if( _StatusWord == MotionCommand )
//		{
//			MotionInitTime = _Time;
//			_StatusWord = 0;
//		}
//		else
//		{
//			_T = 18.0;
//			_omega = 2.0*M_PI/_T;
//			_amp = 70;
//
//			dq(0) = _amp*M_PI/180.0*sin(_omega*(_Time-MotionInitTime));
//			dqdot(0) = _amp*M_PI/180.0*_omega*cos(_omega*(_Time-MotionInitTime));
//			dqddot(0) = -_amp*M_PI/180*pow(_omega,2)*sin(_omega*(_Time-MotionInitTime));
//
//			_T = 10.0;
//			_omega = 2.0*M_PI/_T;
//			_amp = 20;
//
//			dq(1) = _amp*M_PI/180.0*sin(_omega*(_Time-MotionInitTime)+0.8481) - 15*M_PI/180.0;
//			dqdot(1) = _amp*M_PI/180.0*_omega*cos(_omega*(_Time-MotionInitTime)+0.8481);
//			dqddot(1) = -_amp*M_PI/180*pow(_omega,2)*sin(_omega*(_Time-MotionInitTime)+0.8481);
//
//			_T = 7.0;
//			_omega = 2.0*M_PI/_T;
//			_amp = 50;
//
//			dq(2) = _amp*M_PI/180.0*sin(_omega*(_Time-MotionInitTime)+0.6435) - 30*M_PI/180.0;
//			dqdot(2) = _amp*M_PI/180.0*_omega*cos(_omega*(_Time-MotionInitTime)+0.6435);
//			dqddot(2) = -_amp*M_PI/180*pow(_omega,2)*sin(_omega*(_Time-MotionInitTime)+0.6435);
//
//			dq(2+6) = -dq(2);
//			dqdot(2+6) = -dqdot(2);
//			dqddot(2+6) = -dqddot(2);
//
//			_T = 7.0;
//			_omega = 2.0*M_PI/_T;
//			_amp = 50;
//
//			dq(3) = _amp*M_PI/180.0*sin(_omega*(_Time-MotionInitTime)+0.4115) - 20*M_PI/180.0;
//			dqdot(3) = _amp*M_PI/180.0*_omega*cos(_omega*(_Time-MotionInitTime)+0.4115);
//			dqddot(3) = -_amp*M_PI/180*pow(_omega,2)*sin(_omega*(_Time-MotionInitTime)+0.4115);
//
//			dq(3+6) = -dq(3);
//			dqdot(3+6) = -dqdot(3);
//			dqddot(3+6) = -dqddot(3);
//
//			_T = 7.0;
//			_omega = 2.0*M_PI/_T;
//			_amp = 45;
//
//			dq(4) = _amp*M_PI/180.0*sin(_omega*(_Time-MotionInitTime)+0.729) - 30*M_PI/180.0;
//			dqdot(4) = _amp*M_PI/180.0*_omega*cos(_omega*(_Time-MotionInitTime)+0.729);
//			dqddot(4) = -_amp*M_PI/180*pow(_omega,2)*sin(_omega*(_Time-MotionInitTime)+0.729);
//
//			dq(4+6) = -dq(4);
//			dqdot(4+6) = -dqdot(4);
//			dqddot(4+6) = -dqddot(4);
//
//			_T = 7.0;
//			_omega = 2.0*M_PI/_T;
//			_amp = 70.0;
//
//			dq(5) = _amp*M_PI/180.0*sin(_omega*(_Time-MotionInitTime));
//			dqdot(5) = _amp*M_PI/180.0*_omega*cos(_omega*(_Time-MotionInitTime));
//			dqddot(5) = -_amp*M_PI/180*pow(_omega,2)*sin(_omega*(_Time-MotionInitTime));
//
//			dq(5+6) = -dq(5);
//			dqdot(5+6) = -dqdot(5);
//			dqddot(5+6) = -dqddot(5);
//
//			_T = 7.0;
//			_omega = 2.0*M_PI/_T;
//			_amp = 40.0;
//
//			dq(6) = _amp*M_PI/180.0*sin(_omega*(_Time-MotionInitTime));
//			dqdot(6) = _amp*M_PI/180.0*_omega*cos(_omega*(_Time-MotionInitTime));
//			dqddot(6) = -_amp*M_PI/180*pow(_omega,2)*sin(_omega*(_Time-MotionInitTime));
//
//			dq(6+6) = -dq(6);
//			dqdot(6+6) = -dqdot(6);
//			dqddot(6+6) = -dqddot(6);
//
//			_T = 7.0;
//			_omega = 2.0*M_PI/_T;
//			_amp = 40.0;
//
//			dq(7) = _amp*M_PI/180.0*sin(_omega*(_Time-MotionInitTime));
//			dqdot(7) = _amp*M_PI/180.0*_omega*cos(_omega*(_Time-MotionInitTime));
//			dqddot(7) = -_amp*M_PI/180*pow(_omega,2)*sin(_omega*(_Time-MotionInitTime));
//
//			dq(7+6) = dq(7);
//			dqdot(7+6) = dqdot(7);
//			dqddot(7+6) = dqddot(7);
//
//			MotionProcess = MOVE_JOINT_CYCLIC;
//		}
//	}

	MotionCommand_p = MotionCommand;
	return MotionProcess;
}

uint16_t Motion::TaskMotion( VectorXd &_dx, VectorXd &_dxdot, VectorXd &_dxddot,
                             VectorXd _Target, const VectorXd &x, const VectorXd &qdot,
                             double &_Time, unsigned char &_StatusWord, unsigned char &_MotionType )
{
	MotionCommandTask = _MotionType;

	pManipulator->pKin->GetAnalyticJacobian(AJacobian);
	xdot.setZero(12);
	xdot.noalias() += AJacobian*qdot;

    _dx.setZero(12);
    _dxdot.setZero(12);
    _dxddot.setZero(12);

    auto A = 0.12;
    auto b1 = 0.64;
    auto b2 = -0.33;
    auto b3 = 0.45;
    auto f = 0.2;

    auto l_p1 = 0.58;
    auto l_p2 = 0.33;
    auto l_p3 = 0.42;

    if( MotionCommandTask == MOVE_TASK_CUSTOM )
    {
        if( _StatusWord != MotionCommandTask )
        {
            if( NewTarget==1 )
            {
                int target_size = 6;
                _dx_tmp.setZero(target_size);
                _dxdot_tmp.setZero(target_size);
                _dxddot_tmp.setZero(target_size);

                _x_tmp.setZero(target_size);
                _xdot_tmp.setZero(target_size);

                _x_tmp.head(3) = x.segment(3,3);
                _x_tmp.tail(3) = x.segment(9,3);

                _xdot_tmp.head(3) = xdot.segment(3,3);
                _xdot_tmp.tail(3) = xdot.segment(9,3);

                TaskPoly5th.SetPoly5th(_Time, _x_tmp, _xdot_tmp, TargetPos_Linear, TrajectoryTime, target_size);
                TaskPoly5th.Poly5th(_Time, _dx_tmp, _dxdot_tmp, _dxddot_tmp);
                NewTarget=0;
            }
            else
            {
                TaskPoly5th.Poly5th(_Time, _dx_tmp, _dxdot_tmp, _dxddot_tmp);
            }

            _dx = _Target;
            _dx.segment(3,3) = _dx_tmp.head(3);
            _dx.segment(9,3) = _dx_tmp.tail(3);
            _dxdot.segment(3,3) = _dxdot_tmp.head(3);
            _dxdot.segment(9,3) = _dxdot_tmp.tail(3);
            _dxddot.segment(3,3) = _dxddot_tmp.head(3);
            _dxddot.segment(9,3) = _dxddot_tmp.tail(3);

            MotionProcess = MOVE_TASK_CUSTOM;
        }
        else
        {
            TargetPosTask = _Target;
            TargetPosTask_p = TargetPosTask;

            TargetPos_Linear.setZero(6);
            TargetPos_Linear.head(3) = TargetPosTask.segment(3,3);
            TargetPos_Linear.tail(3) = TargetPosTask.segment(9,3);

            TrajectoryTime=5.0;
            NewTarget=1;
            _StatusWord = 0;
        }
    }
	else if( MotionCommandTask == MOVE_TASK_CUSTOM1 )
	{
        if( _StatusWord == MotionCommandTask )
        {
            MotionInitTime = _Time;
            _StatusWord=0;
            start_pos.setZero(12);
            start_pos = x;
        }
        else
        {
            _dx(0) = start_pos(0);
            _dx(1) = start_pos(1);
            _dx(2) = start_pos(2);
            _dx(3) = A * sin(f * M_PI * (_Time - MotionInitTime)) + start_pos(3);
            _dx(4) = start_pos(4);
            _dx(5) = start_pos(5);

            _dx(6) = start_pos(6);
            _dx(7) = start_pos(7);
            _dx(8) = start_pos(8);
            _dx(9) = start_pos(9);
            _dx(10) = start_pos(10);
            _dx(11) = start_pos(11);

            _dxdot.setZero(12);
            _dxdot(3) = (f * M_PI) * A * cos(f * M_PI * (_Time - MotionInitTime));
//            _dxdot(9) = -(f * M_PI) * A * cos(f * M_PI * (_Time - MotionInitTime));

            _dxddot.setZero(12);
            _dxddot(3) = -(f * M_PI) * (f * M_PI) * A * sin(f * M_PI * (_Time - MotionInitTime));
//            _dxddot(9) = (f * M_PI) * (f * M_PI) * A * sin(f * M_PI * (_Time - MotionInitTime));
            _StatusWord = 0;
		}
	}
    else if( MotionCommandTask == MOVE_TASK_CUSTOM2 )
    {
        if( _StatusWord == MotionCommandTask )
        {
            MotionInitTime = _Time;
            _StatusWord=0;
            start_pos.setZero(12);
            start_pos = x;
        }
        else
        {
            A = 0.07;
            _dx(0) = start_pos(0);
            _dx(1) = start_pos(1);
            _dx(2) = start_pos(2);
            _dx(3) = start_pos(3);
            _dx(4) = A * sin(f * M_PI * (_Time - MotionInitTime)) + start_pos(4);
            _dx(5) = start_pos(5);

            _dx(6) = start_pos(6);
            _dx(7) = start_pos(7);
            _dx(8) = start_pos(8);
            _dx(9) = start_pos(9);
            _dx(10) = start_pos(10);
            _dx(11) = start_pos(11);

            _dxdot(4) = (f * M_PI) * A * cos(f * M_PI * (_Time - MotionInitTime));
            _dxddot(4) = -(f * M_PI) * (f * M_PI) * A * sin(f * M_PI * (_Time - MotionInitTime));
        }
    }

    else if( MotionCommandTask == MOVE_TASK_CUSTOM3 )
    {
        if( _StatusWord == MotionCommandTask )
        {
            MotionInitTime = _Time;
            _StatusWord=0;
            start_pos.setZero(12);
            start_pos = x;
        }
        else
        {
            _dx(0) = start_pos(0);
            _dx(1) = start_pos(1);
            _dx(2) = start_pos(2);
            _dx(3) = start_pos(3);
            _dx(4) = start_pos(4);
            _dx(5) = A * sin(f * M_PI * (_Time - MotionInitTime)) + start_pos(5);

            _dx(6) = start_pos(6);
            _dx(7) = start_pos(7);
            _dx(8) = start_pos(8);
            _dx(9) = start_pos(9);
            _dx(10) = start_pos(10);
            _dx(11) = start_pos(11);

            _dxdot(5) = (f * M_PI) * A * cos(f * M_PI * (_Time - MotionInitTime));
            _dxddot(5) = -(f * M_PI) * (f * M_PI) * A * sin(f * M_PI * (_Time - MotionInitTime));
        }
    }
    else if( MotionCommandTask == MOVE_TASK_CUSTOM4 )
    {
        if( _StatusWord == MotionCommandTask )
        {
            MotionInitTime = _Time;
            _StatusWord=0;
            start_pos.setZero(12);
            start_pos = x;
        }
        else
        {
            A = 0.07;
            _dx(0) = start_pos(0);
            _dx(1) = start_pos(1);
            _dx(2) = start_pos(2);
            _dx(3) = A * sin(f * M_PI * (_Time - MotionInitTime)) + start_pos(3);
            _dx(4) = start_pos(4);
            _dx(5) = start_pos(5);

            _dx(6) = start_pos(6);
            _dx(7) = start_pos(7);
            _dx(8) = start_pos(8);
            _dx(9) = -A * cos(2*f * M_PI * (_Time - MotionInitTime)) + start_pos(9);
            _dx(10) = -A * sin(2*f * M_PI * (_Time - MotionInitTime)) + start_pos(10);
            _dx(11) = start_pos(11);

            _dxdot(3) = (f * M_PI) * A * cos(f * M_PI * (_Time - MotionInitTime));
            _dxdot(9) = (2*f * M_PI) * A * sin(2*f * M_PI * (_Time - MotionInitTime));
            _dxdot(10) = -(2*f * M_PI) * A * cos(2*f * M_PI * (_Time - MotionInitTime));

            _dxddot(3) = -(f * M_PI) * (f * M_PI) * A * sin(f * M_PI * (_Time - MotionInitTime));
            _dxddot(9) = (2*f * M_PI) * (2*f * M_PI) * A * cos(2*f * M_PI * (_Time - MotionInitTime));
            _dxddot(10) = (2*f * M_PI) * (2*f * M_PI) * A * sin(2*f * M_PI * (_Time - MotionInitTime));

        }
    }
    else if( MotionCommandTask == MOVE_TASK_CUSTOM5 )
    {
        if( _StatusWord != MotionCommandTask )
        {
            if( NewTarget==1 )
            {
                int target_size = 6;
                _dx_tmp.setZero(target_size);
                _dxdot_tmp.setZero(target_size);
                _dxddot_tmp.setZero(target_size);

                _x_tmp.setZero(target_size);
                _xdot_tmp.setZero(target_size);

                _x_tmp.head(3) = x.segment(3,3);
                _x_tmp.tail(3) = x.segment(9,3);

                _xdot_tmp.head(3) = xdot.segment(3,3);
                _xdot_tmp.tail(3) = xdot.segment(9,3);

                TaskPoly5th.SetPoly5th(_Time, _x_tmp, _xdot_tmp, TargetPos_Linear, TrajectoryTime, target_size);
                TaskPoly5th.Poly5th(_Time, _dx_tmp, _dxdot_tmp, _dxddot_tmp);
                NewTarget=0;
            }
            else
            {
                TaskPoly5th.Poly5th(_Time, _dx_tmp, _dxdot_tmp, _dxddot_tmp);
            }

            _dx = TargetPosTask;
            _dx.segment(3,3) = _dx_tmp.head(3);
            _dx.segment(9,3) = _dx_tmp.tail(3);
            _dxdot.segment(3,3) = _dxdot_tmp.head(3);
            _dxdot.segment(9,3) = _dxdot_tmp.tail(3);
            _dxddot.segment(3,3) = _dxddot_tmp.head(3);
            _dxddot.segment(9,3) = _dxddot_tmp.tail(3);

            MotionProcess = MOVE_TASK_CUSTOM5;
        }
        else
        {
            TargetPosTask(0) = start_pos(0);
            TargetPosTask(1) = start_pos(1);
            TargetPosTask(2) = start_pos(2);

            TargetPosTask(3) = start_pos(3);
            TargetPosTask(4) = -0.41;
            TargetPosTask(5) = start_pos(5);


            TargetPosTask(6) = start_pos(6);
            TargetPosTask(7) = start_pos(7);
            TargetPosTask(8) = start_pos(8);

            TargetPosTask(9) = start_pos(9);
            TargetPosTask(10) = start_pos(10);
            TargetPosTask(11) = start_pos(11);
            TargetPosTask_p = TargetPosTask;

            TargetPos_Linear.setZero(6);
            TargetPos_Linear.head(3) = TargetPosTask.segment(3,3);
            TargetPos_Linear.tail(3) = TargetPosTask.segment(9,3);

            TrajectoryTime=6.0;
            NewTarget=1;
            _StatusWord = 0;
        }
    }
    else if( MotionCommandTask == MOVE_TASK_CUSTOM6 )
    {
        if( _StatusWord == MotionCommandTask )
        {
            MotionInitTime = _Time;
            _StatusWord=0;
            start_pos.setZero(12);
            start_pos = x;
        }
        else
        {
            _dx(0) = start_pos(0);
            _dx(1) = start_pos(1);
            _dx(2) = start_pos(2);
            _dx(3) = start_pos(3);
            _dx(4) = start_pos(4);
            _dx(5) = start_pos(5);

            _dx(6) = start_pos(6);
            _dx(7) = start_pos(7);
            _dx(8) = start_pos(8);
            _dx(9) = start_pos(9);
            _dx(10) = start_pos(10);
            _dx(11) = start_pos(11);
        }
    }
    else if( MotionCommandTask == MOVE_TASK_CUSTOM7 )
    {
        if( _StatusWord != MotionCommandTask )
        {
            if( NewTarget==1 )
            {
                int target_size = 6;
                _dx_tmp.setZero(target_size);
                _dxdot_tmp.setZero(target_size);
                _dxddot_tmp.setZero(target_size);

                _x_tmp.setZero(target_size);
                _xdot_tmp.setZero(target_size);

                _x_tmp.head(3) = x.segment(3,3);
                _x_tmp.tail(3) = x.segment(9,3);

                _xdot_tmp.head(3) = xdot.segment(3,3);
                _xdot_tmp.tail(3) = xdot.segment(9,3);

                TaskPoly5th.SetPoly5th(_Time, _x_tmp, _xdot_tmp, TargetPos_Linear, TrajectoryTime, target_size);
                TaskPoly5th.Poly5th(_Time, _dx_tmp, _dxdot_tmp, _dxddot_tmp);
                NewTarget=0;
            }
            else
            {
                TaskPoly5th.Poly5th(_Time, _dx_tmp, _dxdot_tmp, _dxddot_tmp);
            }

            _dx = TargetPosTask;
            _dx.segment(3,3) = _dx_tmp.head(3);
            _dx.segment(9,3) = _dx_tmp.tail(3);
            _dxdot.segment(3,3) = _dxdot_tmp.head(3);
            _dxdot.segment(9,3) = _dxdot_tmp.tail(3);
            _dxddot.segment(3,3) = _dxddot_tmp.head(3);
            _dxddot.segment(9,3) = _dxddot_tmp.tail(3);

            MotionProcess = MOVE_TASK_CUSTOM7;
        }
        else
        {
            TargetPosTask(0) = start_pos(0);
            TargetPosTask(1) = start_pos(1);
            TargetPosTask(2) = start_pos(2);

            TargetPosTask(3) = start_pos(3);
            TargetPosTask(4) = start_pos(4);
            TargetPosTask(5) = 0.36;


            TargetPosTask(6) = start_pos(6);
            TargetPosTask(7) = start_pos(7);
            TargetPosTask(8) = start_pos(8);

            TargetPosTask(9) = start_pos(9);
            TargetPosTask(10) = start_pos(10);
            TargetPosTask(11) = start_pos(11);
            TargetPosTask_p = TargetPosTask;

            TargetPos_Linear.setZero(6);
            TargetPos_Linear.head(3) = TargetPosTask.segment(3,3);
            TargetPos_Linear.tail(3) = TargetPosTask.segment(9,3);

            TrajectoryTime=6.0;
            NewTarget=1;
            _StatusWord = 0;
        }
    }
    else if( MotionCommandTask == MOVE_TASK_CUSTOM8 )
    {
        if( _StatusWord != MotionCommandTask )
        {
            if( NewTarget==1 )
            {
                int target_size = 6;
                _dx_tmp.setZero(target_size);
                _dxdot_tmp.setZero(target_size);
                _dxddot_tmp.setZero(target_size);

                _x_tmp.setZero(target_size);
                _xdot_tmp.setZero(target_size);

                _x_tmp.head(3) = x.segment(3,3);
                _x_tmp.tail(3) = x.segment(9,3);

                _xdot_tmp.head(3) = xdot.segment(3,3);
                _xdot_tmp.tail(3) = xdot.segment(9,3);

                TaskPoly5th.SetPoly5th(_Time, _x_tmp, _xdot_tmp, TargetPos_Linear, TrajectoryTime, target_size);
                TaskPoly5th.Poly5th(_Time, _dx_tmp, _dxdot_tmp, _dxddot_tmp);
                NewTarget=0;
            }
            else
            {
                TaskPoly5th.Poly5th(_Time, _dx_tmp, _dxdot_tmp, _dxddot_tmp);
            }

            _dx = TargetPosTask;
            _dx.segment(3,3) = _dx_tmp.head(3);
            _dx.segment(9,3) = _dx_tmp.tail(3);
            _dxdot.segment(3,3) = _dxdot_tmp.head(3);
            _dxdot.segment(9,3) = _dxdot_tmp.tail(3);
            _dxddot.segment(3,3) = _dxddot_tmp.head(3);
            _dxddot.segment(9,3) = _dxddot_tmp.tail(3);

            MotionProcess = MOVE_TASK_CUSTOM8;
        }
        else
        {
            TargetPosTask(0) = start_pos(0);
            TargetPosTask(1) = start_pos(1);
            TargetPosTask(2) = start_pos(2);

            TargetPosTask(3) = 0.520;
            TargetPosTask(4) = -0.35;
            TargetPosTask(5) = 0.530;


            TargetPosTask(6) = start_pos(6);
            TargetPosTask(7) = start_pos(7);
            TargetPosTask(8) = start_pos(8);

            TargetPosTask(9)  = start_pos(9);
            TargetPosTask(10) = 0.35;
            TargetPosTask(11) = start_pos(11);

            TargetPosTask_p = TargetPosTask;

            TargetPos_Linear.setZero(6);
            TargetPos_Linear.head(3) = TargetPosTask.segment(3,3);
            TargetPos_Linear.tail(3) = TargetPosTask.segment(9,3);

            TrajectoryTime=5.0;
            NewTarget=1;
            _StatusWord = 0;
        }
    }
    else if( MotionCommandTask == MOVE_TASK_CUSTOM9 )
    {
        if( _StatusWord != MotionCommandTask )
        {
            if( NewTarget==1 )
            {
                int target_size = 6;
                _dx_tmp.setZero(target_size);
                _dxdot_tmp.setZero(target_size);
                _dxddot_tmp.setZero(target_size);

                _x_tmp.setZero(target_size);
                _xdot_tmp.setZero(target_size);

                _x_tmp.head(3) = x.segment(3,3);
                _x_tmp.tail(3) = x.segment(9,3);

                _xdot_tmp.head(3) = xdot.segment(3,3);
                _xdot_tmp.tail(3) = xdot.segment(9,3);

                TaskPoly5th.SetPoly5th(_Time, _x_tmp, _xdot_tmp, TargetPos_Linear, TrajectoryTime, target_size);
                TaskPoly5th.Poly5th(_Time, _dx_tmp, _dxdot_tmp, _dxddot_tmp);
                NewTarget=0;
            }
            else
            {
                TaskPoly5th.Poly5th(_Time, _dx_tmp, _dxdot_tmp, _dxddot_tmp);
            }

            _dx = TargetPosTask;
            _dx.segment(3,3) = _dx_tmp.head(3);
            _dx.segment(9,3) = _dx_tmp.tail(3);
            _dxdot.segment(3,3) = _dxdot_tmp.head(3);
            _dxdot.segment(9,3) = _dxdot_tmp.tail(3);
            _dxddot.segment(3,3) = _dxddot_tmp.head(3);
            _dxddot.segment(9,3) = _dxddot_tmp.tail(3);

            MotionProcess = MOVE_TASK_CUSTOM9;
        }
        else
        {
            TargetPosTask(0) = start_pos(0);
            TargetPosTask(1) = start_pos(1);
            TargetPosTask(2) = start_pos(2);

            TargetPosTask(3) = 0.520;
            TargetPosTask(4) = -0.2;
            TargetPosTask(5) = 0.530;


            TargetPosTask(6) = start_pos(6);
            TargetPosTask(7) = start_pos(7);
            TargetPosTask(8) = start_pos(8);

            TargetPosTask(9)  = start_pos(9);
            TargetPosTask(10) = 0.2;
            TargetPosTask(11) = start_pos(11);
            TargetPosTask_p = TargetPosTask;

            TargetPos_Linear.setZero(6);
            TargetPos_Linear.head(3) = TargetPosTask.segment(3,3);
            TargetPos_Linear.tail(3) = TargetPosTask.segment(9,3);

            TrajectoryTime=10.0;
            NewTarget=1;
            _StatusWord = 0;
        }
    }
	MotionCommandTask_p = MotionCommandTask;
	return MotionProcess;
}

} /* namespace hyuCtrl */
