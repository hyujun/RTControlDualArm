/**
 * @file PoEKinematics.h
 * @brief Product of Exponential formulation for Kinematics
 * @date 2019-09-17
 * @author Junho Park
 */

#pragma once

#define _USE_MATH_DEFINES
#include <cmath>
#include "kdl/LieOperator.h"
#include "kdl/PropertyDefinition.h"

/**
 * @brief [Biorobotics Lab] Kinematics Solver using Lie-Group(Differential Kinematics)
 * @version 1.2.0
 */
namespace HYUMotionKinematics {

/**
 * @brief PoEKinematics Class for Tree-type Manipulator
 * @version 1.2.0
 */
    class PoEKinematics : public HYUMotionBase::LieOperator {
    // - [/] Phase 1: PoEKinematics Dynamic Memory Allocation & Inverse Optimizationr
    public:
        /**
         * @brief PoEKinematics class constructor
         * @details A chain matrix should be defined.
         */
        PoEKinematics();
        /**
         * @brief PoEKinematics class constructor
         * @details A chain matrix should be defined.
         */
        PoEKinematics( const MatrixXi &_ChainMat );
        virtual ~PoEKinematics();
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        /**
         * @brief Construct the kinematic infomation
         * @param[in] _w omega(twist)
         * @param[in] _p link position
         * @param[in] _l link length
         * @param[in] _link_num number of link attached to base-coordinate
         */
        void UpdateKinematicInfo( const Vector3d &_w, const Vector3d &_p, const Vector3d &_Rot, const Vector3d &_l, const int _link_num );

        /**
         * @brief Calculate the joint velocity v
         * @param[in] _w joint axis with respect to the base coordinate
         * @param[in] _p lint position attached to joint coordinate
         * @return v
         */
        Vector3d GetV( const Vector3d &_w, const Vector3d &_p );

        /**
         * @brief Calculate the initial configuration of serial robot
         * @param[in] _link total length of robot
         * @return SE(3)
         */
        SE3 GetM( const Vector3d &_Rot, const Vector3d &_link );

        void SetTwist( const se3 &_Twist, const int _link_num );
        /**
         * @brief Calculate the Twist of joint
         * @param[in] _w joint axis with respect to the base coordinate
         * @param[in] _v joint velocity
         * @return se3 vector
         */
        se3 GetTwist( const Vector3d &_w, const Vector3d &_v );

        /**
         * @brief Calculate the Homogeneous transformation matrix SE(3)
         * @param[in] _q generalized coordinate of joint position
         */
        void HTransMatrix( const VectorXd &_q );

        void PrepareJacobian( const VectorXd &_q );
        /**
         * @brief calculate the space jacobian
         * @return 6 x n(DoF) jacobian matrix w.r.t, base coordinate
         */
        [[nodiscard]] const MatrixXd& GetSpaceJacobian() const noexcept
        {
            return mSpaceJacobian;
        }

        /**
         * @brief calculate the body jacobian
         * @return 6 x n(DoF) jacobian matrix w.r.t., end-effector coordinate
         */
        [[nodiscard]] const MatrixXd& GetBodyJacobian() const noexcept
        {
            return mBodyJacobian;
        }

        void GetBodyJacobianDot( MatrixXd &_BodyJacobianDot );

        /**
         * @brief calcuate the analytic jacobian
         * @return 6 x n(DoF) jacobian matrix
         */
        // Legacy copy-out overload (used by many call sites)
        void GetAnalyticJacobian( MatrixXd &_AnalyticJacobian )
        {
            _AnalyticJacobian = mAnalyticJacobian;
        }
        // Zero-copy const reference overload (C++17, preferred)
        [[nodiscard]] const MatrixXd& GetAnalyticJacobian() const noexcept
        {
            return mAnalyticJacobian;
        }

        void GetAnalyticJacobianDot(const VectorXd &_qdot, MatrixXd &_AnalyticJacobianDot);

        void GetpinvJacobian( MatrixXd &_pinvJacobian );

        void GetScaledTransJacobian( MatrixXd &_ScaledTransJacobian );

        void GetDampedpInvJacobian( MatrixXd &_DampedpInvJacobian );

        void GetDampedpInvJacobian( MatrixXd &_TargetMat, MatrixXd &_DampedpInvJacobian );

        void GetBlockpInvJacobian( MatrixXd &_BlockpInvJacobian );

        void GetRelativeJacobian( MatrixXd &_RelativeJacobian );

        void GetRelativeJacobianDot( const VectorXd &_qdot, MatrixXd &_RelativeJacobianDot );

        void GetWeightDampedpInvJacobian( const VectorXd &_rdot, const MatrixXd &_WeightMat, MatrixXd &_WDampedpInvJacobian );

        void GetWeightDampedpInvJacobian( const VectorXd &_rdot, const MatrixXd &_WeightMat, MatrixXd &_TargetMat, MatrixXd &_WDampedpInvJacobian );

        void GetWDampedpInvLambda(VectorXd *lambda);

        void GetInverseConditionNumber( double *_InverseCondNumber );

        double GetManipulabilityMeasure();

        double GetManipulabilityMeasure(const MatrixXd &_Jacobian);

        void Getq0dotWithMM(const double &gain, VectorXd &q0dot);

        void Getq0dotWithMM_Relative(const double &gain, const MatrixXd &_RelativeJacobian, VectorXd &q0dot);
        /**
         * @brief forward kinematics of serial robot
         * @return end-effector position x, y, z. not orientation(Working)
         */
        void GetForwardKinematics( Vector3d *_Position, Vector3d *_Orientation, int &_NumChain );

        void GetForwardKinematics( VectorXd &_x );

        void GetForwardKinematicsWithRelative( VectorXd &_x_rel );

        SE3 GetForwardKinematicsSE3( const int &_EndPosition ) const;

        SO3 GetForwardKinematicsSO3( const int &_EndPosition ) const;

        void GetAngleAxis( Vector3d *_Axis, double *_Angle, int &_NumChain );

        void SO3toAngleAxis( const Matrix3d &_RotMat, Vector3d &_orientation );

        void SO3toRollPitchYaw( const Matrix3d &_RotMat, Vector3d &_Orientation );

        void RollPitchYawtoSO3( const double &_Roll_rad, const double &_Pitch_rad, const double &_Yaw_rad, Matrix3d &_RotMat);

        SE3 GetTMat(const int _begin, const int _end)
        {
            return T[_begin][_end];
        }

        int GetNumChain() const
        {
            return m_NumChain;
        }

        se3 GetTwist(const int _pos) const
        {
            return v_se3[_pos];
        }

        SE3 GetMMat(const int _pos) const
        {
            return M[_pos];
        }

        VectorXd qLimit_Low;
        VectorXd qLimit_High;

    protected:

        void SpaceJacobian();

        void SpaceToBodyJacobian();

        void BodyJacobianDot( const VectorXd &_qdot );

        void AnalyticJacobian();

        void AnalyticJacobianDot( const VectorXd &_qdot );

        void ScaledTransJacobian();

        void pInvJacobian();

        void DampedpInvJacobian(const double sigma);

        void DampedpInvJacobian( MatrixXd &_TargetMatrix, const double sigma);

        void RelativeJacobian(const int From, const int To);

        void RelativeJacobianDot( const VectorXd &_qdot );

        void BlockpInvJacobian( Matrix<double, 6, Dynamic> &_Jacobian1, Matrix<double, 6, Dynamic> &_Jacobian2 );

        void WeightpInvJacobian( const VectorXd &_rdot, const MatrixXd &_WeightMat );

        void WeightpInvJacobian( const VectorXd &_rdot, const MatrixXd &_WeightMat, const MatrixXd &_TargetMat );

        MatrixXi ChainMatrix;
        int m_NumChain;
        int m_DoF;
        int ChainJointCount[2];
        int JointEndNum[2];

        SE3 SE3_Tmp;
        SE3 SE3_Tmp2;
        se3 se3_Tmp;
        Adjoint adj_tmp;

        VectorXi Arr[2];

        MatrixXd mSpaceJacobian;
        MatrixXd mBodyJacobian;
        MatrixXd mBodyJacobianDot;
        MatrixXd mAnalyticJacobian;
        MatrixXd mAnalyticJacobianDot;
        VectorXd ScaledFactor;
        MatrixXd mScaledTransJacobian;
        MatrixXd mpInvJacobin;
        MatrixXd mDampedpInvJacobian;
        MatrixXd mBlockpInvJacobian;
        MatrixXd mWeightDampedpInvJacobian;
        Eigen::Matrix<double, 6, Dynamic> mRelativeJacobian;
        Eigen::Matrix<double, 6, Dynamic> mRelativeBodyJacobianDot;
        Eigen::Matrix<double, 6, Dynamic> mRelativeJacobianDot;

        MatrixXd Mat_Tmp;
        VectorXd Vec_Tmp;
        VectorXd msvInv;
        MatrixXd mIdentity_6N;

        // Pre-allocated variables for WeightpInvJacobian to avoid dynamic allocation
        MatrixXd mJ_right1, mJ_left1;
        MatrixXd mP1, mP2;
        MatrixXd mJ_left, mJ_right;
        MatrixXd mJ1_right, mJ2_right, mW_right, mW_inv_right, mY_right, mY_inv_right;
        MatrixXd mZ11_right, mZ12_right, mZ21_right, mZ22_right, mJ_WpInv_right;
        MatrixXd mJ1_left, mJ2_left, mW_left, mW_inv_left, mY_left, mY_inv_left;
        MatrixXd mZ11_left, mZ12_left, mZ21_left, mZ22_left, mJ_WpInv_left;

        // Pre-allocated JacobiSVD for pseudo-inverse
        Eigen::JacobiSVD<MatrixXd> mpInvSVD;

        Quaterniond q;

        double WpInv_epsilon_left;
        double WpInv_epsilon_right;
        VectorXd lambda_left;
        VectorXd lambda_right;

        /**
         * @brief SE(3) Homogeneous transform matrix container
         */
        SE3 T[17][17];
        //SE3 **T;

        /**
         * @brief SE(3) matrix container w.r.t., base coordinate
         */
        SE3 M[17];
        //SE3 *M;

        /**
         * @brief SE(3) matrix container
         */
        SE3 Exp_S[17];
        //SE3 *Exp_S;

        /**
         * @brief twist expression for Adjoint/adjoint matrix
         */
        se3 v_se3[17];
        //se3 *v_se3;

    };

}
