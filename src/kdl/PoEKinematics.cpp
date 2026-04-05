#include "kdl/PoEKinematics.h"

namespace HYUMotionKinematics {

    PoEKinematics::PoEKinematics():m_NumChain(1),m_DoF(6),WpInv_epsilon_left(0.001), WpInv_epsilon_right(0.001)
        this->mBodyJacobian.setZero(6*this->m_NumChain, this->m_DoF);
        this->mSpaceJacobian.setZero(6*this->m_NumChain, this->m_DoF);
        this->mAnalyticJacobian.setZero(6*this->m_NumChain, this->m_DoF);
        this->mIdentity_6N.setIdentity(6*this->m_NumChain, 6*this->m_NumChain);
        this->msvInv.setZero(this->m_DoF);
    }

    PoEKinematics::PoEKinematics( const MatrixXi &_ChainMat ):WpInv_epsilon_left(0.001), WpInv_epsilon_right(0.001)
    {
        ChainMatrix = _ChainMat;

        this->m_DoF = ChainMatrix.cols();
        this->m_NumChain = ChainMatrix.rows();

        int dofCounter;

        for(int i=0; i<this->m_NumChain; ++i)
        {

            ChainJointCount[i]=0;
            for(int j=(this->m_DoF-1); j>=0; --j)
            {
                if(ChainMatrix(i,j) == 1)
                {
                    if(ChainJointCount[i] == 0)
                    {
                        JointEndNum[i] = j+1;
                    }
                    ChainJointCount[i]++;
                }
            }
            this->Arr[i].setZero(ChainJointCount[i]);
            dofCounter = 0;
            for(int k=0; k < this->m_DoF; k++)
            {
                if(ChainMatrix(i,k) == 1)
                {
                    this->Arr[i](dofCounter) = k+1;
                    dofCounter++;
                }
            }
        }

        qLimit_High.setZero(m_DoF);
        qLimit_Low.setZero(m_DoF);

        for(int l=0; l<m_DoF; l++)
        {
            qLimit_Low(l) = joint_limit.Low[l];
            qLimit_High(l) = joint_limit.High[l];
        }

        this->mBodyJacobian.setZero(6*this->m_NumChain, this->m_DoF);
        this->mSpaceJacobian.setZero(6*this->m_NumChain, this->m_DoF);
        this->mAnalyticJacobian.setZero(6*this->m_NumChain, this->m_DoF);
        this->mIdentity_6N.setIdentity(6*this->m_NumChain, 6*this->m_NumChain);
        this->msvInv.setZero(this->m_DoF);
    }

    PoEKinematics::~PoEKinematics()
    {

    }

    void PoEKinematics::UpdateKinematicInfo( const Vector3d &_w, const Vector3d &_p, const Vector3d &_Rot, const Vector3d &_l, const int _link_num )
    {
        M[_link_num] = GetM(_Rot, _l);

        v_se3[_link_num] = GetTwist(_w, GetV(_w, _p));
    }

    Vector3d PoEKinematics::GetV( const Vector3d &_w, const Vector3d &_p )
    {
        return -SkewMatrix(_w)*_p;
    }

    SE3 PoEKinematics::GetM( const Vector3d &_Rot, const Vector3d &_link )
    {
        SE3_Tmp.setIdentity();
        Eigen::Quaternion<double> q;
        q = Eigen::AngleAxisd(_Rot(2), Vector3d::UnitZ())*Eigen::AngleAxisd(_Rot(1), Vector3d::UnitY())*Eigen::AngleAxisd(_Rot(0), Vector3d::UnitX());
        SE3_Tmp.block(0,0,3,3) = q.matrix();
        SE3_Tmp.block(0,3,3,1) = _link;
        return SE3_Tmp;
    }

    void PoEKinematics::SetTwist( const se3 &_Twist, const int _link_num )
    {
        v_se3[_link_num] = _Twist;
    }

    se3 PoEKinematics::GetTwist( const Vector3d &_w, const Vector3d &_v )
    {
        se3_Tmp.segment(0,3) = _w;
        se3_Tmp.segment(3,3) = _v;

        return se3_Tmp;
    }

    void PoEKinematics::HTransMatrix( const VectorXd &_q )
    {
        for (int end=0; end < m_DoF; end++)
        {
            Exp_S[end] = SE3Matrix(v_se3[end], _q(end));
        }

        for(int i=0; i < this->m_NumChain; ++i)
        {
            SE3_Tmp.setIdentity();

            for(int j=0; j < this->m_DoF; j++)
            {
                if(ChainMatrix(i,j) == 1)
                {
                    SE3_Tmp *= Exp_S[j];

                    T[0][j+1].setZero();
                    T[0][j+1].noalias() += SE3_Tmp*M[j];
                }
            }
        }
    }

    void PoEKinematics::PrepareJacobian( const VectorXd &_q )
    {
        HTransMatrix(_q);
        SpaceJacobian();
        SpaceToBodyJacobian();
        AnalyticJacobian();
        pInvJacobian();
    }

    void PoEKinematics::SpaceJacobian()
    {
        mSpaceJacobian.setZero();
        for(int i=0; i < this->m_NumChain; ++i)
        {
            SE3_Tmp.setIdentity();

            for(int j=0; j < this->m_DoF; ++j)
            {
                if(ChainMatrix(i,j) == 1)
                {
                    if(j == 0)
                    {
                        mSpaceJacobian.block(6*i, j, 6, 1) = v_se3[j];
                    }
                    else
                    {
                        AdjointMatrix(SE3_Tmp, adj_tmp);
                        mSpaceJacobian.block(6*i, j, 6, 1).noalias() += adj_tmp*v_se3[j];
                    }
                    SE3_Tmp*=Exp_S[j];
                }
            }
        }
    }

    void PoEKinematics::SpaceToBodyJacobian()
    {
        mBodyJacobian.setZero();
        for(int i=0; i < this->m_NumChain; i++)
        {
            inverse_SE3(T[0][JointEndNum[i]], SE3_Tmp2);
            AdjointMatrix(SE3_Tmp2, adj_tmp);
            mBodyJacobian.block(6*i,0,6,this->m_DoF).noalias() +=
                    adj_tmp*mSpaceJacobian.block(6*i,0,6,this->m_DoF);
        }
    }

    void PoEKinematics::BodyJacobianDot( const VectorXd &_qdot )
    {
        mBodyJacobianDot.setZero(6*m_NumChain, this->m_DoF);
        adjoint adTmp;
        int RowCount[2]={0,0};

        for(int j = 0; j < this->m_NumChain; j++)
        {
            for(int i=0; i<m_DoF; i++)
            {
                if(ChainMatrix(j,i) == 1)
                {
                    RowCount[j]++;
                    adjointMatrix(mBodyJacobian.col(i).segment(6*j,6), adTmp);
                    for(int k=RowCount[j]; k<ChainJointCount[j]; k++)
                    {
                        mBodyJacobianDot.col(i).segment(6*j,6).noalias() +=
                                adTmp*(mBodyJacobian.col(Arr[j](k)-1).segment(6*j,6)*_qdot(Arr[j](k)-1));
                    }
                }
            }
        }
    }

    void PoEKinematics::GetBodyJacobianDot(MatrixXd &_BodyJacobianDot)
    {
        _BodyJacobianDot = mBodyJacobianDot;
    }

    void PoEKinematics::AnalyticJacobian()
    {
        mAnalyticJacobian.setZero();
        int aJac_case=1;
        for(int i=0; i < this->m_NumChain; i++)
        {
            if(aJac_case == 0)
            {
                Mat_Tmp.setZero(6, 6);
                Mat_Tmp.block(0,0,3,3) = GetForwardKinematicsSO3(JointEndNum[i]);
                Mat_Tmp.block(3,3,3,3) = GetForwardKinematicsSO3(JointEndNum[i]);
                mAnalyticJacobian.block(6*i,0,6,m_DoF).noalias() += Mat_Tmp*mBodyJacobian.block(6*i,0,6,m_DoF);
            }
            else
            {
                mAnalyticJacobian.block(6*i, 0, 3, this->m_DoF) = mSpaceJacobian.block(6*i, 0, 3, this->m_DoF);
                mAnalyticJacobian.block(6*i+3, 0, 3, this->m_DoF).noalias() += T[0][JointEndNum[i]].block(0,0,3,3)*mBodyJacobian.block(6*i+3, 0, 3, this->m_DoF);
            }
        }
    }

    void PoEKinematics::AnalyticJacobianDot( const VectorXd &_qdot )
    {
        mAnalyticJacobianDot.setZero();
        BodyJacobianDot(_qdot);
        Matrix3d rot_tmp;
        Vector3d vec_tmp;

        for(int i=0; i<m_NumChain; i++)
        {
            rot_tmp.setZero();
            vec_tmp.setZero();
            for(int j=0; j<m_DoF; j++)
            {
                if(ChainMatrix(i,j) == 1)
                {
                    vec_tmp.noalias() += mSpaceJacobian.col(j).segment(6*i,3)*_qdot(j);
                }
            }
            SkewMatrix(vec_tmp, rot_tmp);
            mAnalyticJacobianDot.block(6*i, 0, 3, this->m_DoF).noalias() +=
                    rot_tmp*mAnalyticJacobian.block(6*i, 0, 3, this->m_DoF);
            mAnalyticJacobianDot.block(6*i, 0, 3, this->m_DoF).noalias() +=
                    GetForwardKinematicsSO3(JointEndNum[i])*mBodyJacobianDot.block(6*i, 0, 3, this->m_DoF);
            mAnalyticJacobianDot.block(6*i+3, 0, 3, this->m_DoF).noalias() +=
                    rot_tmp*mAnalyticJacobian.block(6*i+3, 0, 3, this->m_DoF);
            mAnalyticJacobianDot.block(6*i+3, 0, 3, this->m_DoF).noalias() +=
                    GetForwardKinematicsSO3(JointEndNum[i])*mBodyJacobianDot.block(6*i+3, 0, 3, this->m_DoF);
        }
    }

    void PoEKinematics::GetAnalyticJacobianDot( const VectorXd &_qdot, MatrixXd &_AnalyticJacobianDot )
    {
        AnalyticJacobianDot(_qdot);
        _AnalyticJacobianDot = mAnalyticJacobianDot;
    }

    void PoEKinematics::pInvJacobian()
    {
        mpInvJacobin.setZero(m_DoF, 6*m_NumChain);
        mpInvSVD.compute(mAnalyticJacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
        double tolerance = 1e-5;
        auto sv = mpInvSVD.singularValues();
        msvInv = sv;
        for (int i = 0; i < sv.size(); ++i) {
            if (sv(i) > tolerance) msvInv(i) = 1.0 / sv(i);
            else msvInv(i) = 0.0;
        }
        mpInvJacobin.noalias() = mpInvSVD.matrixV() * msvInv.asDiagonal() * mpInvSVD.matrixU().transpose();
    }

    void PoEKinematics::GetpinvJacobian( MatrixXd &_pinvJacobian )
    {
        _pinvJacobian = mpInvJacobin;
    }

    void PoEKinematics::ScaledTransJacobian()
    {
        ScaledFactor.setZero(m_DoF);
        for(int i=0; i<m_DoF; i++)
        {
            ScaledFactor(i) = 1.0/mAnalyticJacobian.col(i).squaredNorm();
        }

        mScaledTransJacobian.setZero(m_DoF,6*m_NumChain);
        mScaledTransJacobian.noalias() += ScaledFactor.asDiagonal()*mAnalyticJacobian.transpose();
    }

    void PoEKinematics::GetScaledTransJacobian( MatrixXd &_ScaledTransJacobian )
    {
        ScaledTransJacobian();
        _ScaledTransJacobian = mScaledTransJacobian;
    }

    void PoEKinematics::DampedpInvJacobian(const double sigma)
    {
        Mat_Tmp.setZero(6*m_NumChain, 6*m_NumChain);
        Mat_Tmp.noalias() += mAnalyticJacobian*mAnalyticJacobian.transpose();
        Mat_Tmp.noalias() += sigma*mIdentity_6N;

        mDampedpInvJacobian.setZero( m_DoF,6*m_NumChain );
        mDampedpInvJacobian.noalias() += mAnalyticJacobian.transpose()*Mat_Tmp.ldlt().solve(mIdentity_6N);
    }

    void PoEKinematics::GetDampedpInvJacobian( MatrixXd &_DampedpInvJacobian )
    {
        DampedpInvJacobian(0.0015);
        _DampedpInvJacobian = mDampedpInvJacobian;
    }

    void PoEKinematics::DampedpInvJacobian(MatrixXd &_TargetMatrix, const double sigma)
    {
        size_t r = _TargetMatrix.rows();
        Mat_Tmp.setZero(r, r);
        Mat_Tmp.noalias() += _TargetMatrix*_TargetMatrix.transpose();
        Mat_Tmp.noalias() += sigma*mIdentity_6N.block(0,0,r,r);

        mDampedpInvJacobian.setZero( m_DoF,6*m_NumChain );
        mDampedpInvJacobian.noalias() += _TargetMatrix.transpose()*Mat_Tmp.ldlt().solve(mIdentity_6N.block(0,0,r,r));
    }

    void PoEKinematics::GetDampedpInvJacobian(MatrixXd &_TargetMat, MatrixXd &_DampedpInvJacobian)
    {
        DampedpInvJacobian(_TargetMat, 0.0025);
        _DampedpInvJacobian = mDampedpInvJacobian;
    }

    void PoEKinematics::BlockpInvJacobian( Matrix<double, 6, Dynamic> &_Jacobian1, Matrix<double, 6, Dynamic> &_Jacobian2 )
    {
        mP1.setIdentity(16, 16);
        Mat_Tmp.noalias() = _Jacobian1*_Jacobian1.transpose();
        mP1.noalias() += -_Jacobian1.transpose()*Mat_Tmp.ldlt().solve(mIdentity_6N.block(0,0,6,6))*_Jacobian1;
        mP2.setIdentity(16, 16);
        Mat_Tmp.noalias() = _Jacobian2*_Jacobian2.transpose();
        mP2.noalias() += -_Jacobian2.transpose()*Mat_Tmp.ldlt().solve(mIdentity_6N.block(0,0,6,6))*_Jacobian2;

        mBlockpInvJacobian.setZero(m_DoF, 6*m_NumChain);
        mBlockpInvJacobian.block(0,0,16,6) = (_Jacobian1*mP2).completeOrthogonalDecomposition().pseudoInverse();
        mBlockpInvJacobian.block(0,6,16,6) = (_Jacobian2*mP1).completeOrthogonalDecomposition().pseudoInverse();
    }

    void PoEKinematics::GetBlockpInvJacobian(MatrixXd &_BlockpInvJacobian)
    {
        Matrix<double, 6, Dynamic> J1 = mAnalyticJacobian.block(0,0,6,16);
        Matrix<double, 6, Dynamic> J2 = mAnalyticJacobian.block(6,0,6,16);
        BlockpInvJacobian(J1, J2);
        _BlockpInvJacobian = mBlockpInvJacobian;
    }

    void PoEKinematics::WeightpInvJacobian( const VectorXd &_rdot, const MatrixXd &_WeightMat )
    {

        WpInv_epsilon_left = 0.0001;
        WpInv_epsilon_right = 0.00081;

        mWeightDampedpInvJacobian.setZero(16,12);

        VectorXd r1_right, r2_right, r1_left, r2_left;
        r2_right = _rdot.segment(0,3);  //rotation error for right-arm
        r1_right = _rdot.segment(3,3);  //translation error for right-arm
        r2_left = _rdot.segment(6,3);   //rotation error for left-arm
        r1_left = _rdot.segment(9,3);   //translation error for left-arm

        // analytic jacobian devided into left-arm jacobian & right-arm jacobian
        mJ_right1 = mAnalyticJacobian.block(0,0,6,16);
        mJ_left1 = mAnalyticJacobian.block(6,0,6,16);

        mP1 = MatrixXd::Identity(16, 16);
        Mat_Tmp.noalias() = mJ_right1*mJ_right1.transpose();
        mP1.noalias() += -mJ_right1.transpose()*Mat_Tmp.ldlt().solve(MatrixXd::Identity(6,6))*mJ_right1;
        mP2 = MatrixXd::Identity(16, 16);
        Mat_Tmp.noalias() = mJ_left1*mJ_left1.transpose();
        mP2.noalias() += -mJ_left1.transpose()*Mat_Tmp.ldlt().solve(MatrixXd::Identity(6,6))*mJ_left1;

        mJ_right = mJ_right1*mP2;
        mJ_left = mJ_left1*mP1;

        // 1st priority : Translation p 3x1, 2nd priority : Rotation r 3x1 for right-arm
        mJ1_right = mJ_right.block(3,0,3,16);
        mJ2_right = mJ_right.block(0,0,3,16);

        mW_right = MatrixXd::Zero(16, 16);
        mW_right.noalias() += mJ1_right.transpose()*mJ1_right;
        mW_right.noalias() += mJ2_right.transpose()*mJ2_right;
        mW_right.noalias() += WpInv_epsilon_right*_WeightMat;
        mW_inv_right = mW_right.ldlt().solve(MatrixXd::Identity(16, 16));

        mY_right = MatrixXd::Zero(3, 3);
        mY_right.noalias() += mJ1_right*mW_inv_right*mJ1_right.transpose();
        mY_inv_right = mY_right.ldlt().solve(MatrixXd::Identity(3, 3));

        mZ11_right = mW_inv_right;
        mZ11_right.noalias() += -mW_inv_right*(mJ1_right.transpose()*((mY_inv_right*mJ1_right)*mW_inv_right));
        mZ12_right = MatrixXd::Zero(16, 3);
        mZ12_right.noalias() += mW_inv_right*(mJ1_right.transpose()*mY_inv_right);
        mZ21_right = MatrixXd::Zero(3, 16);
        mZ21_right.noalias() += (mY_inv_right*mJ1_right)*mW_inv_right;
        mZ22_right = MatrixXd::Identity(3, 3);
        mZ22_right.noalias() += -mY_inv_right;

        mJ_WpInv_right = MatrixXd::Zero(16, 6);
        mJ_WpInv_right.block(0,0,16,3).noalias() += mZ11_right*mJ2_right.transpose();
        mJ_WpInv_right.block(0,3,16,3) = mZ12_right;
        lambda_right = -mZ21_right*mJ2_right.transpose()*r2_right - mZ22_right*r1_right;
        mWeightDampedpInvJacobian.block(0,0,16,6) = mJ_WpInv_right;

        // 1st priority : Translation p 3x1, 2nd priority : Rotation r 3x1 for left-arm
        mJ1_left = mJ_left.block(3,0,3,16);
        mJ2_left = mJ_left.block(0,0,3,16);

        mW_left = MatrixXd::Zero(16,16);
        mW_left.noalias() += mJ1_left.transpose()*mJ1_left;
        mW_left.noalias() += mJ2_left.transpose()*mJ2_left;
        mW_left.noalias() += WpInv_epsilon_left*_WeightMat;
        mW_inv_left = mW_left.ldlt().solve(MatrixXd::Identity(16, 16));

        mY_left = MatrixXd::Zero(3,3);
        mY_left.noalias() += mJ1_left*mW_inv_left*mJ1_left.transpose();
        mY_inv_left = mY_left.ldlt().solve(MatrixXd::Identity(3, 3));

        mZ11_left = mW_inv_left;
        mZ11_left.noalias() += -mW_inv_left*(mJ1_left.transpose()*((mY_inv_left*mJ1_left)*mW_inv_left));
        mZ12_left = MatrixXd::Zero(16,3);
        mZ12_left.noalias() += mW_inv_left*(mJ1_left.transpose()*mY_inv_left);
        mZ21_left = MatrixXd::Zero(3,16);
        mZ21_left.noalias() += (mY_inv_left*mJ1_left)*mW_inv_left;
        mZ22_left = MatrixXd::Identity(3,3);
        mZ22_left.noalias() += -mY_inv_left;

        mJ_WpInv_left = MatrixXd::Zero(16, 6);
        mJ_WpInv_left.block(0,0,16,3).noalias() += mZ11_left*mJ2_left.transpose();
        mJ_WpInv_left.block(0,3,16,3) = mZ12_left;
        lambda_left = -mZ21_left*mJ2_left.transpose()*r2_left - mZ22_left*r1_left;
        mWeightDampedpInvJacobian.block(0,6,16,6) = mJ_WpInv_left;
    }

    void PoEKinematics::GetWeightDampedpInvJacobian( const VectorXd &_rdot, const MatrixXd &_WeightMat, MatrixXd &_WDampedpInvJacobian )
    {
        WeightpInvJacobian(_rdot, _WeightMat);
        _WDampedpInvJacobian = mWeightDampedpInvJacobian;
    }

    void PoEKinematics::WeightpInvJacobian( const VectorXd &_rdot, const MatrixXd &_WeightMat, const MatrixXd &_TargetMat )
    {

        WpInv_epsilon_left = 0.001;
        WpInv_epsilon_right = 0.001;

        mWeightDampedpInvJacobian.setZero(16,12);

        VectorXd r1_right, r2_right, r1_left, r2_left;
        r2_right = _rdot.segment(0,3);  //rotation error for right-arm
        r1_right = _rdot.segment(3,3);  //translation error for right-arm
        r2_left = _rdot.segment(6,3);   //rotation error for left-arm
        r1_left = _rdot.segment(9,3);   //translation error for left-arm

        // analytic jacobian devided into left-arm jacobian & right-arm jacobian
        mJ_right1 = _TargetMat.block(0,0,6,16);
        mJ_left1 = _TargetMat.block(6,0,6,16);

        mP1 = MatrixXd::Identity(16, 16);
        Mat_Tmp.noalias() = mJ_right1*mJ_right1.transpose();
        mP1.noalias() += -mJ_right1.transpose()*Mat_Tmp.ldlt().solve(MatrixXd::Identity(6,6))*mJ_right1;
        mP2 = MatrixXd::Identity(16, 16);
        Mat_Tmp.noalias() = mJ_left1*mJ_left1.transpose();
        mP2.noalias() += -mJ_left1.transpose()*Mat_Tmp.ldlt().solve(MatrixXd::Identity(6,6))*mJ_left1;

        mJ_right = mJ_right1*mP2;
        mJ_left = mJ_left1*mP1;

        // 1st priority : Translation p 3x1, 2nd priority : Rotation r 3x1 for right-arm
        mJ1_right = mJ_right.block(3,0,3,16);
        mJ2_right = mJ_right.block(0,0,3,16);

        mW_right = MatrixXd::Zero(16, 16);
        mW_right.noalias() += mJ1_right.transpose()*mJ1_right;
        mW_right.noalias() += mJ2_right.transpose()*mJ2_right;
        mW_right.noalias() += WpInv_epsilon_right*_WeightMat;
        mW_inv_right = mW_right.ldlt().solve(MatrixXd::Identity(16, 16));

        mY_right = MatrixXd::Zero(3, 3);
        mY_right.noalias() += mJ1_right*mW_inv_right*mJ1_right.transpose();
        mY_inv_right = mY_right.ldlt().solve(MatrixXd::Identity(3, 3));

        mZ11_right = mW_inv_right;
        mZ11_right.noalias() += -mW_inv_right*(mJ1_right.transpose()*((mY_inv_right*mJ1_right)*mW_inv_right));
        mZ12_right = MatrixXd::Zero(16, 3);
        mZ12_right.noalias() += mW_inv_right*(mJ1_right.transpose()*mY_inv_right);
        mZ21_right = MatrixXd::Zero(3, 16);
        mZ21_right.noalias() += (mY_inv_right*mJ1_right)*mW_inv_right;
        mZ22_right = MatrixXd::Identity(3, 3);
        mZ22_right.noalias() += -mY_inv_right;

        mJ_WpInv_right = MatrixXd::Zero(16, 6);
        mJ_WpInv_right.block(0,0,16,3).noalias() += mZ11_right*mJ2_right.transpose();
        mJ_WpInv_right.block(0,3,16,3) = mZ12_right;
        lambda_right = -mZ21_right*mJ2_right.transpose()*r2_right - mZ22_right*r1_right;
        mWeightDampedpInvJacobian.block(0,0,16,6) = mJ_WpInv_right;

        // 1st priority : Translation p 3x1, 2nd priority : Rotation r 3x1 for left-arm
        mJ1_left = mJ_left.block(3,0,3,16);
        mJ2_left = mJ_left.block(0,0,3,16);

        mW_left = MatrixXd::Zero(16,16);
        mW_left.noalias() += mJ1_left.transpose()*mJ1_left;
        mW_left.noalias() += mJ2_left.transpose()*mJ2_left;
        mW_left.noalias() += WpInv_epsilon_left*_WeightMat;
        mW_inv_left = mW_left.ldlt().solve(MatrixXd::Identity(16, 16));

        mY_left = MatrixXd::Zero(3,3);
        mY_left.noalias() += mJ1_left*mW_inv_left*mJ1_left.transpose();
        mY_inv_left = mY_left.ldlt().solve(MatrixXd::Identity(3, 3));

        mZ11_left = mW_inv_left;
        mZ11_left.noalias() += -mW_inv_left*(mJ1_left.transpose()*((mY_inv_left*mJ1_left)*mW_inv_left));
        mZ12_left = MatrixXd::Zero(16,3);
        mZ12_left.noalias() += mW_inv_left*(mJ1_left.transpose()*mY_inv_left);
        mZ21_left = MatrixXd::Zero(3,16);
        mZ21_left.noalias() += (mY_inv_left*mJ1_left)*mW_inv_left;
        mZ22_left = MatrixXd::Identity(3,3);
        mZ22_left.noalias() += -mY_inv_left;

        mJ_WpInv_left = MatrixXd::Zero(16, 6);
        mJ_WpInv_left.block(0,0,16,3).noalias() += mZ11_left*mJ2_left.transpose();
        mJ_WpInv_left.block(0,3,16,3) = mZ12_left;
        lambda_left = -mZ21_left*mJ2_left.transpose()*r2_left - mZ22_left*r1_left;
        mWeightDampedpInvJacobian.block(0,6,16,6) = mJ_WpInv_left;
    }

    void PoEKinematics::GetWeightDampedpInvJacobian( const VectorXd &_rdot, const MatrixXd &_WeightMat, MatrixXd &_TargetMat, MatrixXd &_WDampedpInvJacobian )
    {
        WeightpInvJacobian(_rdot, _WeightMat, _TargetMat);
        _WDampedpInvJacobian = mWeightDampedpInvJacobian;
    }

    void PoEKinematics::GetWDampedpInvLambda(VectorXd *lambda)
    {
        lambda[0] = lambda_right;
        lambda[1] = lambda_left;
    }

    void PoEKinematics::RelativeJacobian( const int From, const int To )
    {
        Matrix<double, 6, 6> Body2Analyic = Eigen::Matrix<double, 6, 6>::Zero();
        Matrix<double, 6, 16> RelJacTmp = Eigen::Matrix<double, 6, 16>::Zero();
        mRelativeJacobian.setZero(6, m_DoF);
        if( From == 0 && To == 1 )
        {
            inverse_SE3(T[0][JointEndNum[To]], SE3_Tmp2);
            AdjointMatrix(SE3_Tmp2, adj_tmp);
            RelJacTmp.block(0, 2, 6, 7).noalias() += -adj_tmp*mSpaceJacobian.block(0, 2, 6, 7);
            RelJacTmp.block(0, 9, 6, 7) = mBodyJacobian.block(6, 9, 6, 7);
            Body2Analyic.block(3,3,3,3).noalias() += GetForwardKinematicsSO3(JointEndNum[From]).transpose()*GetForwardKinematicsSO3(JointEndNum[To]);
            Body2Analyic.block(0,0,3,3) = Body2Analyic.block(3,3,3,3);
        }
        mRelativeJacobian.noalias() += Body2Analyic*RelJacTmp;
    }

    void PoEKinematics::GetRelativeJacobian( MatrixXd &_RelativeJacobian )
    {
        RelativeJacobian(0,1); // right to left
        _RelativeJacobian = mRelativeJacobian;
    }

    void PoEKinematics::RelativeJacobianDot(const VectorXd &_qdot)
    {
        mRelativeBodyJacobianDot.setZero(6,m_DoF);
        mRelativeJacobianDot.setZero(6,m_DoF);
        adjoint adTmp;
        Adjoint AdTmp;
        inverse_SE3(T[0][JointEndNum[1]], SE3_Tmp2);
        AdjointMatrix(SE3_Tmp2, AdTmp);
        for(int i=3; i<=8; i++)
        {
            adTmp = adjointMatrix(mSpaceJacobian.col(i).segment(0,6));
            for(int k=2; k<i; k++)
            {
                mRelativeBodyJacobianDot.col(i).noalias() +=
                        -AdTmp*(adTmp*(mSpaceJacobian.col(k).segment(0,6)*_qdot(k)));
            }
        }
        for(int j=9; j<=14; j++)
        {
            adTmp = adjointMatrix(mBodyJacobian.col(j).segment(6,6));
            for(int l=j+1; l<=15; l++)
            {
                mRelativeBodyJacobianDot.col(j).noalias() += adTmp*(mBodyJacobian.col(l).segment(6,6)*_qdot(l));
            }
        }

        Matrix<double, 6, 6> Body2Analyic = Eigen::Matrix<double, 6, 6>::Zero();
        Body2Analyic.block(3,3,3,3).noalias() += GetForwardKinematicsSO3(JointEndNum[0]).transpose()*GetForwardKinematicsSO3(JointEndNum[1]);
        Body2Analyic.block(0,0,3,3) = Body2Analyic.block(3,3,3,3);

        mRelativeJacobianDot.noalias() += Body2Analyic*mRelativeBodyJacobianDot;

        Matrix<double, 6, 6> adMat;
        for(int n=2; n<16;n++)
        {
            adMat.setZero();
            if(n>=2 && n<=8)
            {
                adMat.block(0,0,3,3).noalias() += SkewMatrix(mBodyJacobian.col(n).segment(0,3)*_qdot(n));
                adMat.block(3,3,3,3) = adMat.block(0,0,3,3);
            }
            else
            {
                adMat.block(0,0,3,3).noalias() += SkewMatrix(mSpaceJacobian.col(n).segment(6,3)*_qdot(n));
                adMat.block(3,3,3,3) = adMat.block(0,0,3,3);
            }
            mRelativeJacobianDot.col(n).noalias() += adMat*mRelativeJacobian.col(n);
        }
    }

    void PoEKinematics::GetRelativeJacobianDot(const VectorXd &_qdot, MatrixXd &_RelativeJacobianDot)
    {
        _RelativeJacobianDot.setZero(6, m_DoF);
        RelativeJacobianDot(_qdot);
        _RelativeJacobianDot = mRelativeJacobianDot;
    }

    void PoEKinematics::GetInverseConditionNumber( double *_InverseCondNumber )
    {
        for(int i=0; i<this->m_NumChain; i++)
        {
            Mat_Tmp = this->mAnalyticJacobian.block(6*i,0, 6,this->m_DoF);
            auto jac_svd = Eigen::JacobiSVD<Eigen::Matrix<double, 6, Eigen::Dynamic>>{Mat_Tmp};
            _InverseCondNumber[i] = jac_svd.singularValues().minCoeff() / jac_svd.singularValues().maxCoeff();
        }
    }

    double PoEKinematics::GetManipulabilityMeasure()
    {
        return sqrt((mAnalyticJacobian*mAnalyticJacobian.transpose()).determinant());
    }

    double PoEKinematics::GetManipulabilityMeasure(const MatrixXd &_Jacobian)
    {
        return sqrt((_Jacobian*_Jacobian.transpose()).determinant());
    }

    void PoEKinematics::Getq0dotWithMM(const double &gain, VectorXd &q0dot)
    {
        q0dot.setZero(m_DoF);
        Matrix<double, 12, 16> dJbdq, dJadq;
        Matrix<double, 12, 12> MatTrace;

        auto ManipulabilityMeasure = GetManipulabilityMeasure();

        Matrix<double, 12,12> K, dKdq;
        K.setZero();
        K.block(0,0,3,3) = GetForwardKinematicsSO3(JointEndNum[0]);
        K.block(3,3,3,3) = GetForwardKinematicsSO3(JointEndNum[0]);
        K.block(6,6,3,3) = GetForwardKinematicsSO3(JointEndNum[1]);
        K.block(9,9,3,3) = GetForwardKinematicsSO3(JointEndNum[1]);

        MatrixXd pInvJacobian;
        GetpinvJacobian(pInvJacobian);

        int RowCount[2]={0,0};

        for(int i=1; i<m_DoF; i++)
        {
            dJbdq.setZero();
            dKdq.setZero();
            for(int j=0; j<m_NumChain; j++)
            {
                if(ChainMatrix(j,i) == 1)
                {
                    RowCount[j]++;
                    for(int k=0; k<RowCount[j]; k++)
                    {
                        dJbdq.col(Arr[j](k)-1).segment(6*j,6).noalias() +=
                                adjointMatrix(mBodyJacobian.col(Arr[j](k)-1).segment(6*j,6))*mBodyJacobian.col(i).segment(6*j,6);
                    }
                    dKdq.block(6*j,6*j,3,3).noalias() = SkewMatrix(mSpaceJacobian.col(i).segment(6*j,3));
                    dKdq.block(6*j+3,6*j+3,3,3) = dKdq.block(6*j,6*j,3,3) ;
                }
            }

            dJadq.setZero();
            dJadq.noalias() += dKdq*mAnalyticJacobian;
            dJadq.noalias() += K*dJbdq;

            MatTrace.setZero();
            MatTrace.noalias() += dJadq*pInvJacobian;

            q0dot(i) = gain*ManipulabilityMeasure*MatTrace.trace();
        }
    }

    void PoEKinematics::Getq0dotWithMM_Relative(const double &gain, const MatrixXd &_RelativeJacobian, VectorXd &q0dot)
    {
        q0dot.setZero(m_DoF);
        Matrix<double, 12, 16> dJbdq, dJadq;
        Matrix<double, 12, 12> MatTrace;

        adjoint adTmp;
        Adjoint AdTmp;
        AdTmp = AdjointMatrix(inverse_SE3(T[0][JointEndNum[1]]));

        auto ManipulabilityMeasure = GetManipulabilityMeasure(_RelativeJacobian);

        Matrix<double, 12,12> K, dKdq;
        K.setZero();
        K.block(0,0,3,3) = GetForwardKinematicsSO3(JointEndNum[0]);
        K.block(3,3,3,3) = GetForwardKinematicsSO3(JointEndNum[0]);
        K.block(6,6,3,3).noalias() += GetForwardKinematicsSO3(JointEndNum[0]).transpose()*GetForwardKinematicsSO3(JointEndNum[1]);
        K.block(9,9,3,3) = K.block(6,6,3,3);

        MatrixXd pInvJacobian;
        pInvJacobian = _RelativeJacobian.completeOrthogonalDecomposition().pseudoInverse();

        int RowCount[2]={0,0};

        for(int i=1; i<m_DoF; i++)
        {
            dJbdq.setZero();
            dKdq.setZero();

            if(ChainMatrix(0,i) == 1)
            {
                RowCount[0]++;
                for(int k=0; k<RowCount[0]; k++)
                {
                    dJbdq.col(Arr[0](k)-1).segment(0,6).noalias() +=
                            adjointMatrix(mBodyJacobian.col(Arr[0](k)-1).segment(0,6))*mBodyJacobian.col(i).segment(0,6);
                }
                dKdq.block(0,0,3,3).noalias() = SkewMatrix(mSpaceJacobian.col(i).segment(0,3));
                dKdq.block(3,3,3,3) = dKdq.block(0,0,3,3) ;
            }

            if( i >=2 && i <=7)
            {
                for(int j=i; j<8;j++)
                {
                    adTmp = adjointMatrix(mSpaceJacobian.col(j).segment(0,6));
                    dJbdq.col(j).segment(6,6).noalias() += -AdTmp*(adTmp*(mSpaceJacobian.col(i).segment(0,6)));
                }
                dKdq.block(6,6,3,3).noalias() += SkewMatrix(mBodyJacobian.col(i).segment(0,3));
                dKdq.block(9,9,3,3) = dKdq.block(6,6,3,3);
            }
            else if(i>=10 && i<=15)
            {
                for(int k=9;k<i; k++)
                {
                    adTmp = adjointMatrix(mBodyJacobian.col(k).segment(6,6));
                    dJbdq.col(k).segment(6,6).noalias() += adTmp*(mBodyJacobian.col(i).segment(6,6));
                }
                dKdq.block(6,6,3,3).noalias() += SkewMatrix(mSpaceJacobian.col(i).segment(6,3));
                dKdq.block(9,9,3,3) = dKdq.block(6,6,3,3);
            }

            dJadq.setZero();
            dJadq.noalias() += dKdq*_RelativeJacobian;
            dJadq.noalias() += K*dJbdq;

            MatTrace.setZero();
            MatTrace.noalias() += dJadq*pInvJacobian;

            q0dot(i) = gain*ManipulabilityMeasure*MatTrace.trace();
        }
    }

    void PoEKinematics::GetForwardKinematics( Vector3d *_Position, Vector3d *_Orientation, int &_NumChain )
    {
        _NumChain = this->m_NumChain;

        for(int i=0; i<this->m_NumChain; i++)
        {
            _Position[i].setZero();
            _Position[i] = T[0][JointEndNum[i]].block(0,3,3,1);
            SO3toRollPitchYaw(T[0][JointEndNum[i]].block(0,0,3,3), _Orientation[i]);
        }
    }

    void PoEKinematics::GetForwardKinematics(VectorXd &_x)
    {
        _x.setZero(12);
        Vector3d tmp_RPY;
        for(int i=0; i<this->m_NumChain; i++)
        {
            _x.segment(6*i+3, 3) = T[0][JointEndNum[i]].block(0,3,3,1);
            SO3toRollPitchYaw(T[0][JointEndNum[i]].block(0,0,3,3), tmp_RPY);
            _x.segment(6*i, 3) = tmp_RPY;
        }
    }

    void PoEKinematics::GetForwardKinematicsWithRelative(VectorXd &_x_rel)
    {
        _x_rel.setZero(12);
        Vector3d tmp_RPY;

        _x_rel.segment(3, 3) = T[0][JointEndNum[0]].block(0,3,3,1);
        SO3toRollPitchYaw(T[0][JointEndNum[0]].block(0,0,3,3), tmp_RPY);
        _x_rel.segment(0, 3) = tmp_RPY;

        SE3 HTRel;
        HTRel = inverse_SE3(T[0][JointEndNum[0]])*T[0][JointEndNum[1]];
        _x_rel.segment(9, 3) = HTRel.block(0,3,3,1);
        SO3toRollPitchYaw(HTRel.block(0,0,3,3), tmp_RPY);
        _x_rel.segment(6, 3) = tmp_RPY;

    }

    SE3 PoEKinematics::GetForwardKinematicsSE3( const int &_EndPosition ) const
    {
        return T[0][_EndPosition];
    }

    SO3 PoEKinematics::GetForwardKinematicsSO3(const int &_EndPosition) const
    {
        return T[0][_EndPosition].block(0, 0, 3, 3);
    }

    void PoEKinematics::GetAngleAxis( Vector3d *_Axis, double *_Angle, int &_NumChain )
    {
        _NumChain = this->m_NumChain;
        AngleAxis<double> rot;
        for(int i=0; i<this->m_NumChain; i++)
        {
            rot = GetForwardKinematicsSO3(JointEndNum[i]);
            _Axis[i] = rot.axis();
            _Angle[i] = rot.angle();
            //LogSO3(T[0][JointEndNum[i]].block(0,0,3,3), _Axis[i], _Angle[i]);
        }
    }

    void PoEKinematics::SO3toAngleAxis(const Matrix3d &_RotMat, Vector3d &_orientation)
    {
        AngleAxis<double> rot;
        rot = _RotMat;
        _orientation = rot.axis()*rot.angle();
    }

    void PoEKinematics::RollPitchYawtoSO3( const double &_Roll_rad, const double &_Pitch_rad, const double &_Yaw_rad, Matrix3d &_RotMat)
    {
        Eigen::Quaterniond q;
        q = Eigen::AngleAxisd(_Yaw_rad, Vector3d::UnitZ())*Eigen::AngleAxisd(_Pitch_rad, Vector3d::UnitY())*Eigen::AngleAxisd(_Roll_rad, Vector3d::UnitX());
        _RotMat = q.toRotationMatrix();
    }

    void PoEKinematics::SO3toRollPitchYaw( const Matrix3d &_RotMat, Vector3d &_Orientation )
    {
        q = _RotMat;

        _Orientation(0) = atan2(2.0*(q.x()*q.w()+q.y()*q.z()) , 1.0-2.0*(q.x()*q.x() + q.y()*q.y()) );

        double sinp = 2.0*( q.w()*q.y() - q.z()*q.x() );
        if(abs(sinp) >= 1)
            _Orientation(1) = copysign(M_PI_2, sinp);
        else
            _Orientation(1) = asin(sinp);

        _Orientation(2) = atan2( 2.0*(q.w()*q.z() + q.x()*q.y()) , 1.0-2.0*( q.y()*q.y() + q.z()*q.z() ) );
    }


}
