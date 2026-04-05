#include "kdl/LieDynamics.h"


// how to use
// 1. Update_info
// 2. Prepare_Dynamics
// 3. Get M_Matrix, C_Matrix, G_Matrix

namespace HYUMotionDynamics{

    Liedynamics::Liedynamics():isFirstRun(0)
    {
        m_NumChain = 1;
        m_DoF = 6;

        this->Iner_mat.setZero(6*this->m_DoF, 6*this->m_DoF);
        this->Gamma_mat.setZero(6*this->m_DoF, 6*this->m_DoF);
        this->L_mat.setZero(6*this->m_DoF, 6*this->m_DoF);
        this->ad_Aqd.setZero(6*this->m_DoF, 6*this->m_DoF);
        this->ad_V.setZero(6*this->m_DoF, 6*this->m_DoF);

        this->A_mat.setZero(6*this->m_DoF, this->m_DoF);
        this->LA_mat.setZero(6*this->m_DoF, this->m_DoF);

        this->V.setZero(6*this->m_DoF);
        this->VdotBase.setZero(6*this->m_DoF);

        this->mC_Temp1.setZero(6*this->m_DoF, 6*this->m_DoF);
        this->mC_Temp2.setZero(6*this->m_DoF, 6*this->m_DoF);
        this->mC_Temp3.setZero(6*this->m_DoF, 6*this->m_DoF);
        this->mC_Temp4.setZero(6*this->m_DoF, 6*this->m_DoF);
        this->mC_Temp5.setZero(6*this->m_DoF, this->m_DoF);
        this->mC_VecTemp1.setZero(6*this->m_DoF);
        this->mC_VecTemp2.setZero(6*this->m_DoF);
        this->mC_VecTemp3.setZero(6*this->m_DoF);
        this->mTask_M.setZero(this->m_DoF, this->m_DoF);
        this->mTask_G.setZero(this->m_DoF);

        grav.setZero(6);
        grav << 0, 0, 0, 0, 0, 9.8;
    }

    Liedynamics::Liedynamics( const MatrixXi &_ChainMatrix ):isFirstRun(0)
    {

        ChainMatrix = _ChainMatrix;

        m_NumChain = ChainMatrix.rows();
        m_DoF = ChainMatrix.cols();

        for(int i=0; i < ChainMatrix.rows(); ++i)
        {
            ChainJointCount[i]=0;
            for(int j=(ChainMatrix.cols()-1); j>=0; --j)
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
            Arr[i].resize(ChainJointCount[i]);
        }

        Iner_mat.setZero(6*this->m_DoF, 6*this->m_DoF);
        Gamma_mat.setZero(6*this->m_DoF, 6*this->m_DoF);
        L_mat.setZero(6*this->m_DoF, 6*this->m_DoF);
        LA_mat.setZero(6*this->m_DoF, this->m_DoF);
        ad_Aqd.setZero(6*this->m_DoF, 6*this->m_DoF);
        ad_V.setZero(6*this->m_DoF, 6*this->m_DoF);

        V.setZero(6*this->m_DoF);
        VdotBase.setZero(6*this->m_DoF);

        mC_Temp1.setZero(6*this->m_DoF, 6*this->m_DoF);
        mC_Temp2.setZero(6*this->m_DoF, 6*this->m_DoF);
        mC_Temp3.setZero(6*this->m_DoF, 6*this->m_DoF);
        mC_Temp4.setZero(6*this->m_DoF, 6*this->m_DoF);
        mC_Temp5.setZero(6*this->m_DoF, this->m_DoF);
        mC_VecTemp1.setZero(6*this->m_DoF);
        mC_VecTemp2.setZero(6*this->m_DoF);
        mC_VecTemp3.setZero(6*this->m_DoF);
        mTask_M.setZero(this->m_DoF, this->m_DoF);
        mTask_G.setZero(this->m_DoF);

        grav.setZero(6);
        grav << 0, 0, 0, 0, 0, 9.8;
    }

    Liedynamics::~Liedynamics()
    = default;

    void Liedynamics::UpdateDynamicInfo( const Vector3d &_w, const Vector3d &_p, const Matrix3d &_Inertia, const double &_Mass, const Vector3d &_CoM, const int _LinkNum )
    {
        Vector3d zero = Vector3d::Zero();
        GeneralizedInertia(_Inertia, _Mass, GIner[_LinkNum]);
        UpdateKinematicInfo(_w, _p, zero, _CoM, _LinkNum);
        A[_LinkNum] = AdjointMatrix(inverse_SE3(GetMMat(_LinkNum)))*GetTwist(_LinkNum);
        SetTwist(A[_LinkNum], _LinkNum);
    }

    void Liedynamics::GeneralizedInertia(const Matrix3d &_Inertia, const double &_Mass, Matrix6d &GIner)
    {
        GIner.setZero();
        GIner.block<3, 3>(0, 0) = _Inertia; /**If the Coordinate is changed, then THE INERTIA HAVE TO CHANGE w.r.t. the coordinate*/
        GIner.block<3, 3>(3, 3).noalias() += _Mass*Matrix3d::Identity();
    }

    void Liedynamics::Inertia_Link( void )
    {
        this->Iner_mat.setZero();
        for (int i = 0; i < this->m_DoF; ++i)
        {
            this->Iner_mat.block<6, 6>(6 * i, 6 * i).noalias() += this->GIner[i];
        }
        return;
    }

    void Liedynamics::Gamma_Link( void )
    {
        this->Gamma_mat.setZero();
        for (int i = 1; i < this->m_DoF; ++i)
        {
            LieOperator::AdjointMatrix(GetTMat(i+1, i), adj_tmp);
            this->Gamma_mat.block<6, 6>(6 * i, 6 * (i - 1)).noalias() += adj_tmp;
        }
        return;
    }

    void Liedynamics::DynHTransMatrix(const VectorXd &_q)
    {
        int TCounter;

        for(int i=0; i < this->m_NumChain; ++i)
        {
            TCounter=0;
            Arr[i].setZero();

            for(int j=0; j < this->m_DoF; j++)
            {
                if(ChainMatrix(i,j) == 1)
                {
                    Arr[i](TCounter) = j+1;
                    if(j==0)
                    {
                        T[Arr[i](TCounter)][0].setZero();
                        T[Arr[i](TCounter)][0].noalias() += inverse_SE3(SE3Matrix(v_se3[j], _q(j)))*inverse_SE3(M[j]);
                    }
                    else
                    {
                        T[Arr[i](TCounter)][Arr[i](TCounter-1)].setZero();
                        T[Arr[i](TCounter)][Arr[i](TCounter-1)].noalias() +=
                                inverse_SE3(SE3Matrix(v_se3[j], _q(j)))*inverse_SE3(M[j])*M[Arr[i](TCounter-1)-1];
                    }
                    TCounter++;
                }
            }

            for(int k=0; k<TCounter; k++)
            {
                for(int l=(k+2); l<TCounter; l++)
                {
                    T[Arr[i](l)][Arr[i](k)].setZero();
                    T[Arr[i](l)][Arr[i](k)].noalias() += T[Arr[i](l)][Arr[i](l-1)]*T[Arr[i](l-1)][Arr[i](k)];
                }
            }
        }
    }

    void Liedynamics::L_link( )
    {
        L_mat.setIdentity();

        for(int k=0; k < this->m_NumChain; k++)
        {
            for (int i = 0; i < this->m_DoF; ++i)
            {
                if(ChainMatrix(k,i) == 1)
                {
                    for(int j=i+1; j< this->m_DoF; ++j)
                    {
                        if(ChainMatrix(k,j) == 1)
                        {
                            if(!(k>0 && ChainMatrix(k-1,j) == 1))
                            {
                                LieOperator::AdjointMatrix(GetTMat(j+1,i+1), adj_tmp);
                                L_mat.block(6*j, 6*i, 6, 6).noalias() += adj_tmp;
                            }
                        }
                    }
                }
            }
        }
    }

    void Liedynamics::A_Link( void )
    {
        A_mat.setZero();

        for (int i = 0; i < this->m_DoF; ++i)
        {
            this->A_mat.block<6, 1>(6 * i,  i) = A[i];
        }
        return;
    }

    void Liedynamics::LA_Link()
    {
        LA_mat.setZero(6*m_DoF, m_DoF);
        for(int i=0; i<m_DoF; i++)
        {
            for(int j=i; j<m_DoF; j++)
            {
                LA_mat.block(6*j, i, 6, 1).noalias() += L_mat.block(6*j, 6*i, 6, 6)*A[i];
            }
        }
    }

    void Liedynamics::ad_Aqdot_Link( const VectorXd &_qdot )
    {
        this->ad_Aqd.setZero();
        for (int i = 0; i < this->m_DoF; ++i)
        {
            this->ad_Aqd.block<6, 6>(6 * i, 6 * i).noalias() += LieOperator::adjointMatrix(A[i]*_qdot(i));
        }
        return;
    }

    void Liedynamics::ad_V_Link( const VectorXd &_qdot )
    {
        V.setZero();
        V.noalias() += LA_mat*_qdot;

        this->ad_V.setZero();

        for (int i = 0; i < this->m_DoF; ++i)
        {
            this->ad_V.block<6, 6>(6 * i, 6 * i).noalias() += LieOperator::adjointMatrix(V.segment(6 * i, 6));
        }

        return;
    }

    void Liedynamics::Vdot_base( void )
    {
        VdotBase.setZero();
        VdotBase.segment(0, 6).noalias() += LieOperator::AdjointMatrix(GetTMat(1, 0))*grav;
        return;
    }

    void Liedynamics::PrepareDynamics( const VectorXd &_q, const VectorXd &_qdot )
    {
        if(isFirstRun == 0)
        {
            Inertia_Link();
            A_Link();
            isFirstRun = 1;
        }

        DynHTransMatrix(_q);

        L_link();
        Vdot_base();
        LA_Link();

        //Gamma_Link();
        //ad_Aqdot_Link(_qdot);
        //ad_V_Link(_qdot);
    }

    void Liedynamics::M_Matrix( MatrixXd &_M )
    {
        _M.setZero(m_DoF, m_DoF);
        mM_Tmp.setZero(6*m_DoF, m_DoF); // Re-use member variable
        
        for(int k=0; k<m_DoF; k++)
        {
            mM_Tmp.block(6*k, 0, 6, m_DoF).noalias() = GIner[k] * LA_mat.block(6*k, 0, 6, m_DoF);
        }
        
        _M.triangularView<Eigen::Upper>() = LA_mat.transpose() * mM_Tmp;
        _M.triangularView<Eigen::StrictlyLower>() = _M.transpose();
    }

    void Liedynamics::C_Matrix( MatrixXd &_C )
    {
        _C.setZero(this->m_DoF, this->m_DoF);
        mC_Temp1.noalias() = Iner_mat * L_mat;
        mC_Temp2.noalias() = mC_Temp1 * ad_Aqd;
        mC_Temp3.noalias() = mC_Temp2 * Gamma_mat;
        mC_Temp4.noalias() = ad_V.transpose() * Iner_mat;
        mC_Temp3.noalias() -= mC_Temp4;
        mC_Temp5.noalias() = mC_Temp3 * LA_mat;
        _C.noalias() = -LA_mat.transpose() * mC_Temp5;
    }

    void Liedynamics::C_Vector( VectorXd &_C, const VectorXd &_qdot )
    {
        _C.setZero(this->m_DoF);
        mC_VecTemp1.noalias() = LA_mat * _qdot;
        mC_VecTemp2.noalias() = Gamma_mat * mC_VecTemp1;
        mC_VecTemp3.noalias() = ad_Aqd * mC_VecTemp2;
        mC_VecTemp2.noalias() = L_mat * mC_VecTemp3;
        mC_VecTemp3.noalias() = Iner_mat * mC_VecTemp2;

        mC_VecTemp2.noalias() = Iner_mat * mC_VecTemp1;
        mC_VecTemp1.noalias() = ad_V.transpose() * mC_VecTemp2;

        mC_VecTemp2.noalias() = mC_VecTemp3 - mC_VecTemp1;
        _C.noalias() += -LA_mat.transpose() * mC_VecTemp2;
    }

    void Liedynamics::G_Matrix( VectorXd &_G )
    {
        _G.setZero();
        mC_VecTemp1.noalias() = L_mat * VdotBase;
        mC_VecTemp2.noalias() = Iner_mat * mC_VecTemp1;
        _G.noalias() += LA_mat.transpose() * mC_VecTemp2;
    }

    void Liedynamics::MG_Mat_Joint( MatrixXd &_M, VectorXd&_G )
    {
        M_Matrix(_M);
        G_Matrix(_G);
    }

    void Liedynamics::M_Mat_Task(MatrixXd &_Mx, MatrixXd &_pInv)
    {
        M_Matrix(this->mTask_M);
        _Mx.setZero(6*this->m_NumChain, 6*this->m_NumChain);
        _Mx.noalias() += _pInv.transpose() * this->mTask_M * _pInv;
    }

    void Liedynamics::MG_Mat_Task(MatrixXd &_Mx, VectorXd &_Gx)
    {
        M_Matrix(this->mTask_M);
        G_Matrix(this->mTask_G);
        GetAnalyticJacobian(this->Jacobian_mat);
        GetpinvJacobian(this->pinvJacobian_mat);

        _Mx.setZero(6*this->m_NumChain, 6*this->m_NumChain);
        _Mx.noalias() += pinvJacobian_mat.transpose() * this->mTask_M * pinvJacobian_mat;

        _Gx.setZero(6*this->m_NumChain);
        _Gx.noalias() += pinvJacobian_mat.transpose() * this->mTask_G;
    }

    void Liedynamics::Mdot_Matrix( MatrixXd &_Mdot )
    {
        _Mdot.setZero(m_DoF, m_DoF);
        mC_Temp5.noalias() = Iner_mat * LA_mat;
        mC_Temp1.noalias() = L_mat.transpose() * mC_Temp5;
        mC_Temp2.noalias() = ad_Aqd.transpose() * mC_Temp1;
        mC_Temp3.noalias() = Gamma_mat.transpose() * mC_Temp2;
        _Mdot.noalias() += -LA_mat.transpose() * mC_Temp3;

        mC_Temp1.noalias() = Gamma_mat * LA_mat;
        mC_Temp2.noalias() = ad_Aqd * mC_Temp1;
        mC_Temp3.noalias() = L_mat * mC_Temp2;
        mC_Temp4.noalias() = Iner_mat * mC_Temp3;
        _Mdot.noalias() += -LA_mat.transpose() * mC_Temp4;
    }

}
