#include<dqrobotics/robot_modeling/DQ_CooperativeDualTaskSpace.h>
#include<dqrobotics/robot_modeling/DQ_ExtendedCooperativeDualTaskSpace.h>

namespace DQ_robotics
{

DQ_ExtendedCooperativeDualTaskSpace::DQ_ExtendedCooperativeDualTaskSpace(const std::shared_ptr<DQ_Kinematics> &robot1, const std::shared_ptr<DQ_Kinematics> &robot2 )
    : DQ_CooperativeDualTaskSpace(robot1, robot2),
    robot1_(robot1),
    robot2_(robot2),
    alpha_(0.5),
    beta_(true)
{

}

DQ_ExtendedCooperativeDualTaskSpace::DQ_ExtendedCooperativeDualTaskSpace(const std::shared_ptr<DQ_Kinematics> &robot1, const std::shared_ptr<DQ_Kinematics> &robot2, const bool &beta, const double &alpha )
: DQ_CooperativeDualTaskSpace(robot1, robot2),
  robot1_(robot1),
  robot2_(robot2),
  alpha_(alpha),
  beta_(beta)
{

}

void DQ_ExtendedCooperativeDualTaskSpace::setAlpha (const double &alpha) {
    alpha_ = alpha;
}

double DQ_ExtendedCooperativeDualTaskSpace::getAlpha ()  {
    return alpha_;
}

void DQ_ExtendedCooperativeDualTaskSpace::setBeta (const bool &beta) {
    beta_ = beta;
}

bool DQ_ExtendedCooperativeDualTaskSpace::getBeta () {
    return beta_;
}

DQ DQ_ExtendedCooperativeDualTaskSpace::relative_pose(const VectorXd &theta)
{
    return pow(conj(pose2(theta)),beta_)*pose1(theta);
}

DQ DQ_ExtendedCooperativeDualTaskSpace::absolute_pose(const VectorXd &theta)
{
    return pose2(theta)*(pow(relative_pose(theta),(alpha_*beta_)));
}
/**
 * @brief DQ_ExtendedCooperativeDualTaskSpace::_relative_twist given in respect to inertial frame
 * @param twist_0_0_1: given in respect to inertial frame
 * @param twist_0_0_2: given in respect to inertial frame
 * @return
 */
DQ DQ_ExtendedCooperativeDualTaskSpace::relative_twist(const DQ twist_0_0_1, const DQ twist_0_0_2)
{
    return twist_0_0_1-twist_0_0_2;
}
/**
* @brief DQ_ExtendedCooperativeDualTaskSpace::absolute_twist given in respect to absolute frame
* @param twist_1: given in respect to body frame
* @param twist_2: given in respect to body frame
* @return
*/
DQ DQ_ExtendedCooperativeDualTaskSpace::absolute_twist(const DQ twist_1_0_1, const DQ twist_2_0_2, const VectorXd &theta)
{
    const DQ r_0_a = rotation(absolute_pose(theta));
    const DQ r_a_2 = conj(r_0_a)*rotation(pose2(theta));
    const DQ r_a_1 = conj(r_0_a)*rotation(pose1(theta));
    const DQ absolute_twist = (1-alpha_)*Ad(r_a_2,twist_2_0_2) + alpha_*Ad(r_a_1,twist_1_0_1);

    return absolute_twist;
}

MatrixXd DQ_ExtendedCooperativeDualTaskSpace::relative_pose_jacobian(const VectorXd &theta)
{
    const MatrixXd Jx1 = pose_jacobian1(theta); 
    const DQ       x1  = pose1(theta);
    const MatrixXd Jx2 = pose_jacobian2(theta);
    const DQ       x2  = pose2(theta);
    MatrixXd Jxr(8,_get_robot1_ptr()->get_dim_configuration_space()+_get_robot2_ptr()->get_dim_configuration_space());
    if (beta_ == false) {
        Jxr << Jx1,MatrixXd::Zero(8,_get_robot2_ptr()->get_dim_configuration_space());
    } else {
        Jxr << (hamiplus8(conj(x2)))*Jx1,haminus8(x1)*C8()*Jx2;
    }
    return  Jxr;
}

MatrixXd DQ_ExtendedCooperativeDualTaskSpace::absolute_pose_jacobian(const VectorXd &theta)
{
    //Preliminaries
    const MatrixXd Jx2 = pose_jacobian2(theta);
    const DQ       x2  = pose2(theta);
    const MatrixXd Jxr  = relative_pose_jacobian(theta);
    const DQ       xr  = relative_pose(theta);
    const MatrixXd Jtr = DQ_Kinematics::translation_jacobian(Jxr,xr);
    MatrixXd Jxa(8,_get_robot1_ptr()->get_dim_configuration_space()+_get_robot2_ptr()->get_dim_configuration_space());


    //Rotation part
    
    MatrixXd Jpxr = alpha_*haminus4(pow(xr.P(),(alpha_-1)))*Jxr.block(0,0,4,Jxr.cols());
    MatrixXd Jdxr = 0.5*alpha_*(haminus4(pow(xr.P(),alpha_))*Jtr +hamiplus4(translation(xr))*Jpxr);

    MatrixXd Jxr2(8,Jpxr.cols());
    Jxr2 << Jpxr, Jdxr;

    MatrixXd temp(8,_get_robot1_ptr()->get_dim_configuration_space()+_get_robot2_ptr()->get_dim_configuration_space());
    temp << MatrixXd::Zero(8,_get_robot1_ptr()->get_dim_configuration_space()),Jx2;

    if (beta_ == false) {
        Jxa << temp;
    } else {
        Jxa << haminus8(pow(xr, alpha_))*temp +hamiplus8(x2)*Jxr2;
    }

    return Jxa;
}

}
