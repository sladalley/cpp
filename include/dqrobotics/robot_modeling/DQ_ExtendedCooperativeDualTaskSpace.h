#ifndef DQ_EXTENDEDCOOPERATIVEDUALTASKSPACE_H
#define DQ_EXTENDEDCOOPERATIVEDUALTASKSPACE_H

#include<dqrobotics/DQ.h>
#include<dqrobotics/robot_modeling/DQ_Kinematics.h>
#include<dqrobotics/robot_modeling/DQ_SerialManipulatorDH.h>
#include<dqrobotics/robot_modeling/DQ_CooperativeDualTaskSpace.h>

namespace DQ_robotics
{

class DQ_ExtendedCooperativeDualTaskSpace: public DQ_CooperativeDualTaskSpace
{
private:
    const std::shared_ptr<DQ_Kinematics>& robot1_;
    const std::shared_ptr<DQ_Kinematics>& robot2_;


protected:
    double alpha_;
    bool beta_;
    bool alphamode_;

public:
    //Remove default constructor

    DQ_ExtendedCooperativeDualTaskSpace()=delete;
    DQ_ExtendedCooperativeDualTaskSpace(const std::shared_ptr<DQ_Kinematics> &robot1,const std::shared_ptr<DQ_Kinematics> &robot2);
    DQ_ExtendedCooperativeDualTaskSpace(const std::shared_ptr<DQ_Kinematics> &robot1,const std::shared_ptr<DQ_Kinematics> &robot2, const bool& beta, const double& alpha);

    void setAlpha(const double& alpha);
    double getAlpha();
    void setAlphaMode(const bool& alphamode);

    void setBeta(const bool& beta);
    bool getBeta();

    int get_configuration_space() override;

    DQ pose1(const VectorXd& theta) override;
    DQ pose2(const VectorXd& theta) override;

    MatrixXd pose_jacobian1(const VectorXd& theta) override;
    MatrixXd pose_jacobian2(const VectorXd& theta) override;


    DQ relative_pose(const VectorXd& theta) override;
    DQ absolute_pose(const VectorXd& theta) override;


    DQ relative_twist(const DQ twist_0_0_1, const DQ twist_0_0_2);
    DQ absolute_twist(const DQ twist_1_0_1, const DQ twist_2_0_2, const VectorXd &theta);

    MatrixXd relative_pose_jacobian(const VectorXd& theta) override;
    MatrixXd absolute_pose_jacobian(const VectorXd& theta) override;

    MatrixXd absolute_alpha_pose_jacobian(const VectorXd& theta);

};


}

#endif
