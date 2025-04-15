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

public:
    //Remove default constructor

    DQ_ExtendedCooperativeDualTaskSpace()=delete;
    DQ_ExtendedCooperativeDualTaskSpace(const std::shared_ptr<DQ_Kinematics> &robot1,const std::shared_ptr<DQ_Kinematics> &robot2);
    DQ_ExtendedCooperativeDualTaskSpace(const std::shared_ptr<DQ_Kinematics> &robot1,const std::shared_ptr<DQ_Kinematics> &robot2, const bool& beta, const double& alpha);

    void setAlpha(const double& alpha);
    double getAlpha();

    void setBeta(const bool& beta);
    bool getBeta();


    DQ relative_pose(const VectorXd& theta) override;
    DQ absolute_pose(const VectorXd& theta) override;

    MatrixXd relative_pose_jacobian(const VectorXd& theta) override;
    MatrixXd absolute_pose_jacobian(const VectorXd& theta) override;

};


}

#endif
