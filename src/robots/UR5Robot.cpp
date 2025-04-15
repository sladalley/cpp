#include<dqrobotics/utils/DQ_Constants.h>
#include "UR5Robot.h"

namespace DQ_robotics
{


DQ_SerialManipulatorDH UR5Robot::kinematics()
{

    const double pi2 = pi/2.0;
    Matrix<double,5,6> UR5_dh_matrix(5,6);
    UR5_dh_matrix <<  -pi2, -pi2, 0, -pi2, 0, 0,
            0.089159-0.02315, 0, 0, 0.10915, 0.09465, 0.0823,
            0, -0.425, -0.39225, 0, 0, 0,
            pi2,0,0,pi2,-pi2,0,
            0,      0,       0,         0,         0,      0;
    DQ_SerialManipulatorDH UR5 (UR5_dh_matrix);
    return UR5;
}

}
