

#include <memory>

#include <dqrobotics/utils/DQ_VFIConstraintManager.h>

using namespace Eigen;

namespace DQ_robotics
{

class DQ_CooperativeVFIConstraintManager: public DQ_VFIConstraintManager
{
protected:
    MatrixXd equality_constraint_matrix_;
    VectorXd equality_constraint_vector_;
    MatrixXd inequality_constraint_matrix_;
    VectorXd inequality_constraint_vector_;

    DQ_CooperativeVFIConstraintManager(const int &dim);
public:
    //Remove default constructor

};


}


