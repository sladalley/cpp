
#include <dqrobotics/robot_control/DQ_CooperativePseudoinverseController.h>
#include <dqrobotics/utils/DQ_LinearAlgebra.h>

namespace DQ_robotics
{



DQ_CooperativePseudoinverseController::DQ_CooperativePseudoinverseController(const std::shared_ptr<DQ_CooperativeDualTaskSpace> &cdts):
    DQ_CooperativeKinematicController(cdts)
{

}

VectorXd DQ_CooperativePseudoinverseController::compute_setpoint_control_signal(const VectorXd& q, const VectorXd& task_reference)
{
    return DQ_CooperativePseudoinverseController::compute_tracking_control_signal(q,task_reference,VectorXd::Zero(task_reference.size()));
}

VectorXd DQ_CooperativePseudoinverseController::compute_tracking_control_signal(const VectorXd &q, const VectorXd &task_reference, const VectorXd &feed_forward)
{
    if(is_set())
    {
        const VectorXd task_variable = get_task_variable(q);

        const MatrixXd J = get_jacobian(q);

        const VectorXd task_error = task_variable-task_reference;

        VectorXd u;
        if(damping_ == 0.0)
            u = pinv(J)*(-gain_*task_error + feed_forward);
        else
            u = (J.transpose()*J + damping_*damping_*MatrixXd::Identity(q.size(), q.size())).inverse()*
                    J.transpose()*
                    (-gain_*task_error + feed_forward);

        verify_stability(task_error);

        last_control_signal_ = u;
        last_error_signal_   = task_error;
        return u;
    }
    else
    {
        throw std::runtime_error("Tried computing the control signal of an unset controller.");
    }
}

}
