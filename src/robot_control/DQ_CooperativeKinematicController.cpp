/**

Contributors:
- Seyonne Leslie-Dalley
*/

#include <dqrobotics/robot_control/DQ_CooperativeKinematicController.h>
#include <stdexcept>
#include <dqrobotics/utils/DQ_Geometry.h>

namespace DQ_robotics
{

std::shared_ptr<DQ_CooperativeDualTaskSpace> DQ_CooperativeKinematicController::_get_cdts() const
{
    if(!cdts_sptr_)
        throw std::runtime_error("DQ_CooperativeKinematicController::_get_cdts invalid cdts pointer");
    return cdts_sptr_;
}

DQ_CooperativeKinematicController::DQ_CooperativeKinematicController(const std::shared_ptr<DQ_CooperativeDualTaskSpace> &cdts):
    DQ_CooperativeKinematicController()
{
    cdts_sptr_ = cdts;
}

DQ_CooperativeKinematicController::DQ_CooperativeKinematicController():
    cdts_sptr_(nullptr),
    control_objectives_(ControlObjective::None), //change to initialisation of empty vector
    control_frames_(ControlFrame::NoFrame), //change to initialisipn of empty vector
    attached_primitive_(0.0),
    target_primitive_(0.0),
    gain_(0.0),
    damping_(0),//Todo: change this inialization to use empty vector
    system_reached_stable_region_(false),//Todo: change this inialization to use empty vector
    last_control_signal_(VectorXd::Zero(1)),
    last_error_signal_(VectorXd::Zero(1)),
    stability_threshold_(0.0),
    stability_counter_(0.0),
    stability_counter_max_(10.0)
{

}

void DQ_CooperativeKinematicController::verify_stability(const VectorXd& task_error)
{
    if((last_error_signal_-task_error).norm() < stability_threshold_)
    {
        if(stability_counter_ < stability_counter_max_)
            stability_counter_++;
    }
    else
    {
        reset_stability_counter();
    }

    if(stability_counter_ >= stability_counter_max_)
    {
        system_reached_stable_region_ = true;
    }
}
std::vector<ControlObjective> DQ_CooperativeKinematicController::get_control_objectives() const
{
    return control_objectives_;
}

std::vector<ControlFrame> DQ_CooperativeKinematicController::get_control_frames() const
{
    return control_frames_;
}

VectorXd DQ_CooperativeKinematicController::get_last_error_signal() const
{
    return last_error_signal_;
}

MatrixXd DQ_CooperativeKinematicController::get_jacobian(const VectorXd &q) const
{
    std::shared_ptr<DQ_CooperativeDualTaskSpace> cdts_local =_get_cdts();
 // need a way to make sure the size of q is correct with the configuration space.
//    if(q.size() != cdts_local->_get_robot1_ptr()->get_dim_configuration_space())
//        throw std::runtime_error("Calling get_jacobian with an incorrect number of joints " + std::to_string(q.size()));

    if(control_objectives_.size()!=control_frames_.size())
        throw std::runtime_error("The number of control objective does not equal the number of defined control frames");

     MatrixXd J_pose;
     DQ       x_pose;
    for(unsigned int i = 0; i < control_objectives_.size();i ++){

        switch(control_frames_.at(i))
        {
        case ControlFrame::NoFrame:
            throw std::runtime_error("The control frame must be intialised with set_control_frame()");

        case ControlFrame::AbsoluteFrame:
            J_pose = cdts_local->absolute_pose_jacobian(q);
            x_pose = cdts_local->absolute_pose(q);
            break;

        case ControlFrame::RelativeFrame:
            J_pose = cdts_local->relative_pose_jacobian(q);
            x_pose = cdts_local->relative_pose(q);
            break;
        }

        switch(control_objectives_.at(i))
        {
        case ControlObjective::None:
            throw std::runtime_error("The control objective must be initialized with set_control_objective()");

        case ControlObjective::Distance:
            return DQ_Kinematics::distance_jacobian(J_pose,x_pose);

        case ControlObjective::DistanceToPlane:
        {
            if(!is_plane(target_primitive_))
            {
                throw std::runtime_error("Please set the target plane with the method set_target_primitive()");
            }
            MatrixXd Jt = DQ_Kinematics::translation_jacobian(J_pose, x_pose);
            DQ t = translation(x_pose);
            return DQ_Kinematics::point_to_plane_distance_jacobian(Jt, t, target_primitive_);
        }

        case ControlObjective::Line:
            return DQ_Kinematics::line_jacobian(J_pose,x_pose,attached_primitive_);

        case ControlObjective::Plane:
            return DQ_Kinematics::plane_jacobian(J_pose,x_pose,attached_primitive_);

        case ControlObjective::Rotation:
            return DQ_Kinematics::rotation_jacobian(J_pose);

        case ControlObjective::Translation:
            return DQ_Kinematics::translation_jacobian(J_pose,x_pose);

        case ControlObjective::Pose:
            return J_pose;
        }
    }
    //The only way I found to fix both possible warnings of either having a default in the switch or not having the default.
    throw std::runtime_error("Unknown ControlObjective");
}

VectorXd DQ_CooperativeKinematicController::get_task_variable(const VectorXd &q) const
{
    std::shared_ptr<DQ_CooperativeDualTaskSpace> cdts_local =_get_cdts();

    if(control_objectives_.size()!=control_frames_.size())
        throw std::runtime_error("The number of control objective does not equal the number of defined control frames");

//    if(q.size() != robot_local->get_dim_configuration_space())
//        throw std::runtime_error("Calling get_task_variable with an incorrect number of joints " + std::to_string(q.size()));

     DQ x_pose;
     std::vector<VectorXd> task;
     int size = 0;


    for(unsigned int i = 0; i < control_frames_.size();i++){

        switch(control_frames_.at(i))
        {
        case ControlFrame::NoFrame:
            throw std::runtime_error("The control frame must be intialised with set_control_frame()");

        case ControlFrame::AbsoluteFrame:
            x_pose = cdts_local->absolute_pose(q);
            break;

        case ControlFrame::RelativeFrame:
            x_pose = cdts_local->relative_pose(q);
            break;
        }

        VectorXd task_variable;

        switch(control_objectives_.at(i))
        {
        case ControlObjective::None:
            throw std::runtime_error("The control objective must be initialized with set_control_objective()");

        case ControlObjective::Distance:
        {
            VectorXd p = vec4(translation(x_pose));
            task_variable = p.transpose()*p;
            break;
        }

        case ControlObjective::DistanceToPlane:
        {
            if(!is_plane(target_primitive_))
            {
                throw std::runtime_error("Set the target plane with the method set_target_primitive()");
            }
            DQ t = translation(x_pose);
            VectorXd distance(1);
            distance(0)=DQ_Geometry::point_to_plane_distance(t, target_primitive_);
            task_variable = distance;
            break;
        }

        case ControlObjective::Line:
            task_variable = vec8(Ad(x_pose,attached_primitive_));
            break;

        case ControlObjective::Plane:
            task_variable = vec8(Adsharp(x_pose,attached_primitive_));
            break;

        case ControlObjective::Rotation:
            task_variable = vec4(rotation(x_pose));
            break;

        case ControlObjective::Translation:
            task_variable = vec4(translation(x_pose));
            break;

        case ControlObjective::Pose:
            task_variable = vec8(x_pose);
            break;
        }

        task.push_back(task_variable);
        size +=task_variable.size();
    }
    VectorXd overall_task(size);
    for (unsigned int i = 0; i < control_objectives_.size();i++){
        overall_task << task.at(i);
    }
    return overall_task;
}

bool DQ_CooperativeKinematicController::is_set() const
{
    bool set = false;
    for(int i = 0;i<control_objectives_.size();i++){

        if(control_objectives_.at(i)==ControlObjective::None)
        {
            set = false;
            break;
        }
        else
        {
            set = true;
        }
    }
    return set;
}

bool DQ_CooperativeKinematicController::system_reached_stable_region() const
{
    return system_reached_stable_region_;
}

void DQ_CooperativeKinematicController::set_control_objectives(const std::vector<ControlObjective>& control_objectives)
{
    control_objectives_ = control_objectives;
    for (unsigned int i = 0; i < control_objectives_.size();i++){
        switch(control_objectives.at(i))
        {
        case ControlObjective::Distance: //This was intentional https://en.cppreference.com/w/cpp/language/attributes/fallthrough
        case ControlObjective::DistanceToPlane:
            last_error_signal_ << VectorXd::Zero(1);
            break;
        case ControlObjective::Line: //This was intentional https://en.cppreference.com/w/cpp/language/attributes/fallthrough
        case ControlObjective::Plane: //This was intentional https://en.cppreference.com/w/cpp/language/attributes/fallthrough
        case ControlObjective::Pose:
            last_error_signal_ << VectorXd::Zero(8);
            break;
        case ControlObjective::Rotation:
        case ControlObjective::Translation:
            last_error_signal_ << VectorXd::Zero(4);
            break;
        case ControlObjective::None:
            break;
        }
    }

}

void DQ_CooperativeKinematicController::set_control_frames(const std::vector<ControlFrame> &control_frames)
{
  control_frames_ = control_frames;
}

void DQ_CooperativeKinematicController::set_gain(const double& gain)
{
    gain_ = gain;
}

double DQ_CooperativeKinematicController::get_gain() const
{
    return gain_;
}

void DQ_CooperativeKinematicController::set_stability_threshold(const double &threshold)
{
    stability_threshold_ = threshold;
}

void DQ_CooperativeKinematicController::set_primitive_to_effector(const DQ &primitive)
{
    attached_primitive_ = primitive;
}

void DQ_CooperativeKinematicController::set_target_primitive(const DQ &primitive)
{
    target_primitive_ = primitive;
}

void DQ_CooperativeKinematicController::set_damping(const double &damping)
{
    damping_ = damping;
}

double DQ_CooperativeKinematicController::get_damping() const
{
    return damping_;
}

void DQ_CooperativeKinematicController::set_stability_counter_max(const int &max)
{
    stability_counter_max_ = max;
}

void DQ_CooperativeKinematicController::reset_stability_counter()
{
    stability_counter_ = 0;
    system_reached_stable_region_ = false;
}

}
