#pragma once
/**


Contributors:
- Seyonne Leslie-Dalley
*/

#include <memory>
#include <vector>
#include <dqrobotics/DQ.h>
#include <dqrobotics/robot_modeling/DQ_CooperativeDualTaskSpace.h>

namespace DQ_robotics
{

enum CooperativeControlObjective
{
    Coop_None,
    Coop_Distance,
    Coop_DistanceToPlane,
    Coop_Line,
    Coop_Plane,
    Coop_Pose,
    Coop_Rotation,
    Coop_Translation
};

enum CooperativeControlFrame
{
    NoFrame,
    AbsoluteFrame,
    RelativeFrame
};

class DQ_CooperativeKinematicController
{
protected:

    std::shared_ptr<DQ_CooperativeDualTaskSpace> cdts_sptr_;
    std::vector<CooperativeControlObjective> control_objectives_;
    std::vector<CooperativeControlFrame> control_frames_;
    DQ attached_primitive_;
    DQ target_primitive_;

    double gain_;
    double damping_;

    bool system_reached_stable_region_;
    VectorXd last_control_signal_;
    VectorXd last_error_signal_;

    double stability_threshold_;
    int stability_counter_;
    int stability_counter_max_;

    std::shared_ptr<DQ_CooperativeDualTaskSpace> _get_cdts() const;

    DQ_CooperativeKinematicController(const std::shared_ptr<DQ_CooperativeDualTaskSpace>& cdts);
    DQ_CooperativeKinematicController();
public:

    std::vector<CooperativeControlObjective> get_control_objectives() const;
    std::vector<CooperativeControlFrame> get_control_frames() const;

    MatrixXd get_jacobian(const VectorXd& q) const;

    VectorXd get_task_variable(const VectorXd& q) const;

    VectorXd get_last_error_signal() const;

    bool is_set() const;

    bool system_reached_stable_region() const;

    void set_control_objectives(const std::vector<CooperativeControlObjective>& control_objectives);
    void set_control_frames(const std::vector<CooperativeControlFrame>& control_frames);

    void set_gain(const double& gain);
    double get_gain() const;

    void set_damping(const double& damping);
    double get_damping() const;

    void set_stability_threshold(const double& threshold);

    void set_primitive_to_effector(const DQ& primitive);

    void set_target_primitive(const DQ& primitive);

    void set_stability_counter_max(const int& max);

    void reset_stability_counter();

    //Virtual
    virtual ~DQ_CooperativeKinematicController()=default;
   virtual VectorXd compute_setpoint_control_signal(const VectorXd& q, const VectorXd& task_reference)=0;
   virtual VectorXd compute_tracking_control_signal(const VectorXd& q, const VectorXd& task_reference, const VectorXd& feed_forward)=0;
    virtual void verify_stability(const VectorXd& task_error);

};

}
