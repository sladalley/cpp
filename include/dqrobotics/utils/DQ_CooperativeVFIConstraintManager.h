

#include <memory>
#include <dqrobotics/robot_modeling/DQ_CooperativeDualTaskSpace.h>
#include <dqrobotics/utils/DQ_VFIConstraintManager.h>

using namespace Eigen;

namespace DQ_robotics
{

enum class CooperativeVFIFrame
{
    None,
    AbsoluteFrame,
    RelativeFrame,
};

struct cooperative_VFI_definition
{
    CooperativeVFIFrame vfi_frame = CooperativeVFIFrame::None;
    PrimitiveType cooperative_type = PrimitiveType:: None;
    PrimitiveType workspace_type = PrimitiveType:: None;
    std::shared_ptr<DQ_CooperativeDualTaskSpace> cdts = nullptr;
    int joint_index = -1;
    VectorXd joint_angles = VectorXd::Zero(0);
    DQ workspace_primitive = DQ(0);
    DQ line_plane_normal = -k_;
    Direction direction = Direction::None;
    double safe_distance = 0.0;
    double vfi_gain = 1;

    cooperative_VFI_definition() = default;

    cooperative_VFI_definition(
        CooperativeVFIFrame vfi_frame,
        PrimitiveType cooperative_type,
        PrimitiveType workspace_type,
        std::shared_ptr<DQ_CooperativeDualTaskSpace> cdts,
        int joint_index,
        const VectorXd& joint_angles,
        const DQ& workspace_primitive,
        double safe_distance,
        Direction direction,
        double vfi_gain,
        const DQ& line_plane_normal

    )
        : vfi_frame(vfi_frame),
          cooperative_type(cooperative_type),
          workspace_type(workspace_type),
          cdts(cdts),
          joint_index(joint_index),
          joint_angles(joint_angles),
          workspace_primitive(workspace_primitive),
          line_plane_normal(line_plane_normal),
          direction(direction),
          vfi_gain(vfi_gain),
          safe_distance(safe_distance)
    {}

};

class DQ_CooperativeVFIConstraintManager: public DQ_VFIConstraintManager
{
protected:

    std::vector<cooperative_VFI_definition> cooperative_vfis_;

    std::tuple<MatrixXd, VectorXd> cooperative_line_to_line_angle_VFI(const cooperative_VFI_definition& vfi_);
    std::tuple<MatrixXd, VectorXd> cooperative_point_to_point_VFI(const cooperative_VFI_definition& vfi_);


public:
    DQ_CooperativeVFIConstraintManager(const int &dim, const int& num_robots);

    void set_cooperative_vfi(const std::vector<cooperative_VFI_definition>& cooperative_vfis);
    void get_robot_to_environment_vfi();

    void compute_cooperative_constraints();

};


}


