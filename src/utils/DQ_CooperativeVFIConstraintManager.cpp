#include <dqrobotics/utils/DQ_CooperativeVFIConstraintManager.h>

namespace DQ_robotics
{


DQ_CooperativeVFIConstraintManager::DQ_CooperativeVFIConstraintManager(const int& dim, const int& num_robots):
    DQ_VFIConstraintManager(dim, num_robots)
{

}

void DQ_CooperativeVFIConstraintManager::compute_cooperative_constraints()
{
    for(unsigned int i = 0; i<cooperative_vfis_.size(); i++){
        auto vfi = cooperative_vfis_.at(i);
        MatrixXd constraint_jacobian;
        VectorXd constraint_derror = VectorXd(1);

        switch (vfi.vfi_frame) {
            case CooperativeVFIFrame::None:
                throw std::runtime_error("The vfi frame must be intialised with in the vfi definition");
            case CooperativeVFIFrame::AbsoluteFrame:
                //point_to_point_constraint
                //pose_constraint
                if((vfi.cooperative_type == PrimitiveType::LineAngle) && (vfi.workspace_type == PrimitiveType::LineAngle)){
                            std::tie(constraint_jacobian, constraint_derror) = cooperative_line_to_line_angle_VFI(vfi);
                            add_inequality_constraint(constraint_jacobian, constraint_derror);
                        }

                else throw std::runtime_error("Not defined");

            case CooperativeVFIFrame::RelativeFrame:
            //pose_constraint
            //position_constraint
            if((vfi.cooperative_type == PrimitiveType::LineAngle) && (vfi.workspace_type == PrimitiveType::LineAngle)){
                        std::tie(constraint_jacobian, constraint_derror) = cooperative_line_to_line_angle_VFI(vfi);
                        add_inequality_constraint(constraint_jacobian, constraint_derror);
                    }
                 else throw std::runtime_error("TNot defined");

        }
    }
}

std::tuple<MatrixXd, VectorXd> DQ_CooperativeVFIConstraintManager::cooperative_line_to_line_angle_VFI(const cooperative_VFI_definition &vfi){
    MatrixXd line_to_line_angle_jacobian;
    VectorXd line_to_line_angle_derror = VectorXd(1);
    auto cdts_ptr = vfi.cdts;
    auto q = vfi.joint_angles;
    auto workspace_line = vfi.workspace_primitive;

    if(!is_line(workspace_line)){
        throw std::runtime_error("Workspace primitive is not a line");
    }

    auto zone = vfi.direction;
    auto d_safe = vfi.safe_distance;
    DQ x;
    MatrixXd Jx;
    if (vfi.vfi_frame==CooperativeVFIFrame::AbsoluteFrame){
    x = cdts_ptr->absolute_pose(q);
    Jx = cdts_ptr->absolute_pose_jacobian(q);
    }
    else if (vfi.vfi_frame==CooperativeVFIFrame::RelativeFrame){
        x = cdts_ptr->relative_pose(q);
        Jx = cdts_ptr->relative_pose_jacobian(q);
        }
    else throw std::runtime_error("Check vfi_frame is set correctly");

    DQ line_normal = vfi.line_plane_normal;
    DQ cooperative_frame_line = Ad(x.P(),line_normal);
    auto Jline = DQ_Kinematics::line_jacobian(Jx,x, line_normal);

    auto J = DQ_Kinematics::line_to_line_angle_jacobian(Jline,line_normal, workspace_line);

    double phi = DQ_Geometry::line_to_line_angle(cooperative_frame_line, workspace_line);
     std::cout << "angle: " << phi*180/(3.1415926) << std::endl;
    double f = 2-2*cos(phi);
    double fsafe = 2-2*cos(d_safe);

    if(zone == Direction::ForbiddenZone){
        line_to_line_angle_jacobian = -J;
        line_to_line_angle_derror << f - fsafe;
    }
    else if(zone == Direction::SafeZone){
        line_to_line_angle_jacobian = J;
        line_to_line_angle_derror <<  fsafe - f;
    }
    else throw std::runtime_error("Zone of VFI not defined");

    return std::make_tuple(line_to_line_angle_jacobian, vfi.vfi_gain*line_to_line_angle_derror);
}

void DQ_CooperativeVFIConstraintManager::set_cooperative_vfi(const std::vector<cooperative_VFI_definition> &cooperative_vfis)
{
  cooperative_vfis_= cooperative_vfis;
}


}
