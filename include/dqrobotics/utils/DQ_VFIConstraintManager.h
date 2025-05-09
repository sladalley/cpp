/*
# Copyright (c) 2023 Juan Jose Quiroz Omana
#
#    constraints_manager is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    constraints_manager is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with constraints_manager.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Juan Jose Quiroz Omana, email: juanjqogm@gmail.com
#
# ################################################################*/

#pragma once

#include <tuple>
#include <memory>
#include <vector>
#include <algorithm>
#include <dqrobotics/DQ.h>
#include <dqrobotics/robot_modeling/DQ_Kinematics.h>
#include <dqrobotics/utils/DQ_Geometry.h>


using namespace Eigen;

namespace DQ_robotics
{



enum class Direction
{
    None,
    ForbiddenZone,
    SafeZone
};

enum class PrimitiveType
{
    None,
    Line,
    Point,
    Plane,
    Cone,
    LineAngle
};

struct robot_to_workspace_VFI_definition
{
    PrimitiveType robot_type = PrimitiveType:: None;
    PrimitiveType workspace_type = PrimitiveType:: None;
    std::shared_ptr<DQ_Kinematics> robot = nullptr;
    int joint_index = -1;
    int robot_index = 0;
    VectorXd joint_angles = VectorXd::Zero(0);
    DQ workspace_primitive = DQ(0);
    DQ workspace_primitive_2 = DQ(0);
    DQ line_plane_normal = -k_;
    Direction direction = Direction::None;
    double safe_distance = 0.0;
    double vfi_gain = 1;
    bool used = false;
    double cone_h = 0.0;
    double cone_phi = 0.0;

    robot_to_workspace_VFI_definition() = default;

    robot_to_workspace_VFI_definition(
        PrimitiveType robot_type,
        PrimitiveType workspace_type,
        std::shared_ptr<DQ_Kinematics> robot,
        int joint_index,
        int robot_index,
        const VectorXd& joint_angles,
        const DQ& workspace_primitive,
        double safe_distance,
        Direction direction,
        double vfi_gain,
        const DQ& line_plane_normal,
        const DQ& workspace_primitive_2,
        bool used,
        double cone_h,
        double cone_phi


    )
        : robot_type(robot_type),
          workspace_type(workspace_type),
          robot(robot),
          joint_index(joint_index),
          robot_index(robot_index),
          joint_angles(joint_angles),
          workspace_primitive(workspace_primitive),
          workspace_primitive_2(workspace_primitive_2),
          line_plane_normal(line_plane_normal),
          direction(direction),
          vfi_gain(vfi_gain),
          safe_distance(safe_distance),
          used(used),
          cone_h(cone_h),
          cone_phi(cone_phi)

    {}

};


class DQ_VFIConstraintManager
{
protected:
    int dim_configuration_;
    int num_robots_;
    VectorXd q_dot_min_ = VectorXd::Zero(0);
    VectorXd q_dot_max_ = VectorXd::Zero(0);
    VectorXd q_min_ = VectorXd::Zero(0);
    VectorXd q_max_ = VectorXd::Zero(0);

    MatrixXd equality_constraint_matrix_ = MatrixXd::Zero(0,0);
    VectorXd equality_constraint_vector_ = VectorXd::Zero(0);
    MatrixXd inequality_constraint_matrix_ = MatrixXd::Zero(0,0);
    VectorXd inequality_constraint_vector_ = VectorXd::Zero(0);

    std::vector<robot_to_workspace_VFI_definition> robot_to_workspace_vfis_;

    MatrixXd _raw_add_matrix_constraint(const MatrixXd& A0, const MatrixXd& A);
    VectorXd _raw_add_vector_constraint(const VectorXd& b0, const VectorXd& b);
    void _check_matrix_and_vector_sizes(const MatrixXd& A, const VectorXd& b);
    MatrixXd _make_matrix_compatible_size(const MatrixXd& A, const robot_to_workspace_VFI_definition &vfi);

    void _check_vectors_size(const VectorXd& q1, const VectorXd& q2, const std::string &msg);
    void _check_vector_initialization(const VectorXd& q, const std::string &msg);
    MatrixXd _create_matrix(const MatrixXd& A);

    std::tuple<MatrixXd, VectorXd> point_to_point_VFI(const robot_to_workspace_VFI_definition& vfi_);
    std::tuple<MatrixXd, VectorXd> point_to_line_VFI(const robot_to_workspace_VFI_definition& vfi_);
    std::tuple<MatrixXd, VectorXd> point_to_plane_VFI(const robot_to_workspace_VFI_definition& vfi_);
    std::tuple<MatrixXd, VectorXd> line_to_line_VFI(const robot_to_workspace_VFI_definition& vfi_);
    std::tuple<MatrixXd, VectorXd> line_to_point_VFI(const robot_to_workspace_VFI_definition& vfi_);
    std::tuple<MatrixXd, VectorXd> plane_to_point_VFI(const robot_to_workspace_VFI_definition& vfi_);
    std::tuple<MatrixXd, VectorXd> line_to_line_angle_VFI(const robot_to_workspace_VFI_definition& vfi_);
    std::tuple<MatrixXd, VectorXd> point_to_cone_VFI(const robot_to_workspace_VFI_definition& vfi_);


public:
    DQ_VFIConstraintManager() = delete;
    DQ_VFIConstraintManager(const int& dim_configuration, const int& num_robots);


    void add_equality_constraint(const MatrixXd& A, const VectorXd& b);
    void add_inequality_constraint(const MatrixXd& A, const VectorXd& b);

    std::tuple<MatrixXd, VectorXd> get_equality_constraints();
    std::tuple<MatrixXd, VectorXd> get_inequality_constraints();

    void set_joint_position_limits(const VectorXd& q_lower_bound, const VectorXd& q_upper_bound);
    void set_joint_velocity_limits(const VectorXd& q_dot_lower_bound, const VectorXd& q_dot_upper_bound);

   // void set_robot_to_robot_vfi(std::vector<std::tuple<PrimitiveType, PrimitiveType>> robot_to_robot_vfi_primitive_type, std::vector<std::tuple< std::shared_ptr<DQ_Kinematics>,  int, VectorXd,  std::shared_ptr<DQ_Kinematics> , int, VectorXd >> robot_to_robot_vfi, std::vector<std::tuple<Direction, double>> robot_to_robot_vfi_definition);
    void set_robot_to_workspace_vfi(const std::vector<robot_to_workspace_VFI_definition>& robot_to_workspace_vfis);
//    std::vector<robot_to_workspace_VFI_definition> get_robot_to_workspace_vfi();
    void get_robot_to_environment_vfi();

    void compute_robot_to_robot_constraint();
    void compute_robot_to_workspace_constraint();
    void compute_joint_velocity_constraint(const VectorXd& theta, const double& k, const double& gamma);
    std::tuple<double, double> compute_point_to_cone_values(const robot_to_workspace_VFI_definition& vfi_);




};
}
