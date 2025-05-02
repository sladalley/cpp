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
#include <Eigen/Dense>
#include <vector>


using namespace Eigen;

class ConstraintsManager
{
protected:
    int dim_configuration_;

    VectorXd q_dot_min_ = VectorXd::Zero(0);
    VectorXd q_dot_max_ = VectorXd::Zero(0);
    VectorXd q_min_ = VectorXd::Zero(0);
    VectorXd q_max_ = VectorXd::Zero(0);

    MatrixXd equality_constraint_matrix_ = MatrixXd::Zero(0,0);
    VectorXd equality_constraint_vector_ = VectorXd::Zero(0);
    MatrixXd inequality_constraint_matrix_ = MatrixXd::Zero(0,0);
    VectorXd inequality_constraint_vector_ = VectorXd::Zero(0);

    MatrixXd _raw_add_matrix_constraint(const MatrixXd& A0, const MatrixXd& A);
    VectorXd _raw_add_vector_constraint(const VectorXd& b0, const VectorXd& b);
    void _check_matrix_and_vector_sizes(const MatrixXd& A, const VectorXd& b);

    void _check_vectors_size(const VectorXd& q1, const VectorXd& q2, const std::string &msg);
    void _check_vector_initialization(const VectorXd& q, const std::string &msg);
    MatrixXd _create_matrix(const MatrixXd& A);

public:
    ConstraintsManager() = delete;
    ConstraintsManager(const int& dim_configuration);

    void add_equality_constraint(const MatrixXd& A, const VectorXd& b);
    void add_inequality_constraint(const MatrixXd& A, const VectorXd& b);

    std::tuple<MatrixXd, VectorXd> get_equality_constraints();
    std::tuple<MatrixXd, VectorXd> get_inequality_constraints();

    void set_joint_position_limits(const VectorXd& q_lower_bound, const VectorXd& q_upper_bound);
    void set_joint_velocity_limits(const VectorXd& q_dot_lower_bound, const VectorXd& q_dot_upper_bound);
};
