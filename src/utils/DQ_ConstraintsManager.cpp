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


#include <dqrobotics/utils/DQ_ConstraintsManager.h>

/**
 * @brief Constructor
 * @param int dim_configuration Dimension of the robot configuration
 */
ConstraintsManager::ConstraintsManager(const int& dim_configuration):dim_configuration_(dim_configuration)
{

}

/**
 * @brief The method _create_matrix returns a matrix containing the matrix A
 *        and taking into acount the dimension of the robot configuration. If matrix A has lower columns than
 *        dim_configuration then _create_matrix completes the correct size with zeros.
 * @param MatrixXd A Input matrix
 * @return
 */
MatrixXd ConstraintsManager::_create_matrix(const MatrixXd& A)
{
    MatrixXd constraint_matrix = MatrixXd::Zero(A.rows(), dim_configuration_);
    constraint_matrix.block(0,0, A.rows(), A.cols()) = A;
    return constraint_matrix;
}


/**
 * @brief The method _check_matrix_and_vector_sizes(A,b) check the if the rows of Matrix A
 * has the same dimension of Vector b.
 * @param MatrixXd A
 * @param VectorXd b
 */
void ConstraintsManager::_check_matrix_and_vector_sizes(const MatrixXd& A, const VectorXd& b)
{
    int m = A.rows();
    int n = A.cols();
    int nb = b.size();
    if (m != nb)
    {
        throw std::runtime_error(std::string("Incompatible sizes. The rows of Matrix A must have the same dimension of Vector b. ")
                               + std::string("But you used A ")+ std::to_string(m)+ std::string("x")+ std::to_string(n)
                               + std::string(" and b ")+ std::to_string(nb) + std::string("x1"));
    }
    if (n != dim_configuration_)
    {
        throw std::runtime_error(std::string("Incompatible sizes. The cols of Matrix A must have dimension ") + std::to_string(dim_configuration_)
                                 + std::string(". But you used A ")+ std::to_string(m)+ std::string("x")+ std::to_string(n)
                                 + std::string(" and b ")+ std::to_string(nb) + std::string("x1"));
    }

}

/**
 * @brief ConstraintsManager::_check_vectors_size() checks the dimensions of two vectors.
 * @param VectorXd q1 First vector to be checked.
 * @param VectorXd q2 Second vector to be checked.
 * @param std::string msg Desired error message if vectors q1 and q2 have different size.
 */
void ConstraintsManager::_check_vectors_size(const VectorXd& q1, const VectorXd& q2, const std::string &msg)
{
    if (q1.size() != q2.size() )
    {
       throw std::runtime_error(msg);
    }
}

/**
 * @brief ConstraintsManager::_check_vector_initialization()
 * @param VectorXd q
 * @param std::string msg
 */
void ConstraintsManager::_check_vector_initialization(const VectorXd& q, const std::string &msg)
{
    if (q.size() == 0)
    {
        throw std::runtime_error(msg);
    }
}


/**
 * @brief The method _raw_add_matrix_constraint(A0, A) returns a constraint_matrix, which is given as
 *         constraint_matrix = [A0
 *                              A]
 * @param MatrixXd A0
 * @param MatrixXd A
 * @return MatrixXd constraint_matrix
 */
MatrixXd ConstraintsManager::_raw_add_matrix_constraint(const MatrixXd& A0, const MatrixXd& A)
{
        int m = A.rows();
        int n = A.cols();
        int m0 = A0.rows();
        int n0 = A0.cols();
        MatrixXd constraint_matrix = MatrixXd::Zero(m0+m, dim_configuration_);
        if(n != n0)
        {
            throw std::runtime_error(std::string("Incompatible sizes. The equality matrix must be ")
                                   + std::string("m x ")+ std::to_string(n0)
                                   + std::string(". But you used m x ")+ std::to_string(n));
        }
        constraint_matrix.block(0,0, m0, n0) = A0;
        constraint_matrix.block(m0,0, m, n) = A;
        return constraint_matrix;
}


/**
 * @brief The method _raw_add_vector_constraint(b0, b) returns the vector constraint_vector,
 *        which is given as
 *        constraint_vector = [b0
 *                             b];
 * @param VectorXd b0
 * @param VectorXd b
 * @return VectorXd constraint_vector
 */
VectorXd ConstraintsManager::_raw_add_vector_constraint(const VectorXd& b0, const VectorXd& b)
{
    int nb = b.size();
    int nb0 = b0.size();
    VectorXd constraint_vector =  VectorXd::Zero(nb0+nb);
    constraint_vector.head(nb0) = b0;
    constraint_vector.segment(nb0, nb) = b;
    return constraint_vector;
}


/**
 * @brief The method add_equality_constraint(A, b) adds the equality constraint A*x = b
 *        to the set of equality constraints.
 *
 * @param MatrixXd A
 * @param VectorXd b
 */
void ConstraintsManager::add_equality_constraint(const MatrixXd& A, const VectorXd& b)
{
    _check_matrix_and_vector_sizes(A, b);

    if (equality_constraint_matrix_.size() == 0)
    {
        equality_constraint_matrix_ = _create_matrix(A);
        equality_constraint_vector_ = b;
    }else
    {
        equality_constraint_matrix_ = _raw_add_matrix_constraint(equality_constraint_matrix_, A);
        equality_constraint_vector_ = _raw_add_vector_constraint(equality_constraint_vector_, b);
    }
}



/**
 * @brief The method add_inequality_constraint(A, b) adds the inequality constraint A*x <= b
 *        to the set of ineequality constraints.
 * @param MatrixXd A
 * @param VectorXd b
 */
void ConstraintsManager::add_inequality_constraint(const MatrixXd& A, const VectorXd& b)
{
    _check_matrix_and_vector_sizes(A, b);

    if (inequality_constraint_matrix_.size() == 0)
    {
        inequality_constraint_matrix_ = _create_matrix(A);
        inequality_constraint_vector_ = b;
    }else
    {
        inequality_constraint_matrix_ = _raw_add_matrix_constraint(inequality_constraint_matrix_, A);
        inequality_constraint_vector_ = _raw_add_vector_constraint(inequality_constraint_vector_, b);
    }
}


/**
 * @brief The method get_equality_constraints() returns the set of equality constraints
 *        composed of the Matrix A and the vector b, where
 *        A*x = b.
 *        Warning: This method deletes all the equality constraints stored.
 * @return std::tuple<MatrixXd A, VectorXd b>
 */
std::tuple<MatrixXd, VectorXd> ConstraintsManager::get_equality_constraints()
{
    MatrixXd A = equality_constraint_matrix_;
    equality_constraint_matrix_ = MatrixXd::Zero(0,0);
    return std::make_tuple(A, equality_constraint_vector_);
}


/**
 * @brief The method get_inequality_constraints() returns the set of inequality constraints
 *        composed of the Matrix A and the vector b, where
 *        A*x <= b
 *        Warning: This method deletes all the inequality constraints stored.
 * @return
 */
std::tuple<MatrixXd, VectorXd> ConstraintsManager::get_inequality_constraints()
{
    MatrixXd A = inequality_constraint_matrix_;
    inequality_constraint_matrix_ = MatrixXd::Zero(0,0);
    return std::make_tuple(A, inequality_constraint_vector_);
}


/**
 * @brief ConstraintsManager::set_joint_position_limits
 * @param VectorXd q_lower_bound
 * @param VectorXd q_upper_bound
 */
void ConstraintsManager::set_joint_position_limits(const VectorXd& q_lower_bound, const VectorXd& q_upper_bound)
{
    _check_vectors_size(q_lower_bound, q_upper_bound,
                          std::string("The sizes are incompatibles. q_lower_bound has size ") + std::to_string(q_lower_bound.size())
                          + std::string(" and q_upper_bound has size ") + std::to_string(q_upper_bound.size()));
    _check_vectors_size(q_lower_bound, VectorXd::Zero(dim_configuration_),
                          std::string("The sizes are incompatibles. The joint limits have size ") + std::to_string(q_lower_bound.size())
                          + std::string(" and the robot configuration has size ") + std::to_string(dim_configuration_));
    q_min_ = q_lower_bound;
    q_max_ = q_upper_bound;
}

/**
 * @brief ConstraintsManager::set_joint_velocity_limits
 * @param q_dot_lower_bound
 * @param q_dot_upper_bound
 */
void ConstraintsManager::set_joint_velocity_limits(const VectorXd& q_dot_lower_bound, const VectorXd& q_dot_upper_bound)
{
    _check_vectors_size(q_dot_lower_bound, q_dot_upper_bound,
                          std::string("The sizes are incompatibles. q_dot_lower_bound has size ") + std::to_string(q_dot_lower_bound.size())
                          + std::string(" and q_dot_upper_bound has size ") + std::to_string(q_dot_upper_bound.size()));
    _check_vectors_size(q_dot_lower_bound, VectorXd::Zero(dim_configuration_),
                          std::string("The sizes are incompatibles. The joint limits have size ") + std::to_string(q_dot_lower_bound.size())
                          + std::string(" and the robot configuration has size ") + std::to_string(dim_configuration_));
    q_dot_min_ = q_dot_lower_bound;
    q_dot_max_ = q_dot_upper_bound;
}
