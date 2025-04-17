#pragma once
/**
(C) Copyright 2019-2022 DQ Robotics Developers

This file is part of DQ Robotics.

    DQ Robotics is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    DQ Robotics is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.

Contributors:
- Murilo M. Marinho (murilomarinho@ieee.org)
*/

#include<dqrobotics/robot_control/DQ_CooperativeKinematicConstrainedController.h>
#include<dqrobotics/solvers/DQ_QuadraticProgrammingSolver.h>

using namespace Eigen;

namespace DQ_robotics
{
class DQ_CooperativeQuadraticProgrammingController:public DQ_CooperativeKinematicConstrainedController
{
protected:

    std::shared_ptr<DQ_QuadraticProgrammingSolver> qp_solver_sptr_;

    DQ_CooperativeQuadraticProgrammingController(const std::shared_ptr<DQ_CooperativeDualTaskSpace>& cdts,
                                      const std::shared_ptr<DQ_QuadraticProgrammingSolver>& solver);
public:
    //Remove default constructor
    DQ_CooperativeQuadraticProgrammingController()=delete;

    virtual MatrixXd compute_objective_function_symmetric_matrix(const MatrixXd& J, const VectorXd& task_error)=0;
    virtual VectorXd compute_objective_function_linear_component(const MatrixXd& J, const VectorXd& task_error)=0;

    virtual VectorXd compute_setpoint_control_signal(const VectorXd&q, const VectorXd& task_reference) override;
    virtual VectorXd compute_tracking_control_signal(const VectorXd&q, const VectorXd& task_reference, const VectorXd& feed_forward) override;

};
}
