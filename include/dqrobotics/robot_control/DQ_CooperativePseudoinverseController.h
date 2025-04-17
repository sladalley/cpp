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
#include <dqrobotics/DQ.h>
#include <dqrobotics/robot_control/DQ_CooperativeKinematicController.h>

namespace DQ_robotics
{

class DQ_CooperativePseudoinverseController: public DQ_CooperativeKinematicController
{
public:
    DQ_CooperativePseudoinverseController() = delete;

    DQ_CooperativePseudoinverseController(const std::shared_ptr<DQ_CooperativeDualTaskSpace>& cdts);

    VectorXd compute_setpoint_control_signal(const VectorXd& q, const VectorXd& task_reference) override;
    VectorXd compute_tracking_control_signal(const VectorXd& q, const VectorXd& task_reference, const VectorXd& feed_forward) override;
};

}
