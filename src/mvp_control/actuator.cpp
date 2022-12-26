/*
    This file is part of MVP-Control program.

    MVP-Control is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    MVP-Control is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MVP-Control.  If not, see <https://www.gnu.org/licenses/>.

    Author: Emir Cem Gezer
    Email: emircem@uri.edu;emircem.gezer@gmail.com
    Year: 2022

    Copyright (C) 2022 Smart Ocean Systems Laboratory
*/

#include "actuator.hpp"
#include "utility"
#include "exception.hpp"
#include "mvp_control/dictionary.h"
#include "polynomial_solver.hpp"

using namespace mvp;

void Actuator::f_compute_single_axis_contribution(
    const Eigen::Isometry3d &t,
    int axis) {

    auto rpy = t.translation().cross(t.rotation().col(axis));

    /**
     * X, Y, Z
     * roll, pitch, yaw
     * surge, sway, heave,
     * roll_rate, pitch_rate, yaw_rate
     */

    m_contribution_matrix.col(axis) <<
        Eigen::Vector3d::Zero(),
        rpy,
        t.rotation().col(axis),
        rpy;
}


void Thruster::initialize() {

    // todo: topics will go here

}

void Thruster::compute_constraints() {
    m_upper_limit << m_config.force_limits[MAX];
    m_lower_limit << m_config.force_limits[MIN];

    m_constraint_matrix = Eigen::MatrixXd(3, 1);
    m_constraint_matrix << 1, 0 ,0;
}

void Thruster::compute_contribution(const Eigen::Isometry3d &t) {

    // get contribution for X axis
    f_compute_single_axis_contribution(t, 0);
}

bool Thruster::command(const Eigen::Vector3d &N) {

    double result;
    bool ifreal = PolynomialSolver::solve(
        m_config.control_curve,
        nullptr,
        &result,
        N.x()
    );

    if(!ifreal) {
        std::cerr << "No feasible command found for force: " << N << std::endl;
        return false;
    }

    return true;
}

void AzimuthThruster::initialize() {

    // todo: topics will go here


}

void AzimuthThruster::compute_contribution(const Eigen::Isometry3d &t) {

    // get contribution for X axis
    f_compute_single_axis_contribution(t, 0);

    // get contribution for Z axis
    if(!m_config.joint_y_limits.empty()) {
        f_compute_single_axis_contribution(t, 2);
    }

    // get contribution for Y axis
    if(!m_config.joint_z_limits.empty()) {
        f_compute_single_axis_contribution(t, 1);
    }
}

void AzimuthThruster::compute_constraints() {

    // Get number of joints and force components first
    int n_joints = 0;
    int r = 0;

    // assume the force is on the X direction
    double Fx{1}, Fy{0}, Fz{0};

    // check the configuration for joints
    if(!m_config.joint_y_limits.empty()){
        Fz = 1;
        n_joints++;
    }
    if(!m_config.joint_y_limits.empty()){
        Fy = 1;
        n_joints++;
    }

    // Resize the constraint matrix
    m_constraint_matrix = Eigen::MatrixXd(3, 1 + n_joints * 2);

    // setup for force
    m_constraint_matrix.row(r) << Fx, Fy, Fz;
    m_upper_limit(r) = m_config.force_limits[MAX];
    m_lower_limit(r) = m_config.force_limits[MIN];

    // setup angle constraints
    if(Fz == 1) {
        r++;
        m_constraint_matrix.row(r) << std::tan(m_config.joint_y_limits[MAX]), 0, -1;
        m_upper_limit(r) = 0;
        m_lower_limit(r) = 0;

        r++;
        m_constraint_matrix.row(r) << std::tan(m_config.joint_y_limits[MIN]), 0, -1;
        m_upper_limit(r) = 0;
        m_lower_limit(r) = 0;
    }

    // setup angle constraints
    if(Fy == 1) {
        r++;
        m_constraint_matrix.row(r) << 0, std::tan(m_config.joint_y_limits[MAX]), -1;
        m_upper_limit(r) = 0;
        m_lower_limit(r) = 0;

        r++;
        m_constraint_matrix.row(r) << 0, std::tan(m_config.joint_y_limits[MIN]), -1;
        m_upper_limit(r) = 0;
        m_lower_limit(r) = 0;
    }
}

bool AzimuthThruster::command(const Eigen::Vector3d& N) {

    if(!m_config.joint_y_limits.empty()) {
        auto angle = atan2(N.x(), N.z());
    }

    double result;
    bool ifreal = PolynomialSolver::solve(
        m_config.control_curve,
        nullptr,
        &result,
        N.norm()
    );

    if(!ifreal) {
        std::cout << "No feasible command found for force: " << N << std::endl;
        return false;
    }

    return true;
}