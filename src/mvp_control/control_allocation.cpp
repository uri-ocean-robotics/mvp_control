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

/*******************************************************************************
 * Project
 */
#include "control_allocation.hpp"
#include "exception.hpp"
#include "mvp_control/dictionary.h"

/*******************************************************************************
 * STL
 */

#include "functional"
#include "mutex"

/*******************************************************************************
 * Quadratic Solver
 */
#include "osqp++.h"

/*******************************************************************************
 * Eigen
 */
#include "Eigen/Core"

using namespace mvp;

ControlAllocation::ControlAllocation() {

    /**
     * Initialize the error state
     */
    m_error_state = Eigen::VectorXd(CONTROLLABLE_DOF_LENGTH);

    /**
     * Create the multiple input multiple output PID controller
     */
    m_pid.reset(new MimoPID());

    /**
     * MIMO PID object does not implement the error function. It asks programmer
     * to assign a error function. When it calculates the gains it uses the
     * function that is binded to it's error function.
     */
    m_pid->set_error_function(
        std::bind(
            &ControlAllocation::f_error_function,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

}

bool ControlAllocation::calculate_forces(Eigen::VectorXd *cmd, double dt) {

    /**
     * This function basically computes one iteration of the controller. It
     * computes the PID gains. Optimizes the thrust.
     */

    /**
     * vector 'u' represents the input values of the system. That is all degrees
     * of freedoms.
     */
    Eigen::VectorXd u;
    if(!f_calculate_pid(&u, dt)){
        return false;
    }

    /**
     * Below code computes the forces that later will be requested from the
     * thrusters. Values in the force vector are not thruster set points yet.
     */
    if(f_solve(cmd, u)) {
        return true;
    }

    return false;
}

bool ControlAllocation::f_calculate_pid(Eigen::VectorXd *u, double dt) {
    return m_pid->calculate(u, m_desired_state, m_system_state, dt);
}

bool ControlAllocation::f_solve(
    Eigen::VectorXd *cmd, const Eigen::VectorXd &desired) {

    // Control allocation matrix
    Eigen::MatrixXd T(
        m_controlled_freedoms.size(),
        m_control_allocation_matrix.cols()
    );

    // Control matrix
    Eigen::VectorXd Tau(m_controlled_freedoms.size());

    {
        /**rix and the
         * control input. This is important for online mode updates.
         *
         * Quadratic programming solving is probably the most time consuming
         * part of the program. During that time changes may happen. Most likely
         * change would happen on the 'm_controlled_freedoms' vector. This
         * vector updates when controllers mode change. Scoped lock is making
         * sure that those variables will not change while execution is inside
         * in this scope.
         */
        std::scoped_lock lock(m_allocation_matrix_lock,
                              m_controlled_freedoms_lock);

        /**
         * We are using a portion of the control allocation mat
         */
        for (int i = 0; i < m_controlled_freedoms.size(); i++) {
            T.row(i) = m_control_allocation_matrix.row(
                m_controlled_freedoms.at(i));
            Tau(i) = desired(m_controlled_freedoms.at(i));
        }
    }

    /**
     * Now we are preparing data for quadratic solver to consume.
     */

    // Q -> objective matrix
    Eigen::MatrixXd Q = 2 * T.transpose() * T;
    // c -> objective vector
    Eigen::VectorXd c = (-2 * (Tau.transpose() * T)).transpose();

    std::vector<Eigen::Triplet<double>> Q_triplets;
    for(int i = 0 ; i < Q.rows() ; i++) {
        for(int j = 0 ; j < Q.cols() ; j++) {
            Q_triplets.emplace_back(Eigen::Triplet<double>{i, j, Q(i,j)});
        }
    }

    std::vector<Eigen::Triplet<double>> A_triplets;
    for(int i = 0 ; i < m_control_constraints_matrix.rows() ; i++) {
        for(int j = 0 ; j < m_control_constraints_matrix.cols() ; j++) {
            A_triplets.emplace_back(
                Eigen::Triplet<double>{
                    i, j, m_control_constraints_matrix(i,j)
                }
            );
        }
    }
    // Creating a quadratic solver instance.
    osqp::OsqpInstance qp_instance;

    /**
     * Translating the objective matrix into sparse matrix. Eigen::SparseMatrix
     * is the data type that is consumed by quadratic solver.
     */

    // Set the objective matrix
    qp_instance.objective_matrix = Eigen::SparseMatrix<double>(
        Q.rows(),
        Q.cols());

    qp_instance.objective_matrix.setFromTriplets(
        Q_triplets.begin(),
        Q_triplets.end());

    // Set the objective vector
    qp_instance.objective_vector = c;

    // Set the bounds
    qp_instance.lower_bounds = m_lower_limit;
    qp_instance.upper_bounds = m_upper_limit;

    // Set the constraint matrix
    qp_instance.constraint_matrix = Eigen::SparseMatrix<double>(
        m_control_constraints_matrix.cols(),
        m_control_constraints_matrix.cols());

    qp_instance.constraint_matrix.setFromTriplets(
        A_triplets.begin(),
        A_triplets.end());

    // Initialize the solver
    osqp::OsqpSolver solver;
    osqp::OsqpSettings settings;

    settings.verbose = false;

    auto status = solver.Init(qp_instance, settings);

    if(not status.ok()) {
        std::cerr << "Can not set the OSQP solver!" << std::endl;
        return false;
    }

    osqp::OsqpExitCode exitCode = solver.Solve();

    if(exitCode == osqp::OsqpExitCode::kOptimal) {
        *cmd = solver.primal_solution();
        return true;
    } else {
        std::cerr
            << "OSQP Solver returned non optimal solution, exit code: "
            << static_cast<std::underlying_type<osqp::OsqpExitCode>::type>(
                exitCode)
            << std::endl;
        return false;
    }

}

Eigen::ArrayXd ControlAllocation::f_error_function(Eigen::ArrayXd desired,
                                                   Eigen::ArrayXd current)
{

    std::lock_guard<std::recursive_mutex> lock(m_desired_state_lock);

    if(desired.size() != current.size()) {
        throw control_exception(
            "desired and current state sizes are different"
        );
    }

    Eigen::ArrayXd error = desired - current;

    for(const auto& i : {DOF::ROLL, DOF::PITCH, DOF::YAW,
                         DOF::ROLL_RATE, DOF::PITCH_RATE, DOF::YAW_RATE}) {

        // todo: wrap2pi implementation

        auto d = (fmod(desired(i) + M_PI, 2*M_PI) - M_PI);
        auto c = (fmod(current(i) + M_PI, 2*M_PI) - M_PI);

        auto t = d - c;
        double diff = (fmod(t + M_PI, 2*M_PI) - M_PI);
        error(i) = diff < -M_PI ? diff + 2*M_PI : diff;
    }

    m_error_state = error;

    return error;
}

void ControlAllocation::update_freedoms(std::vector<int> freedoms) {
    std::scoped_lock lock(m_controlled_freedoms_lock);
    m_controlled_freedoms = std::move(freedoms);

}

void ControlAllocation::update_desired_state(
        const decltype(m_desired_state) &desired_state) {
    std::scoped_lock lock(m_desired_state_lock);
    m_desired_state = desired_state;
}

void ControlAllocation::update_for_pose(const Eigen::Isometry3d& t) {

    // for each thruster compute contribution in earth frame
    for(int j = 0 ; j < m_control_allocation_matrix.cols() ; j++) {
        Eigen::Vector3d uvw{
            m_control_allocation_matrix(DOF::SURGE, j),
            m_control_allocation_matrix(DOF::SWAY, j),
            m_control_allocation_matrix(DOF::HEAVE, j)
        };

        auto xyz = t.rotation() * uvw;

        m_control_allocation_matrix(DOF::X, j) = xyz(0);
        m_control_allocation_matrix(DOF::Y, j) = xyz(1);
        m_control_allocation_matrix(DOF::Z, j) = xyz(2);
    }

}

void ControlAllocation::compose_matrices(
    const std::vector<std::shared_ptr<Actuator>> &actuators) {

    // compose the control allocation matrix
    //      during the operation, this matrix is susceptible to change.
    //              (Fx  Fy  Fz)(Fx  Fy  Fz)(Fx  Fy  Fz)
    // x        -> [ a0, a0, a0, a1, a1, a1, a2, a2, a2 ...]
    // y        -> [ a0, a0, a0, a1, a1, a1, a2, a2, a2 ...]
    // z        -> [ a0, a0, a0, a1, a1, a1, a2, a2, a2 ...]
    // roll     -> [ a0, a0, a0, a1, a1, a1, a2, a2, a2 ...]
    // pitch    -> [ a0, a0, a0, a1, a1, a1, a2, a2, a2 ...]
    // yaw      -> [ a0, a0, a0, a1, a1, a1, a2, a2, a2 ...]
    // surge    -> [ a0, a0, a0, a1, a1, a1, a2, a2, a2 ...]
    // sway     -> [ a0, a0, a0, a1, a1, a1, a2, a2, a2 ...]
    // heave    -> [ a0, a0, a0, a1, a1, a1, a2, a2, a2 ...]
    // roll rate-> [ a0, a0, a0, a1, a1, a1, a2, a2, a2 ...]
    // pitch rate> [ a0, a0, a0, a1, a1, a1, a2, a2, a2 ...]
    // yaw rate -> [ a0, a0, a0, a1, a1, a1, a2, a2, a2 ...]

    m_control_allocation_matrix = Eigen::MatrixXd::Zero(
        CONTROLLABLE_DOF_LENGTH, 3 * actuators.size());

    // initialize row and column counters
    long r = 0;
    long c = 0;
    for(const auto& i : actuators) {
        m_control_allocation_matrix.block(
            r,
            c,
            r,
            c + i->get_contribution_matrix().cols()
        ) = i->get_contribution_matrix();
        c += i->get_contribution_matrix().cols();
    }

    // compose constraint matrix.
    //      during the operation this matrix will not change.
    // Example:
    // [a0, a0, a0,  0,  0,  0,  0,  0,  0 ...]
    // [a0, a0, a0,  0,  0,  0,  0,  0,  0 ...]
    // [a0, a0, a0,  0,  0,  0,  0,  0,  0 ...]
    // [a0, a0, a0,  0,  0,  0,  0,  0,  0 ...]
    // [a0, a0, a0,  0,  0,  0,  0,  0,  0 ...]
    // [ 0,  0,  0, a1, a1, a1,  0,  0,  0 ...]
    // [ 0,  0,  0,  0,  0,  0, a2, a2, a2 ...]
    // [ 0,  0,  0,  0,  0,  0, a2, a2, a2 ...]
    // [ 0,  0,  0,  0,  0,  0, a2, a2, a2 ...]
    // [ .,  .,  .,  .,  .,  .,  .,  .,  . ...]
    // [ .,  .,  .,  .,  .,  .,  .,  .,  . ...]

    long row_size = 0;
    long col_size = 0;
    for(const auto& i : actuators) {
        row_size += i->get_constraint_matrix().rows();
        col_size += i->get_constraint_matrix().cols();
    }

    m_control_constraints_matrix = Eigen::MatrixXd::Zero(
        row_size, col_size);

    // reset row and column counters
    r = 0;
    c = 0;
    for(const auto& i : actuators) {
        m_control_constraints_matrix.block(
            r,
            c,
            r + i->get_constraint_matrix().rows(),
            c + i->get_constraint_matrix().cols()
        ) = i->get_constraint_matrix();
        r += i->get_constraint_matrix().rows();
        c += i->get_constraint_matrix().cols();
    }

    // combine limits
    // reset row and column counters
    r = 0;
    c = 0;
    for(const auto& i : actuators) {

        // It is expected that the lower and upper limit vector sizes to be same
        long limit_size = i->get_upper_limit_vector().size();

        m_upper_limit.segment(r, r + limit_size) =
            i->get_upper_limit_vector();
        m_lower_limit.segment(r, r + limit_size) =
            i->get_lower_limit_vector();

        r += limit_size;
    }

}