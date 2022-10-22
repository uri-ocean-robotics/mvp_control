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

#include "mvp_control.h"
#include "ros/ros.h"
#include "mvp_control/dictionary.h"
#include "exception.hpp"
#include "functional"

using namespace ctrl;

MvpControl::MvpControl() {

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
            &MvpControl::f_error_function,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

}

void MvpControl::set_control_allocation_matrix(
        const decltype(m_control_allocation_matrix)& matrix) {
    m_control_allocation_matrix = matrix;
}

auto MvpControl::get_control_allocation_matrix() ->
        decltype(m_control_allocation_matrix) {
    return m_control_allocation_matrix;
}

auto MvpControl::get_pid() -> decltype(m_pid) {
    return m_pid;
}

void MvpControl::set_pid(const MimoPID::Ptr &pid) {
    m_pid = pid;
}

auto MvpControl::get_system_state() -> decltype(m_system_state) {
    return m_system_state;
}

void MvpControl::set_system_state(
        const decltype(m_system_state) &system_state) {
    m_system_state = system_state;
}

auto MvpControl::get_desired_state() -> decltype(m_desired_state) {
    return m_desired_state;
}

void MvpControl::set_desired_state(
        const decltype(m_desired_state) &desired_state) {
    m_desired_state = desired_state;
}

void MvpControl::set_lower_limit(const decltype(m_lower_limit) &lower_limit) {
    m_lower_limit = lower_limit;
}

void MvpControl::set_upper_limit(const decltype(m_upper_limit) &upper_limit) {
    m_upper_limit = upper_limit;
}

bool MvpControl::calculate_needed_forces(Eigen::VectorXd *f, double dt) {

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
    if(f_optimize_thrust(f, u)) {
        return true;
    } else {
        // todo: create a warning
    }

    return false;
}

bool MvpControl::f_calculate_pid(Eigen::VectorXd *u, double dt) {
    return m_pid->calculate(u, m_desired_state, m_system_state, dt);
}

bool MvpControl::f_optimize_thrust(Eigen::VectorXd *t, Eigen::VectorXd u) {

    // Control allocation matrix
    Eigen::MatrixXd T(
        m_controlled_freedoms.size(),
        m_control_allocation_matrix.cols()
    );

    // Control matrix
    Eigen::VectorXd U(m_controlled_freedoms.size());

    {
        /**
         * Quadatic equation solving is probably the most time consuming part
         * of the program. During that time changes may happen. Most likely
         * change would happen on the 'm_controlled_freedoms' vector. This
         * vector updates when controllers mode change. Scoped lock is making
         * sure that those variables will not change while execution is inside
         * in this scope.
         */
        std::scoped_lock lock(m_allocation_matrix_lock,
                              m_controlled_freedoms_lock);
        /**
         * We are using a portion of the control allication matrix and the
         * control input. This is important for online mode updates.
         *
         */
        for (int i = 0; i < m_controlled_freedoms.size(); i++) {
            T.row(i) = m_control_allocation_matrix.row(
                m_controlled_freedoms.at(i));
            U(i) = u(m_controlled_freedoms.at(i));
        }
    }

    /**
     * Now we are preparing data for quadratic solver to consume.
     */

    // Q -> objective matrix
    Eigen::MatrixXd Q = 2 * T.transpose() * T;
    // c -> objective vector
    Eigen::VectorXd c = (-2 * (U.transpose() * T)).transpose();

    std::vector<Eigen::Triplet<double>> Q_triplets;
    for(int i = 0 ; i < Q.rows() ; i++) {
        for(int j = 0 ; j < Q.cols() ; j++) {
            Q_triplets.emplace_back(Eigen::Triplet<double>{i, j, Q(i,j)});
        }
    }


    // Creating a quadratic solver instance.
    osqp::OsqpInstance qp_instance;

    /**
     * Translating the objective matrix into sparse matrix. Eigen::SparseMatrix
     * is the data type that is consumed by quadratic solver.
     */
    Eigen::SparseMatrix<double> Q_sparse(Q.rows(), Q.cols());
    Q_sparse.setFromTriplets(Q_triplets.begin(), Q_triplets.end());
    qp_instance.objective_matrix = Q_sparse;

    qp_instance.objective_vector = c;

    qp_instance.lower_bounds.resize(m_lower_limit.size());
    qp_instance.lower_bounds << m_lower_limit;

    qp_instance.upper_bounds.resize(m_upper_limit.size());
    qp_instance.upper_bounds << m_upper_limit;

    qp_instance.constraint_matrix =
        Eigen::SparseMatrix<double>(Q.cols(),Q.cols());

    qp_instance.constraint_matrix.setIdentity();

    osqp::OsqpSolver solver;
    osqp::OsqpSettings settings;

    settings.verbose = false;

    auto status = solver.Init(qp_instance, settings);

    if(not status.ok()) {
        return false;
    }

    osqp::OsqpExitCode exitCode = solver.Solve();

    switch (exitCode) {
        case osqp::OsqpExitCode::kOptimal: {
            *t = solver.primal_solution();
            return true;
            break;
        }
        case osqp::OsqpExitCode::kPrimalInfeasible:
            break;
        case osqp::OsqpExitCode::kDualInfeasible:
            break;
        case osqp::OsqpExitCode::kOptimalInaccurate:
            break;
        case osqp::OsqpExitCode::kPrimalInfeasibleInaccurate:
            break;
        case osqp::OsqpExitCode::kDualInfeasibleInaccurate:
            break;
        case osqp::OsqpExitCode::kMaxIterations:
            break;
        case osqp::OsqpExitCode::kInterrupted:
            break;
        case osqp::OsqpExitCode::kTimeLimitReached:
            break;
        case osqp::OsqpExitCode::kNonConvex:
            break;
        case osqp::OsqpExitCode::kUnknown:
            break;
        default:
            break;
    }

    return false;
}

void MvpControl::set_controlled_freedoms(decltype(m_controlled_freedoms) f) {
    m_controlled_freedoms = f;
}

auto MvpControl::get_state_error() -> decltype(this->m_error_state) {
    return m_error_state;
}

Eigen::ArrayXd MvpControl::f_error_function(Eigen::ArrayXd desired,
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

void MvpControl::update_control_allocation_matrix(
        const decltype(m_control_allocation_matrix) &m) {
    std::scoped_lock lock(m_allocation_matrix_lock);
    m_control_allocation_matrix = m;
}

void MvpControl::update_freedoms(std::vector<int> freedoms) {
    std::scoped_lock lock(m_controlled_freedoms_lock);
    m_controlled_freedoms = std::move(freedoms);

}

void MvpControl::update_desired_state(
        const decltype(m_desired_state) &desired_state) {
    std::scoped_lock lock(m_desired_state_lock);
    m_desired_state = desired_state;
}