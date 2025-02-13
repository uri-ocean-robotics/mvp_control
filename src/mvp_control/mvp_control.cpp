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

#include "mvp_control/mvp_control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "mvp_control/dictionary.hpp"
#include "mvp_control/exception.hpp"
#include "functional"

using namespace ctrl;

MvpControl::MvpControl() {

    /**
     * Initialize the error state
     */
    m_error_state = Eigen::VectorXd(CONTROLLABLE_DOF_LENGTH);
    m_error_state.setZero();

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

void MvpControl::set_direction_cost_matrix(
        const decltype(m_direction_cost_matrix)& matrix) {
    m_direction_cost_matrix = matrix;
}

void MvpControl::set_direction_cost_matrix_c(
        const decltype(m_direction_cost_matrix_c)& matrix) {
    m_direction_cost_matrix_c = matrix;
}

auto MvpControl::get_control_allocation_matrix() ->
        decltype(m_control_allocation_matrix) {
    return m_control_allocation_matrix;
}

auto MvpControl::get_direction_cost_matrix() ->
        decltype(m_direction_cost_matrix) {
    return m_direction_cost_matrix;
}

auto MvpControl::get_direction_cost_matrix_c() ->
        decltype(m_direction_cost_matrix_c) {
    return m_direction_cost_matrix_c;
}


void MvpControl::set_total_force_cost_factor(
        const decltype(m_total_force_cost_factor)& factor) {
    m_total_force_cost_factor = factor;
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

void MvpControl::set_constraint_matrix(const decltype(m_constrain_matrix) &matrix)
{
    m_constrain_matrix = matrix;
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
    // std::cout<<"control_effort: "<< u.transpose()<<std::endl;

    /**
     * Below code computes the forces that later will be requested from the
     * thrusters. Values in the force vector are not thruster set points yet.
     */
    if(f_optimize_thrust(f, u)) {
        // std::cout<<"f: "<< f<<std::endl;
        return true;
    } else {
        // todo: create a warning
        printf("optimize_thruster false return\r\n");
        return false;
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
    // Eigen::MatrixXd B2(
    //     m_controlled_freedoms.size(),
    //     m_control_allocation_matrix.cols()
    // );
    Eigen::MatrixXd B2(
        m_control_allocation_matrix.cols(),
        m_control_allocation_matrix.cols()
    );
    B2.setIdentity();
    // // Control matrix
    Eigen::VectorXd U(m_controlled_freedoms.size());
    Eigen::VectorXd B1 =m_direction_cost_matrix_c;

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
            T.row(i) = m_control_allocation_matrix.row(m_controlled_freedoms.at(i));
            // B2.row(i) = m_direction_cost_matrix.row(m_controlled_freedoms.at(i));
            
            U(i) = u(m_controlled_freedoms.at(i));
        }
    }

    /**
     * Now we are preparing data for quadratic solver to consume.
     */
    
    
    // std::cout << "T size: " << T.rows() << " x " << T.cols() << std::endl;

    // std::cout << "B2 size: " << B2.rows() << " x " << B2.cols() << std::endl;

    // std::cout << "U size: " << U.size() << std::endl;

    // std::cout << "B1 size: " << B1.size() << std::endl;

    // Q -> objective matrix
    Eigen::VectorXd ones = Eigen::VectorXd::Ones(m_control_allocation_matrix.cols());

    Eigen::MatrixXd Q = 2 * T.transpose() * T + 2*m_total_force_cost_factor*B2;//B2.transpose()*B2;//- 2*m_direction_lambda*B2.transpose()*B2;
    // Eigen::MatrixXd Q = 2 * T.transpose() * T + m_total_force_cost_factor*ones*ones.transpose();//- 2*m_direction_lambda*B2.transpose()*B2;

    // c -> objective vector
    Eigen::VectorXd c = (-2 * (U.transpose() * T)).transpose(); // + m_direction_lambda*B1;

    // std::cout << "c size: " << c.size() << std::endl;


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

    // qp_instance.constraint_matrix =
        // Eigen::SparseMatrix<double>(Q.cols(),Q.cols());
    qp_instance.constraint_matrix = m_constrain_matrix;
    // qp_instance.constraint_matrix.setIdentity();

    osqp::OsqpSolver solver;
    osqp::OsqpSettings settings;

    settings.verbose = false;
    // **Customize OSQP Settings Here**
    settings.eps_abs = 1e-3;  // Set absolute tolerance
    settings.eps_rel = 1e-3;  // Set relative tolerance
    settings.eps_prim_inf = 1e-4; // Primal infeasibility tolerance
    settings.eps_dual_inf = 1e-4; // Dual infeasibility tolerance
    settings.max_iter = 3*1e5;    // Set maximum iterations
    settings.scaling = false;      // Enable automatic scaling

    auto status = solver.Init(qp_instance, settings);

    if(not status.ok()) {
        return false;
    }

    osqp::OsqpExitCode exitCode = solver.Solve();
    // printf("final cost = %lf\r\n", solver.objective_value());

    switch (exitCode) {
        case osqp::OsqpExitCode::kOptimal: {
            *t = solver.primal_solution();
            // Eigen::VectorXd J;
            // Eigen::MatrixXd J1;
            // J1 = T *solver.primal_solution()-U;
            // // J = J1.transpose() * J1;
            // std::cout<<"J1: "<< J1.transpose()<<std::endl;
            return true;
            break;
        }
        case osqp::OsqpExitCode::kPrimalInfeasible:
            printf("Error: PrimalInfeasible\r\n");
            return false;
            break;
        case osqp::OsqpExitCode::kDualInfeasible:
            printf("Error: kDualInfeasible\r\n");
            return false;
            break;
        case osqp::OsqpExitCode::kOptimalInaccurate:
            printf("Error: kOptimalInaccurate\r\n");
            return false;
            break;
        case osqp::OsqpExitCode::kPrimalInfeasibleInaccurate:
            printf("Error: kPrimalInfeasibleInaccurate\r\n");
            return false;
            break;
        case osqp::OsqpExitCode::kDualInfeasibleInaccurate:
            printf("Error: kDualInfeasibleInaccurate\r\n");
            return false;
            break;
        case osqp::OsqpExitCode::kMaxIterations:
            printf("Error: kMaxIterations\r\n");
            return false;
            break;
        case osqp::OsqpExitCode::kInterrupted:
            printf("Error: kInterrupted\r\n");
            return false;
            break;
        case osqp::OsqpExitCode::kTimeLimitReached:
            printf("Error: kTimeLimitReached\r\n");
            return false;
            break;
        case osqp::OsqpExitCode::kNonConvex:
            printf("Error: kNonConvex\r\n");
            return false;
            break;
        case osqp::OsqpExitCode::kUnknown:
            printf("Error: Unknow\r\n");
            return false;
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
                         DOF::P, DOF::Q, DOF::R}) {

        // todo: wrap2pi implementation

        //wrap desired and current in to -pi to pi
        auto d = (fmod(desired(i) + std::copysign(M_PI, desired(i)), 2*M_PI) 
                - std::copysign(M_PI, desired(i)));
        auto c = (fmod(current(i) + std::copysign(M_PI,current(i)), 2*M_PI) 
                - std::copysign(M_PI,current(i)));
        auto t = d - c;
        double diff = (fmod(t + std::copysign(M_PI,t), 2*M_PI) 
                - std::copysign(M_PI,t));
        error(i) = diff;
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