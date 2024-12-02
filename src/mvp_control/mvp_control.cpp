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

    Author: Farhang Naderi
    Email: farhang.naderi@uri.edu;farhang.nba@gmail.com
    Year: 2024

    Copyright (C) 2024 Smart Ocean Systems Laboratory
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

auto MvpControl::get_control_allocation_matrix() ->
        decltype(m_control_allocation_matrix) {
    return m_control_allocation_matrix;
}

void MvpControl::set_thruster_articulation_vector(
        const decltype(m_thruster_vector)& vector) {
    m_thruster_vector = vector;
}

auto MvpControl::get_thruster_articulation_vector() ->
        decltype(m_thruster_vector){
    return m_thruster_vector;
}

void MvpControl::set_controller_frequency(
        const decltype(m_controller_frequency)& frequency) {
    m_controller_frequency = frequency;
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

void MvpControl::set_lower_angle(const decltype(m_lower_angle) &lower_angle) {
    m_lower_angle = lower_angle;
}

void MvpControl::set_upper_angle(const decltype(m_upper_angle) &upper_angle) {
    m_upper_angle = upper_angle;
}

void MvpControl::set_servo_speed(const decltype(m_servo_speed) &servo_speed) {
    m_servo_speed = servo_speed;
}

void MvpControl::set_current_angle(const int* m_thruster_index, double angle) {
    m_current_angles[*m_thruster_index] = angle;
}

// Setter for thruster directions
void MvpControl::set_thrust_direction(const std::vector<int>& thruster_directions) {
    // if (thruster_directions.size() == m_thruster_directions.size()) {
        m_thruster_directions = thruster_directions;  // Update if sizes match
    // } else {
    //     ROS_WARN("Size mismatch in thruster directions. Expected %zu, got %zu", m_thruster_directions.size(), thruster_directions.size());
    // }
}


// Getter for thruster directions
std::vector<int> MvpControl::get_thrust_direction() const {
    return m_thruster_directions;  // Return the stored thruster directions
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
    if (f_optimize_thrust(f, u))
    {
        return true;
    }
    else
    {
        ROS_WARN("Optimization of thrust failed!");
    }
    return false;
}

bool MvpControl::f_calculate_pid(Eigen::VectorXd *u, double dt)
{
    return m_pid->calculate(u, m_desired_state, m_system_state, dt);
}


bool MvpControl::f_optimize_thrust(Eigen::VectorXd *t, Eigen::VectorXd u) {
    // static bool is_initialized = false;
    // if (!is_initialized) {
        // Initialize current angles if not already done
        //m_current_angles.resize(m_thruster_vector.size(), 0.0);
        m_current_angles.resize(m_thruster_vector.size());
        std::vector<int> thruster_direction_action = get_thrust_direction();
        // printf("thruster_direction_action size: %zu\n", thruster_direction_action.size());
        // is_initialized = true;
    // }

    // Static variable to store the previous thruster directions
    static std::vector<int> prev_thruster_direction_action;

    // // Detect changes in thruster directions
    // bool direction_changed = false;
    // if (prev_thruster_direction_action.empty()) {
    //     // First call, initialize the previous directions
    //     prev_thruster_direction_action = thruster_direction_action;
    // } else if (prev_thruster_direction_action.size() != thruster_direction_action.size()) {
    //     // Size changed, so direction has changed
    //     direction_changed = true;
    // } else {
    //     // Compare each element
    //     for (size_t i = 0; i < thruster_direction_action.size(); ++i) {
    //         if (thruster_direction_action[i] != prev_thruster_direction_action[i]) {
    //             direction_changed = true;
    //             break;
    //         }
    //     }
    // }

    // Allocate and initialize control matrices and vectors
    Eigen::MatrixXd T(m_controlled_freedoms.size(), m_control_allocation_matrix.cols());
    Eigen::VectorXd U(m_controlled_freedoms.size());

    // Scoped lock for thread safety when accessing shared resources
    {
        std::scoped_lock lock(m_allocation_matrix_lock, m_controlled_freedoms_lock);
        for (int i = 0; i < m_controlled_freedoms.size(); ++i) {
            int idx = m_controlled_freedoms.at(i);
            if (idx < 0 || idx >= m_control_allocation_matrix.rows() || idx >= u.size()) {
                ROS_ERROR_STREAM("Index out of bounds when accessing controlled freedoms: " << idx);
                return false;
            }
            T.row(i) = m_control_allocation_matrix.row(idx);
            U(i) = u(idx);
        }
    }

    // Prepare data for the quadratic solver
    Eigen::MatrixXd Q = 2 * T.transpose() * T;
    Eigen::VectorXd c = -2 * T.transpose() * U;

    // Calculate the time step
    double deltaT = 1.0 / m_controller_frequency;

    // Calculate the number of pairs and singles in the thruster vector
    int pair_count = 0;
    int single_count = 0;
    for (size_t i = 0; i < m_thruster_vector.size(); ++i) {
        if (i + 1 < m_thruster_vector.size() &&
            m_thruster_vector[i] == ARTICULATED_THRUSTER_X &&
            m_thruster_vector[i + 1] == ARTICULATED_THRUSTER_Y) {
            pair_count++;
            i++; // Skip the next element since it forms a pair
        } else {
            single_count++;
        }
    }

    // Calculate the number of constraints 
    int kNumConstraints = NUM_CONSTRAINTS_PER_PAIR * pair_count + NUM_CONSTRAINTS_PER_SINGLE * single_count;
    int kNumVariables = m_control_allocation_matrix.cols();

    // Initialize thruster direction vector

    //std::vector<int> thruster_direction_action = get_thrust_direction();

    if (thruster_direction_action.size() != m_thruster_vector.size()) {
        thruster_direction_action.resize(m_thruster_vector.size(), 1); // Default to 1
    }
    // Ensure that each direction is valid, or default to Positive Force
    for (size_t i = 0; i < m_thruster_vector.size(); ++i) {
        if (thruster_direction_action[i] != 1 && thruster_direction_action[i] != -1) {
            thruster_direction_action[i] = 1;
        }
    }

   // Step 1: Calculate the total number of elements in the bounds vectors
    int total_elements = 0;
    for (size_t i = 0; i < m_thruster_vector.size(); ++i) {
        int thruster_value = m_thruster_vector[i];
        if (thruster_value == FIXED_THRUSTER) {
            total_elements += 1;
        }
        else if (thruster_value == ARTICULATED_THRUSTER_Y) {
            total_elements += 2;
        }
        else {
            total_elements += 3;
        }
    }

    // Step 2: Initialize new upper and lower limit vectors
    Eigen::VectorXi m_adjusted_upper_limit(total_elements);
    Eigen::VectorXi m_adjusted_lower_limit(total_elements);

    // Step 3: Populate the Eigen vectors
    int current_index = 0;
    for (size_t i = 0; i < m_thruster_vector.size(); ++i) {
        int thruster_value = m_thruster_vector[i];
        int upper = m_upper_limit[i];
        int lower = m_lower_limit[i];

        if (thruster_value == FIXED_THRUSTER) {
            // Non-articulated thruster
            m_adjusted_upper_limit(current_index) = upper;
            m_adjusted_lower_limit(current_index) = lower;
            current_index++;
        }
        else if (thruster_value == ARTICULATED_THRUSTER_X) {
            m_adjusted_upper_limit(current_index) = upper;
            m_adjusted_lower_limit(current_index) = lower;
            current_index++;
        }
        else if (thruster_value == ARTICULATED_THRUSTER_Y) {
            m_adjusted_upper_limit(current_index) = upper;
            m_adjusted_lower_limit(current_index) = lower;
            current_index++;
            m_adjusted_upper_limit(current_index) = upper;
            m_adjusted_lower_limit(current_index) = lower;
            current_index++;
        }
    }

    // Setup OSQP solver instance
    osqp::OsqpInstance qp_instance;
    qp_instance.objective_matrix = Q.sparseView();
    qp_instance.objective_vector = c;
    qp_instance.lower_bounds.resize(kNumConstraints);
    qp_instance.upper_bounds.resize(kNumConstraints);

    Eigen::SparseMatrix<double> A_sparse(kNumConstraints, kNumVariables);
    A_sparse.setZero(); // Set all constraint matrix values to 0
    std::vector<Eigen::Triplet<double>> A_triplets;

    int j = 0;  // Row counter

    // Construct constraint matrix and bounds
    for (size_t i = 0; i < m_thruster_vector.size(); ++i) {
        int thruster_setting = static_cast<int>(m_thruster_vector[i]);

            switch (thruster_setting) {
                case FIXED_THRUSTER: {
                    // Assign specific row index for thrust force constraint
                    int thrustRow = j;

                    // Non-articulated thruster constraints
                    A_triplets.emplace_back(thrustRow, i, 1.0);
                    qp_instance.lower_bounds[thrustRow] = m_adjusted_lower_limit[thrustRow];
                    qp_instance.upper_bounds[thrustRow] = m_adjusted_upper_limit[thrustRow];
                    
                    j += 1; // Move to the next set of constraints
                    break;
                }

                case ARTICULATED_THRUSTER_X: {
                    // Assign specific row indices for articulated thruster constraints
                    int thrustRow = j;
                    int angleUpperRow = j + 1;
                    int angleLowerRow = j + 2;

                    if (thruster_direction_action[i] == 1) {

                        // Compute alpha_u and alpha_l
                        double alpha_u = std::min(m_servo_speed[i] * deltaT, m_upper_angle[i] - m_current_angles[i]);
                        double alpha_l = std::max(-m_servo_speed[i] * deltaT, m_lower_angle[i] - m_current_angles[i]);
                        
                        // Compute force_coefficient
                        double force_coefficient = std::min(abs(std::cos(alpha_u)), abs(std::cos(alpha_l)));

                        // Add thrust constraint
                        A_triplets.emplace_back(thrustRow, i, 1.0);
                        qp_instance.lower_bounds[thrustRow] = 0;
                        qp_instance.upper_bounds[thrustRow] = m_adjusted_upper_limit[thrustRow] * force_coefficient;

                        // Add angleUpperRow constraint
                        A_triplets.emplace_back(angleUpperRow, i, std::tan(alpha_u));
                        A_triplets.emplace_back(angleUpperRow, i + 1, -1.0);
                        qp_instance.lower_bounds[angleUpperRow] = 0;
                        qp_instance.upper_bounds[angleUpperRow] = kInfinity;

                        // Add angleLowerRow constraint
                        A_triplets.emplace_back(angleLowerRow, i, std::tan(alpha_l));
                        A_triplets.emplace_back(angleLowerRow, i + 1, -1.0);
                        qp_instance.lower_bounds[angleLowerRow] = -kInfinity;
                        qp_instance.upper_bounds[angleLowerRow] = 0;

                        // Update the constraint row index
                        j += 3; // Jumping the constraint rows

                    } else if (thruster_direction_action[i] == -1) {

                        // Negative thrust direction
                        A_triplets.emplace_back(j, i, 1.0);
                        A_triplets.emplace_back(angleUpperRow, i, tan(-std::min(m_servo_speed[i] * deltaT , m_upper_angle[i] - m_current_angles[i])));
                        A_triplets.emplace_back(angleLowerRow, i, tan(std::max(-m_servo_speed[i] * deltaT , m_lower_angle[i] - m_current_angles[i])));
                        A_triplets.emplace_back(angleUpperRow, i + 1, 1.0);
                        A_triplets.emplace_back(angleLowerRow, i + 1, -1.0);
                        qp_instance.lower_bounds[j] = m_adjusted_lower_limit[j] * std::cos(m_servo_speed[i] * deltaT);
                        qp_instance.upper_bounds[j] = 0;
                        qp_instance.lower_bounds[angleUpperRow] = 0;
                        qp_instance.upper_bounds[angleUpperRow] = kInfinity;
                        qp_instance.lower_bounds[angleLowerRow] = 0;
                        qp_instance.upper_bounds[angleLowerRow] = kInfinity;
                        j += 3; //jumping the constraint rows
                    } else {
                        ROS_ERROR("Thruster direction is not set!");
                        return false;
                    }
                    break;
                }

                case ARTICULATED_THRUSTER_Y:
                    // Specific handling for ARTICULATED_THRUSTER_Y is not required
                    break;

                default:
                    ROS_ERROR_STREAM("Unexpected thruster setting: " << thruster_setting);
                    return false;
            }
        }   

    // Populate constraint matrix
    A_sparse.setFromTriplets(A_triplets.begin(), A_triplets.end());
    qp_instance.constraint_matrix = A_sparse;

    // Initialize OSQP solver
    osqp::OsqpSolver solver;
    osqp::OsqpSettings solver_settings;

    // **Customize OSQP Settings Here**
    solver_settings.verbose = false; // Disable verbose output
    solver_settings.eps_abs = 1e-4;  // Set absolute tolerance
    solver_settings.eps_rel = 1e-4;  // Set relative tolerance
    solver_settings.eps_prim_inf = 1e-4; // Primal infeasibility tolerance
    solver_settings.eps_dual_inf = 1e-4; // Dual infeasibility tolerance
    solver_settings.max_iter = 10000;    // Set maximum iterations
    solver_settings.scaling = false;      // Enable automatic scaling

    // **End of Customized Settings**

    auto status = solver.Init(qp_instance, solver_settings);

    if (!status.ok()) {
        ROS_ERROR("OSQP solver initialization failed.");
        return false;
    }

    // Solve the quadratic programming problem
    osqp::OsqpExitCode exitCode = solver.Solve();

    // Handle solver exit codes
    switch (exitCode) {
        case osqp::OsqpExitCode::kOptimal:
            *t = solver.primal_solution();
            return true;
        case osqp::OsqpExitCode::kPrimalInfeasible:
            ROS_ERROR("The problem is primal infeasible.");
            break;
        case osqp::OsqpExitCode::kDualInfeasible:
            ROS_ERROR("The problem is dual infeasible.");
            break;
        case osqp::OsqpExitCode::kOptimalInaccurate:
            ROS_ERROR("The optimal solution is inaccurate.");
            break;
        case osqp::OsqpExitCode::kPrimalInfeasibleInaccurate:
            ROS_ERROR("The problem is primal infeasible and the solution is inaccurate.");
            break;
        case osqp::OsqpExitCode::kDualInfeasibleInaccurate:
            ROS_ERROR("The problem is dual infeasible and the solution is inaccurate.");
            break;
        case osqp::OsqpExitCode::kMaxIterations:
            ROS_ERROR("The maximum number of iterations has been reached.");
            break;
        case osqp::OsqpExitCode::kInterrupted:
            ROS_ERROR("The optimization was interrupted.");
            break;
        case osqp::OsqpExitCode::kTimeLimitReached:
            ROS_ERROR("The time limit was reached before a solution was found.");
            break;
        case osqp::OsqpExitCode::kNonConvex:
            ROS_ERROR("The problem is non-convex.");
            break;
        case osqp::OsqpExitCode::kUnknown:
        default:
            ROS_ERROR("An unknown error occurred.");
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

        // auto d = (fmod(desired(i) + M_PI, 2*M_PI) - M_PI);
        // auto c = (fmod(current(i) + M_PI, 2*M_PI) - M_PI);

        // auto t = d - c;
        // double diff = (fmod(t + M_PI, 2*M_PI) - M_PI);
        // error(i) = diff < -M_PI ? diff + 2*M_PI : diff;

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