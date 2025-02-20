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

#include "mvp_control/vector_thruster_ros.hpp"

#include "utility"
#include "mvp_control/exception.hpp"
#include "mvp_control/dictionary.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace ctrl;

VectorThrusterROS::VectorThrusterROS(){
    m_poly_solver.reset(new PolynomialSolver());
}

VectorThrusterROS::VectorThrusterROS(std::string id, std::string topic_id, std::string servo_topic_id, Eigen::VectorXd contribution_vector) :
        m_id(std::move(id)),
        m_thrust_command_topic_id(std::move(topic_id)),
        m_thruster_servo_topic_id(std::move(servo_topic_id)),
        m_contribution_vector(std::move(contribution_vector))
{

    // m_thrust_publisher = this->create_publisher<std_msgs::msg::Float64>(m_thrust_command_topic_id, 10);

    m_poly_solver.reset(new PolynomialSolver());
}

void VectorThrusterROS::initialize() {
    if(!m_thrust_command_topic_id.empty()) {
        // m_thrust_publisher = this->create_publisher<std_msgs::msg::Float64>(m_thrust_command_topic_id, 100);
    } else {
        throw control_ros_exception("empty topic name");
    }
    if(!m_thrust_force_topic_id.empty()) {
        //  m_force_publisher = this->create_publisher<std_msgs::msg::Float64>(m_thrust_force_topic_id, 100);
    } else {
        throw control_ros_exception("empty topic name");
    }
}

void VectorThrusterROS::set_link_id(const decltype(m_link_id)& link_id) {
    m_link_id = link_id;
}

auto VectorThrusterROS::get_link_id() -> decltype(m_link_id) {
    return m_link_id;
}

auto VectorThrusterROS::get_thrust_command_topic_id() -> decltype(m_thrust_command_topic_id) {
    return m_thrust_command_topic_id;
}

void VectorThrusterROS::set_thrust_command_topic_id(const decltype(m_thrust_command_topic_id) &topic_id) {
    m_thrust_command_topic_id = topic_id;
}

auto VectorThrusterROS::get_thrust_force_topic_id() -> decltype(this->m_thrust_force_topic_id) {
    return m_thrust_force_topic_id;
}

void VectorThrusterROS::set_thrust_force_topic_id(const decltype(m_thrust_force_topic_id) &topic_id) {
    m_thrust_force_topic_id = topic_id;
}

auto VectorThrusterROS::get_thruster_servo_topic_id() -> decltype(this->m_thruster_servo_topic_id) {
    return m_thruster_servo_topic_id;
}

void VectorThrusterROS::set_thruster_servo_topic_id(const decltype(m_thruster_servo_topic_id) &topic_id) {
    m_thruster_servo_topic_id = topic_id;
}

auto VectorThrusterROS::get_thruster_servo_joint_id() -> decltype(this->m_thruster_servo_joint_id) {
    return m_thruster_servo_joint_id;
}

void VectorThrusterROS::set_thruster_servo_joint_id(const decltype(m_thruster_servo_joint_id) &joint_id) {
    m_thruster_servo_joint_id = joint_id;
}

auto VectorThrusterROS::get_thruster_servo_angle() -> decltype(this->m_servo_angle)
{
return m_servo_angle;
}
        
void VectorThrusterROS::set_thruster_servo_angle(const decltype(m_servo_angle) &servo_angle)
{
    m_servo_angle = servo_angle;
}

auto VectorThrusterROS::get_thruster_servo_speed() -> decltype(this->m_servo_speed)
{
return m_servo_speed;
}
        
void VectorThrusterROS::set_thruster_servo_speed(const decltype(m_servo_speed) &servo_speed)
{
    m_servo_speed = servo_speed;
}


auto VectorThrusterROS::get_thruster_direction() -> decltype(this->m_thruster_direction)
{
return m_thruster_direction;
}
        
void VectorThrusterROS::set_thruster_direction(const decltype(m_thruster_direction) &thruster_direction)
{
    m_thruster_direction = thruster_direction;
}


auto VectorThrusterROS::get_id() -> decltype(m_id) {
    return m_id;
}

void VectorThrusterROS::set_id(const decltype(m_id)& thruster_id) {
    m_id = thruster_id;
}

auto VectorThrusterROS::get_contribution_vector() -> decltype(m_contribution_vector) {
    return m_contribution_vector;
}

void VectorThrusterROS::set_contribution_vector(const decltype(m_contribution_vector)& contribution_vector) {
    m_contribution_vector = contribution_vector;
}

auto VectorThrusterROS::get_poly_solver() -> decltype(m_poly_solver) {
    return m_poly_solver;
}

void VectorThrusterROS::set_poly_solver(decltype(m_poly_solver) solver) {
    m_poly_solver = std::move(solver);
}


void VectorThrusterROS::set_thruster_auto_mode(decltype(m_auto_dir_mode) &mode) {
    m_auto_dir_mode = mode;
    if(mode){
        m_force_count = 3;
        m_constraint_count =8;
    }
    else{
        m_force_count = 2;
        m_constraint_count = 4;
    }

}

auto VectorThrusterROS::get_thruster_force_count() -> decltype(m_force_count){
    return m_force_count;
}

auto VectorThrusterROS::get_thruster_constraint_count() -> decltype(m_constraint_count)
{
    return m_constraint_count;
}


bool VectorThrusterROS::request_command(double fx, double fy, double current_angle, double &command, double &new_angle ) {

    std::vector<std::complex<double>> roots;
    // double N = m_thruster_direction * std::sqrt(std::pow(fx, 2) + std::pow(fy, 2));

    double sign_x = std::copysign(1.0, fx);

    double N = sign_x * std::sqrt(std::pow(fx, 2) + std::pow(fy, 2));
    // printf("%s:", m_thrust_command_topic_id.c_str());
    // printf(" force=%lf, %lf, %lf\r\n", fx, fy, N);
    //solve angle
    double delta_angle;
    if(fx == 0)
    {
        if(fy > 0){
            delta_angle = 1.5707;
        }
        else{
            delta_angle = -1.5707;
        }
    }
    else{
         delta_angle = std::atan(fy/fx);
    }

    // printf("angle = %lf, Fy =%lf\r\n", delta_angle, fy);
    new_angle = delta_angle + current_angle;

    // printf("angle =%lf, %lf, %lf\r\n", current_angle, new_angle, delta_angle);


    //solve force
    if(N > m_force_max) {
        N = m_force_max;
    } else if (N < m_force_min) {
        N = m_force_min;
    }

    if(!m_poly_solver->solve_for_y(roots, N)) {
        // ROS_WARN_STREAM("No feasible command found for force: " << N);
        // RCLCPP_WARN_STREAM(this->get_logger(), "No feasible command found for force: " << N);
        printf("####No feasible command found for force %lf\r\n", N);
        return false;
    }
    
    for(const auto& r : roots) {
        if(r.imag() != 0){
            continue;
        }
        if(r.real() >= 1 || r.real() < -1) {
            continue;
        }      
        command = r.real();
        break;
    }

    return true;
}
