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

#include "thruster_ros.h"

#include "utility"
#include "exception.hpp"
#include "mvp_control/dictionary.h"

using namespace ctrl;

ThrusterROS::ThrusterROS() {
    m_poly_solver.reset(new PolynomialSolver());
}

ThrusterROS::ThrusterROS(std::string id, std::string topic_id, Eigen::VectorXd contribution_vector) :
        m_id(std::move(id)),
        m_thrust_command_topic_id(std::move(topic_id)),
        m_contribution_vector(std::move(contribution_vector))
{

    m_thrust_publisher = m_nh.advertise<std_msgs::Float64>(
        m_thrust_command_topic_id, 10);

    m_poly_solver.reset(new PolynomialSolver());
}

auto ThrusterROS::get_thrust_command_topic_id() -> decltype(m_thrust_command_topic_id) {
    return m_thrust_command_topic_id;
}

void ThrusterROS::set_thrust_command_topic_id(const decltype(m_thrust_command_topic_id) &topic_id) {
    m_thrust_command_topic_id = topic_id;
}

auto ThrusterROS::get_thrust_force_topic_id() -> decltype(this->m_thrust_force_topic_id) {
    return m_thrust_force_topic_id;
}

void ThrusterROS::set_thrust_force_topic_id(const decltype(m_thrust_force_topic_id) &topic_id) {
    m_thrust_force_topic_id = topic_id;
}

auto ThrusterROS::get_id() -> decltype(m_id) {
    return m_id;
}

void ThrusterROS::set_id(const decltype(m_id)& thruster_id) {
    m_id = thruster_id;
}

auto ThrusterROS::get_contribution_vector() -> decltype(m_contribution_vector) {
    return m_contribution_vector;
}

void ThrusterROS::set_contribution_vector(const decltype(m_contribution_vector)& contribution_vector) {
    m_contribution_vector = contribution_vector;
}

void ThrusterROS::initialize() {

    if(!m_thrust_command_topic_id.empty()) {
        m_thrust_publisher = m_nh.advertise<std_msgs::Float64>(
            m_thrust_command_topic_id, 100);
    } else {
        throw control_ros_exception("empty topic name");
    }

    if(!m_thrust_command_topic_id.empty()) {
         m_force_publisher = m_nh.advertise<std_msgs::Float64>(
             m_thrust_force_topic_id, 100);
    } else {
        throw control_ros_exception("empty topic name");
    }
}

auto ThrusterROS::get_link_id() -> decltype(m_link_id) {
    return m_link_id;
}

void ThrusterROS::set_link_id(const decltype(m_link_id)& link_id) {
    m_link_id = link_id;
}

void ThrusterROS::command(double cmd) {
    std_msgs::Float64 msg;
    msg.data = cmd;
    m_thrust_publisher.publish(msg);
}

auto ThrusterROS::get_poly_solver() -> decltype(m_poly_solver) {
    return m_poly_solver;
}

void ThrusterROS::set_poly_solver(decltype(m_poly_solver) solver) {
    m_poly_solver = std::move(solver);
}

bool ThrusterROS::request_force(double N) {
    std::vector<std::complex<double>> roots;

    std_msgs::Float64  msg;
    msg.data = N;
    m_force_publisher.publish(msg);

    if(N > m_force_max) {
        N = m_force_max;
    } else if (N < m_force_min) {
        N = m_force_min;
    }

    if(!m_poly_solver->solve_for_y(roots, N)) {
        ROS_WARN_STREAM("No feasible command found for force: " << N);
        return false;
    }

    for(const auto& r : roots) {
        if(r.imag() != 0){
            continue;
        }

        if(r.real() >= 1 || r.real() < -1) {
            continue;
        }

        command(r.real());

        break;
    }

    return true;
}