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

#include "thruster_ros.h"

#include "utility"
#include "exception.hpp"
#include "mvp_control/dictionary.h"

using namespace ctrl;

ThrusterROS::ThrusterROS() 
        : m_nh() ,
        m_pnh("~")
{
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

auto ThrusterROS::get_servo_command_topic_id() -> decltype(m_servo_command_topic_id) {
    return m_servo_command_topic_id;
}

void ThrusterROS::set_servo_command_topic_id(const decltype(m_servo_command_topic_id) &topic_id) {
    m_servo_command_topic_id = topic_id;
}
auto ThrusterROS::get_joint_state_topic_id() -> decltype(m_joint_state_topic_id) {
    return m_joint_state_topic_id;
}

void ThrusterROS::set_joint_state_topic_id(const decltype(m_joint_state_topic_id) &topic_id) {
    m_joint_state_topic_id = topic_id;
}

auto ThrusterROS::get_joint_state_desired_topic_id() -> decltype(m_joint_state_desired_topic_id) {
    return m_joint_state_desired_topic_id;
}

void ThrusterROS::set_joint_state_desired_topic_id(const decltype(m_joint_state_desired_topic_id) &topic_id) {
    m_joint_state_desired_topic_id = topic_id;
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

void ThrusterROS::set_servo_joints(const std::vector<std::string>& joints) {
    servo_joints = joints;
}

std::vector<std::string> ThrusterROS::get_servo_joints() const {
    return servo_joints;
}

int ThrusterROS::get_is_articulated() const {
    return is_articulated;
}
void ThrusterROS::set_is_articulated(int value) {
    is_articulated = value;
}

const std::vector<double>& ThrusterROS::getServoSpeeds() const {
    return servo_speeds;
}

void ThrusterROS::setServoSpeeds(const std::vector<double>& speeds) {
    servo_speeds = speeds;
}

void ThrusterROS::initialize() {

    if(!m_thrust_command_topic_id.empty()) {
        m_thrust_publisher = m_nh.advertise<std_msgs::Float64>(
            m_thrust_command_topic_id, 100);
    } else {
        throw control_ros_exception("empty command topic name");
    }

    if(!m_thrust_command_topic_id.empty()) {
         m_force_publisher = m_nh.advertise<std_msgs::Float64>(
             m_thrust_force_topic_id, 100);
    } else {
        throw control_ros_exception("empty force topic name");
    }

    if (!m_joint_state_desired_topic_id.empty()) {
        m_joint_state_publisher = m_nh.advertise<sensor_msgs::JointState>(
            m_joint_state_desired_topic_id, 20);
    } else {
        throw control_ros_exception("empty joint state topic name");
    }    

    // Initialization for the servo command topic
    if (!m_servo_command_topic_id.empty()) {
        m_servo_command_publisher = m_nh.advertise<std_msgs::Float64>(m_servo_command_topic_id, 100);
    } else {
        ROS_WARN_STREAM("Servo command topic name is empty, assuming the thruster is not articulated.");
    }

}

auto ThrusterROS::get_link_id() -> decltype(m_link_id) {
    return m_link_id;
}

void ThrusterROS::set_link_id(const decltype(m_link_id)& link_id) {
    m_link_id = link_id;
}

auto ThrusterROS::get_servo_link_id() -> decltype(m_servo_link_id) {
    return m_servo_link_id;
}

void ThrusterROS::set_servo_link_id(const decltype(m_servo_link_id)& servo_link_id) {
    m_servo_link_id = servo_link_id;
}

auto ThrusterROS::get_frame_id() -> decltype(m_frame_id) {
    return m_frame_id;
}

void ThrusterROS::set_frame_id(const decltype(m_frame_id)& frame_id) {
    m_frame_id = frame_id;
}

void ThrusterROS::command(double cmd) {
    std_msgs::Float64 msg;
    msg.data = cmd;
    m_thrust_publisher.publish(msg);
}

void ThrusterROS::servo_joint_command(double cmd) {
    std_msgs::Float64 msg;
    msg.data = cmd;
    m_servo_command_publisher.publish(msg);
}

auto ThrusterROS::get_poly_solver() -> decltype(m_poly_solver) {
    return m_poly_solver;
}

void ThrusterROS::set_poly_solver(decltype(m_poly_solver) solver) {
    m_poly_solver = std::move(solver);
}

void ThrusterROS::set_servo_coeff(const std::vector<double>& coeff) {
    servo_coeff_ = coeff;
}

const std::vector<double>& ThrusterROS::get_servo_coeff() const {
    return servo_coeff_;
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

// bool ThrusterROS::request_joint_angles(const std::string& joint_name, double requested_angle) {

//     if (!m_joint_state_publisher) {
//         ROS_ERROR("Joint state publisher is not initialized!");
//         return false;
//     }

//     sensor_msgs::JointState joint_state_msg;
//     // joint_state_msg.header.stamp = ros::Time::now(); 
//     joint_state_msg.name.push_back(joint_name);     
//     joint_state_msg.position.push_back(requested_angle);

//     m_joint_state_publisher.publish(joint_state_msg); 

//     servo_joint_command(normalize_angle(requested_angle));

//     return true;
// }

bool ThrusterROS::request_joint_angles(const std::vector<std::string>& joint_names, const std::vector<double>& requested_angles) {

    if (!m_joint_state_publisher) {
        ROS_ERROR("Joint state publisher is not initialized!");
        return false;
    }

    if (joint_names.size() != requested_angles.size()) {
        ROS_ERROR("Mismatch between joint names and angles size!");
        return false;
    }

    sensor_msgs::JointState joint_state_msg;
    joint_state_msg.header.stamp = ros::Time::now(); 

    // Add all joint names and positions to the message
    joint_state_msg.name = joint_names;     
    joint_state_msg.position = requested_angles;

    // Publish all joint states at once
    m_joint_state_publisher.publish(joint_state_msg);

    return true;
}

double ThrusterROS::normalize_angle(double angle) const {
    // Check that the polynomial has the expected four coefficients (slope, offset1, offset2, center)
    if (servo_coeff_.size() != 2) {
        throw std::runtime_error("Servo polynomial must have exactly 4 coefficients.");
    }

    double slope = servo_coeff_[0];
    double offset = servo_coeff_[1];

    // Apply the linear transformation with the chosen offset
    double pwm = slope * angle + offset;

    // Clamp the pwm to the range -1 to 1
    return std::max(std::min(pwm, 1.0), -1.0);
}