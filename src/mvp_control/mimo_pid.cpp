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

#include "mvp_control/mimo_pid.hpp"
#include "mvp_control/exception.hpp"
#include <iostream>

using namespace ctrl;

MimoPID::MimoPID() : m_dt_i(10000), m_error_function(nullptr) {
    m_i(Eigen::ArrayXd(CONTROLLABLE_DOF_LENGTH));
    m_i.setZero();
}

bool MimoPID::calculate(Eigen::VectorXd* u, const Eigen::ArrayXd& desired, const Eigen::ArrayXd& current, double dt) {

    if(m_error_function == nullptr) {
        throw control_exception("error function is not defined for MIMO pid.");
    }

    Eigen::ArrayXd error = m_error_function(desired, current);
   
    
    if(m_i.size() == 0) {
        m_i.resize(error.size());
    }

    // Proportional term
    Eigen::ArrayXd p = m_kp * error;

    Eigen::ArrayXd delta_i;
    delta_i = m_ki * (error *dt);

    // Derivation term
    if(!m_pe.data()) {
        m_pe = Eigen::VectorXd::Zero(error.size());
        return false;
    }

    Eigen::ArrayXd d = m_kd * ((error - m_pe) / dt);

    m_pe = error;

    Eigen::ArrayXd pid_sum = p + m_i + d;

    pid_sum = (pid_sum > m_pid_max).select(m_pid_max, pid_sum);
    pid_sum = (pid_sum < m_pid_min).select(m_pid_min, pid_sum);

    m_i = (pid_sum.array() > m_pid_min.array() && pid_sum.array() < m_pid_max.array()).select(m_i+delta_i, m_i);
    // printf("----------------------\r\n");
    // printf("PID pitch: %lf, %lf,%lf, | all: %lf\r\n", p[4], m_i[4], d[4], pid_sum[4]);
    // printf("PID Yaw: %lf, %lf,%lf, | all: %lf\r\n", p[5], m_i[5], d[5], pid_sum[5]);
    // printf("PID Z: %lf, %lf,%lf, | all: %lf\r\n", p[2], m_i[2], d[2], pid_sum[2]);
    // printf("PID v: %lf, %lf,%lf, | all: %lf\r\n", p[7], m_i[7], d[7], pid_sum[7]);
    // printf("PID u: %lf, %lf,%lf, | all: %lf\r\n", p[6], m_i[6], d[6], pid_sum[6]);
    
    // *u = p + m_i + d;
    *u = pid_sum;

    // std::cout<<"error_p:"<< p <<std::endl;
    // std::cout<<"error_i:"<< m_i <<std::endl;
    // std::cout<<"error_d:"<< d <<std::endl;

    return true;
}

auto MimoPID::get_kp() -> decltype(m_kp) {
    return m_kp;
}

void MimoPID::set_kp(const decltype(m_kp) &gain) {
    m_kp = gain;
}

auto MimoPID::get_ki() -> decltype(m_ki) {
    return m_ki;
}

void MimoPID::set_ki(const decltype(m_ki) &gain) {
    m_ki = gain;
}

auto MimoPID::get_kd() -> decltype(m_kd) {
    return m_kd;
}

void MimoPID::set_kd(const decltype(m_kd) &gain) {
    m_kd = gain;
}

auto MimoPID::get_dt() const -> decltype(m_dt) {
    return m_dt;
}

void MimoPID::set_dt(const decltype(m_dt) &gain) {
    m_dt = gain;
}

auto MimoPID::get_dt_i() const -> decltype(m_dt_i) {
    return m_dt_i;
}

void MimoPID::set_dt_i(const decltype(m_dt_i) &gain) {
    m_dt_i = gain;
}

auto MimoPID::get_pid_max() -> decltype(m_pid_max) {
    return m_pid_max;
}

void MimoPID::set_pid_max(const decltype(m_pid_max) &gain) {
    m_pid_max= gain;
}

auto MimoPID::get_pid_min() -> decltype(m_pid_min) {
    return m_pid_min;
}

void MimoPID::set_pid_min(const decltype(m_pid_min) &gain) {
    m_pid_min= gain;
}

void MimoPID::set_m_i(const decltype(m_i) &new_m_i){
    m_i = new_m_i;
}

auto MimoPID::get_m_i()->decltype(m_i){
    return m_i;
}

auto MimoPID::get_error_function() -> decltype(m_error_function) {
    return m_error_function;
}

void MimoPID::set_error_function(const decltype(m_error_function) &func) {
    m_error_function = func;
}
