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

#include "mimo_pid.h"
#include "exception.hpp"

using namespace ctrl;

MimoPID::MimoPID() : m_error_function(nullptr) , m_dt_i(10000){
     m_i.setZero();
}

bool MimoPID::calculate(Eigen::VectorXd* u, const Eigen::ArrayXd& desired, const Eigen::ArrayXd& current, double dt) {

    if(m_error_function == nullptr) {
        throw control_exception("error function is not defined for MIMO pid.");
    }

    Eigen::ArrayXd error = m_error_function(desired, current);

    if(m_i.size() == 0) {
        m_i.resize(error.size());
        // printf("errorsize = %d\r\n", error.size());
    }

    // Proportional term
    Eigen::ArrayXd p = m_kp * error;
    

    Eigen::ArrayXd delta_i;
    delta_i = m_ki * (error * dt);

    // Derivation term
    if(!m_pe.data()) {
        m_pe = Eigen::VectorXd::Zero(error.size());
        return false;
    }

    Eigen::ArrayXd d = m_kd * ((error - m_pe) / dt);

    m_pe = error;

    Eigen::ArrayXd pid_sum = p + m_i + d;
    
    //saturation
    pid_sum = (pid_sum > m_pid_max).select(m_pid_max, pid_sum);
    pid_sum = (pid_sum < m_pid_min).select(m_pid_min, pid_sum);

    //clamping only update the integral when not saturated
    m_i = (pid_sum.array() > m_pid_min.array() && pid_sum.array() < m_pid_max.array())
           .select(m_i + delta_i, m_i);
    // *u = p + m_i + d;
    *u = pid_sum;

    // for(int i = 0; i<m_i.size(); i++)
    // {
    //     printf("integration[%d]=%lf, total =%lf\r\n", i, m_i[i], pid_sum[i]);

    // }

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

void MimoPID::set_m_i(const decltype(m_i) &new_m_i)
{
    m_i = new_m_i;
}

auto MimoPID::get_m_i() -> decltype(m_i) {
    return m_i;
}

auto MimoPID::get_error_function() -> decltype(m_error_function) {
    return m_error_function;
}

void MimoPID::set_error_function(const decltype(m_error_function) &func) {
    m_error_function = func;
}