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

    m_i += m_ki * (error * dt);

    m_i = (m_i > m_i_max).select(m_i_max, m_i);
    m_i = (m_i < m_i_min).select(m_i_min, m_i);

    // Derivation term
    if(!m_pe.data()) {
        m_pe = Eigen::VectorXd::Zero(error.size());
        return false;
    }

    Eigen::ArrayXd d = m_kd * ((error - m_pe) / dt);

    m_pe = error;

    *u = p + m_i + d;

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

auto MimoPID::get_i_max() -> decltype(m_i_max) {
    return m_i_max;
}

void MimoPID::set_i_max(const decltype(m_i_max) &gain) {
    m_i_max= gain;
}

auto MimoPID::get_i_min() -> decltype(m_i_max) {
    return m_i_min;
}

void MimoPID::set_i_min(const decltype(m_i_min) &gain) {
    m_i_min= gain;
}

auto MimoPID::get_error_function() -> decltype(m_error_function) {
    return m_error_function;
}

void MimoPID::set_error_function(const decltype(m_error_function) &func) {
    m_error_function = func;
}
