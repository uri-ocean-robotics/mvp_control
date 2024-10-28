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

#pragma once

#include "vector"
#include "string"
#include "map"

#include "mvp_msgs/ControlModes.h"

namespace ctrl {

    static constexpr const char * CONF_DOF_X = "x";
    static constexpr const char * CONF_DOF_Y = "y";
    static constexpr const char * CONF_DOF_Z = "z";
    static constexpr const char * CONF_DOF_ROLL = "roll";
    static constexpr const char * CONF_DOF_PITCH = "pitch";
    static constexpr const char * CONF_DOF_YAW = "yaw";
    static constexpr const char * CONF_DOF_SURGE = "surge";
    static constexpr const char * CONF_DOF_SWAY = "sway";
    static constexpr const char * CONF_DOF_HEAVE = "heave";
    static constexpr const char * CONF_DOF_ROLL_RATE = "roll_rate";
    static constexpr const char * CONF_DOF_PITCH_RATE = "pitch_rate";
    static constexpr const char * CONF_DOF_YAW_RATE = "yaw_rate";

    static constexpr const char * CONF_THRUSTER_POLY = "thruster_polynomials";
    static constexpr const char* CONF_SERVO_POLY = "servo_coefficients";
    static constexpr const char * CONF_THRUST_COMMAND_TOPICS = "thruster_command_topics";
    static constexpr const char * CONF_SERVO_COMMAND_TOPICS = "servo_command_topics";
    static constexpr const char * CONF_SERVO_JOINT_TOPIC = "servo_joint_topic";
    static constexpr const char * CONF_SERVO_JOINT_TOPIC_DEFAULT = "/control/servos/joint_states";
    static constexpr const char * CONF_SERVO_JOINT_SETPOINT_TOPIC = "servo_joint_setpoint_topic";
    static constexpr const char * CONF_THRUSTER_FORCE_TOPICS = "thruster_force_topics";
    static constexpr const char * CONF_THRUSTER_IDS = "thruster_ids";
    static constexpr const char * CONF_SERVO_IDS = "servo_ids";
    static constexpr const char * CONF_THRUSTER_SERVO_JOINTS = "thruster_servo_joints";
    static constexpr const char * CONF_THRUSTER_SERVO_SPEEDS = "thruster_servo_speeds";

    static constexpr const char * CONF_THRUSTER_LIMITS = "thruster_limits";
    static constexpr const char * CONF_THRUSTER_MAX = "max";
    static constexpr const char * CONF_THRUSTER_MIN = "min";

    static constexpr const char * CONF_GENERATOR_TYPE = "generator_type";
    static constexpr const char * CONF_GENERATOR_TYPE_OPT_TF = "tf";
    static constexpr const char * CONF_GENERATOR_TYPE_OPT_USER = "user";

    static constexpr const char * CONF_TF_PREFIX = "tf_prefix";
    static constexpr const char * CONF_TF_PREFIX_DEFAULT = "";
    static constexpr const char * CONF_CG_LINK = "cg_link";
    static constexpr const char * CONF_CG_LINK_DEFAULT = "cg_link";
    static constexpr const char * CONF_WORLD_LINK = "world_link";
    static constexpr const char * CONF_WORLD_LINK_DEFAULT = "world";
    static constexpr const char * CONF_ODOMETRY_SOURCE = "odometry_source";
    static constexpr const char * CONF_ODOMETRY_SOURCE_DEFAULT = "odometry";
    static constexpr const char * CONF_CONTROL_MODES = "control_modes";
    static constexpr const char * CONF_ENABLED = "enabled";
    static constexpr const char * CONF_PID_P = "p";
    static constexpr const char * CONF_PID_I = "i";
    static constexpr const char * CONF_PID_D = "d";
    // static constexpr const char * CONF_PID_I_MAX = "i_max";
    // static constexpr const char * CONF_PID_I_MIN = "i_min";
    static constexpr const char * CONF_PID_MAX = "pid_max";
    static constexpr const char * CONF_PID_MIN = "pid_min";
    static constexpr const char * CONF_CONTROL_ALLOCATION_MATRIX = "control_allocation_matrix";
    static constexpr const char * CONF_CONTROL_TF = "control_tf";
    static constexpr const char * CONF_CONTROL_TF_SERVO = "control_tf_servo";
    static constexpr const char * CONF_CONTROLLER_FREQUENCY = "controller_frequency";
    static constexpr const char * CONF_NO_SETPOINT_TIMEOUT = "controller_no_setpoint_timeout";
    
    static constexpr const char * TOPIC_SAFETY = "safety";
    static constexpr const char * TOPIC_STATUS = "status";
    static constexpr const char * TOPIC_CONTROL_PROCESS_VALUE = "controller/process/value";
    static constexpr const char * TOPIC_CONTROL_PROCESS_SET_POINT = "controller/process/set_point";
    static constexpr const char * TOPIC_CONTROL_PROCESS_ERROR = "controller/process/error";

    static constexpr const char * SERVICE_CONTROL_ENABLE = "controller/enable";
    static constexpr const char * SERVICE_CONTROL_DISABLE = "controller/disable";
    static constexpr const char * SERVICE_GET_CONTROLLER_STATE = "controller/get_state";
    static constexpr const char * SERVICE_GET_CONTROL_MODES = "controller/get_modes";
    static constexpr const char * SERVICE_SET_CONTROL_POINT = "controller/set_point";
    static constexpr const char * SERVICE_GET_ACTIVE_MODE = "controller/active_mode";
    static constexpr const char * SERVICE_RESET_INTEGRAL_ERROR = "controller/reset_integral_error";
    
    struct DOF {
        //! @NOTE: ROLL_RATE, PITCH_RATE, YAW_RATE are controlle in global frame
        enum IDX : int {
            X =             mvp_msgs::ControlMode::DOF_X,
            Y =             mvp_msgs::ControlMode::DOF_Y,
            Z =             mvp_msgs::ControlMode::DOF_Z,
            ROLL =          mvp_msgs::ControlMode::DOF_ROLL,
            PITCH =         mvp_msgs::ControlMode::DOF_PITCH,
            YAW =           mvp_msgs::ControlMode::DOF_YAW,
            SURGE =         mvp_msgs::ControlMode::DOF_SURGE,
            SWAY =          mvp_msgs::ControlMode::DOF_SWAY,
            HEAVE =         mvp_msgs::ControlMode::DOF_HEAVE,
            ROLL_RATE =     mvp_msgs::ControlMode::DOF_ROLL_RATE,
            PITCH_RATE =    mvp_msgs::ControlMode::DOF_PITCH_RATE,
            YAW_RATE =      mvp_msgs::ControlMode::DOF_YAW_RATE,
        };
    };

    static const std::map<const char *, int> CONF_DOF_LOOKUP = {
        {CONF_DOF_X,          DOF::X},
        {CONF_DOF_Y,          DOF::Y},
        {CONF_DOF_Z,          DOF::Z},
        {CONF_DOF_ROLL,       DOF::ROLL},
        {CONF_DOF_PITCH,      DOF::PITCH},
        {CONF_DOF_YAW,        DOF::YAW},
        {CONF_DOF_SURGE,      DOF::SURGE},
        {CONF_DOF_SWAY,       DOF::SWAY},
        {CONF_DOF_HEAVE,      DOF::HEAVE},
        {CONF_DOF_ROLL_RATE,  DOF::ROLL_RATE},
        {CONF_DOF_PITCH_RATE, DOF::PITCH_RATE},
        {CONF_DOF_YAW_RATE,   DOF::YAW_RATE},
    };

    static const char * const DOFS[] = {
        CONF_DOF_X,
        CONF_DOF_Y,
        CONF_DOF_Z,
        CONF_DOF_ROLL,
        CONF_DOF_PITCH,
        CONF_DOF_YAW,
        CONF_DOF_SURGE,
        CONF_DOF_SWAY,
        CONF_DOF_HEAVE,
        CONF_DOF_ROLL_RATE,
        CONF_DOF_PITCH_RATE,
        CONF_DOF_YAW_RATE,
        nullptr
    };

    static constexpr int CONTROLLABLE_DOF_LENGTH = 12;
}