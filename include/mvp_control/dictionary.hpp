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

#pragma once

#include "vector"
#include "string"
#include "map"
#include "limits"

#include "mvp_msgs/msg/control_modes.hpp"

namespace ctrl {

    static constexpr const char * CONF_DOF_X = "x";
    static constexpr const char * CONF_DOF_Y = "y";
    static constexpr const char * CONF_DOF_Z = "z";
    static constexpr const char * CONF_DOF_ROLL = "roll";
    static constexpr const char * CONF_DOF_PITCH = "pitch";
    static constexpr const char * CONF_DOF_YAW = "yaw";
    static constexpr const char * CONF_DOF_U = "u";
    static constexpr const char * CONF_DOF_V = "v";
    static constexpr const char * CONF_DOF_W = "w";
    static constexpr const char * CONF_DOF_P = "p";
    static constexpr const char * CONF_DOF_Q = "q";
    static constexpr const char * CONF_DOF_R = "r";

    static constexpr const char * CONF_THRUSTER_POLY = "thruster_polynomials";
    static constexpr const char * CONF_THRUSTER_XYZ_FLAG = "thruster_xyz_flag";

    static constexpr const char * CONF_THRUST_COMMAND_TOPICS = "thruster_command_topics";
    static constexpr const char * CONF_THRUSTER_FORCE_TOPICS = "thruster_force_topics";
    static constexpr const char * CONF_THRUSTER_IDS = "thruster_ids";

    static constexpr const char * CONF_THRUSTER_LIMITS = "thruster_limits";
    static constexpr const char * CONF_THRUSTER_D_LIMIT = "delta_limit";
    static constexpr const char * CONF_THRUSTER_MAX = "max";
    static constexpr const char * CONF_THRUSTER_MIN = "min";

    static constexpr const char * CONF_THRUSTER_SERVO_LIMITS = "thruster_servo_limits";
    static constexpr const char * CONF_THRUSTER_SERVO_TOPIC = "thruster_servo_topic";
    static constexpr const char * CONF_THRUSTER_SERVO_JOINT = "thruster_servo_joint";
    static constexpr const char * CONF_THRUSTER_SERVO_SPEED = "thruster_servo_speed";
    
    static constexpr const char * CONF_GENERATOR_TYPE = "generator_type";
    static constexpr const char * CONF_GENERATOR_TYPE_OPT_TF = "tf";
    static constexpr const char * CONF_GENERATOR_TYPE_OPT_USER = "user";

    static constexpr const char * CONF_TF_PREFIX = "tf_prefix";
    static constexpr const char * CONF_TF_PREFIX_DEFAULT = "";
    static constexpr const char * CONF_CG_LINK = "cg_link";
    static constexpr const char * CONF_CG_LINK_DEFAULT = "cg_link";

    static constexpr const char * CONF_CHILD_LINK = "child_link";
    static constexpr const char * CONF_CHILD_LINK_DEFAULT = "cg_link";

    static constexpr const char * CONF_WORLD_LINK = "world_link";
    static constexpr const char * CONF_WORLD_LINK_DEFAULT = "world";

    static constexpr const char * CONF_WORLD_LINK_INITIAL = "world_link_initial";
    static constexpr const char * CONF_CHILD_LINK_INITIAL = "child_link_initial";



    static constexpr const char * CONF_ODOMETRY_SOURCE = "odometry_source";
    static constexpr const char * CONF_ODOMETRY_SOURCE_DEFAULT = "odometry";
    static constexpr const char * CONF_CONTROL_MODES = "control_modes";
    static constexpr const char * CONF_ENABLED = "enabled";
    static constexpr const char * CONF_PID_P = "p";
    static constexpr const char * CONF_PID_I = "i";
    static constexpr const char * CONF_PID_D = "d";
    static constexpr const char * CONF_PID_MAX = "pid_max";
    static constexpr const char * CONF_PID_MIN = "pid_min";
    static constexpr const char * CONF_CONTROL_ALLOCATION_MATRIX = "control_allocation_matrix";
    static constexpr const char * CONF_CONTROL_TF = "control_tf";
    static constexpr const char * CONF_CONTROLLER_FREQUENCY = "controller_frequency";
    static constexpr const char * CONF_NO_SETPOINT_TIMEOUT = "controller_no_setpoint_timeout";


    static constexpr const char * TOPIC_SAFETY = "safety";
    static constexpr const char * TOPIC_STATUS = "status";
    static constexpr const char * TOPIC_CONTROLLER_STATE = "controller/state";
    static constexpr const char * TOPIC_CONTROL_PROCESS_VALUE = "controller/process/value";
    static constexpr const char * TOPIC_CONTROL_PROCESS_SET_POINT = "controller/process/set_point";
    static constexpr const char * TOPIC_CONTROL_PROCESS_ERROR = "controller/process/error";

    static constexpr const char * SERVICE_CONTROL_ENABLE = "controller/enable";
    static constexpr const char * SERVICE_CONTROL_DISABLE = "controller/disable";
    static constexpr const char * SERVICE_SET_CONTROLLER = "controller/set";
    static constexpr const char * SERVICE_GET_CONTROL_MODES = "controller/get_modes";
    static constexpr const char * SERVICE_SET_CONTROL_POINT = "controller/set_point";
    static constexpr const char * SERVICE_GET_ACTIVE_MODE = "controller/active_mode";
    static constexpr const char * SERVICE_GET_CONTROLLER_STATE = "controller/get_state";

    struct DOF {
        //! @NOTE: ROLL_RATE, PITCH_RATE, YAW_RATE are controlle in global frame
        enum IDX : int {
            X =             mvp_msgs::msg::ControlMode::DOF_X,
            Y =             mvp_msgs::msg::ControlMode::DOF_Y,
            Z =             mvp_msgs::msg::ControlMode::DOF_Z,
            ROLL =          mvp_msgs::msg::ControlMode::DOF_ROLL,
            PITCH =         mvp_msgs::msg::ControlMode::DOF_PITCH,
            YAW =           mvp_msgs::msg::ControlMode::DOF_YAW,
            U =             mvp_msgs::msg::ControlMode::DOF_U,
            V =             mvp_msgs::msg::ControlMode::DOF_V,
            W =             mvp_msgs::msg::ControlMode::DOF_W,
            P =             mvp_msgs::msg::ControlMode::DOF_P,
            Q =             mvp_msgs::msg::ControlMode::DOF_Q,
            R =             mvp_msgs::msg::ControlMode::DOF_R,
        };
    };

    static const std::map<const char *, int> CONF_DOF_LOOKUP = {
        {CONF_DOF_X,        DOF::X},
        {CONF_DOF_Y,        DOF::Y},
        {CONF_DOF_Z,        DOF::Z},
        {CONF_DOF_ROLL,     DOF::ROLL},
        {CONF_DOF_PITCH,    DOF::PITCH},
        {CONF_DOF_YAW,      DOF::YAW},
        {CONF_DOF_U,        DOF::U},
        {CONF_DOF_V,        DOF::V},
        {CONF_DOF_W,        DOF::W},
        {CONF_DOF_P,        DOF::P},
        {CONF_DOF_Q,        DOF::Q},
        {CONF_DOF_R,        DOF::R},
    };

    static const char * const DOFS[] = {
        CONF_DOF_X,
        CONF_DOF_Y,
        CONF_DOF_Z,
        CONF_DOF_ROLL,
        CONF_DOF_PITCH,
        CONF_DOF_YAW,
        CONF_DOF_U,
        CONF_DOF_V,
        CONF_DOF_W,
        CONF_DOF_P,
        CONF_DOF_Q,
        CONF_DOF_R,
        nullptr
    };

    static constexpr int CONTROLLABLE_DOF_LENGTH = 12;
}