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

#include <utility>

#include "cstdlib"
#include "string"
#include "vector"
#include "array"
#include "memory"

#include "Eigen/Dense"

#include "mvp_control/dictionary.h"


namespace mvp {

    enum ActuatorType : u_int8_t {
        THRUSTER,
        AZIMUTH_THRUSTER,
        RUDDER, // TODO: NOT IMPLEMENTED YET!
        UNDEFINED
    };

    struct ActuatorConfig {

        //! @brief Actuator name
        std::string name;

        //! @brief Type of actuator
        ActuatorType type;

        //! @brief Transform Tree Link name
        std::string link_id;

        //! @brief Topic name for thruster_control;
        std::string command_topic;

        std::string force_topic;

        std::string joint_y_topic;

        std::string joint_z_topic;

        //! @brief Force limits in Newtons, [0]: min, [1]: max
        double force_limits[2];

        std::vector<double> control_curve;

        //! @brief Joint Limits around Y-axis in radians. [0]: min, [1]: max.
        std::vector<double> joint_y_limits;

        //! @brief Joint Limits around Z-axis in radians. [0]: min, [1]: max.
        std::vector<double> joint_z_limits;

        //! @brief Topic name for servo control around Y-axis.
        std::string joint_y_name;

        //! @brief Topic name for servo control around Z-axis.
        std::string joint_z_name;

    };

    class Actuator {
    protected:
        /** @brief Thruster contribution vector
         *
         * This vector defines a column in control allocation matrix.
         * Each element in the vector describes contribution on
         * vehicle motion of the thruster in each degree of freedom
         */
        Eigen::Matrix<double, CONTROLLABLE_DOF_LENGTH, 3> m_contribution_matrix;

        Eigen::MatrixXd m_constraint_matrix;

        Eigen::VectorXd m_upper_limit;

        Eigen::VectorXd m_lower_limit;

        ActuatorConfig m_config;

        /**
         * @brief Compute actuator contribution for force and torque per axis
         * @param t
         * @param axis X: 0, Y: 1, Z: 2
         */
        void f_compute_single_axis_contribution(
            const Eigen::Isometry3d &t,
            int axis);

    public:

        Actuator() = default;

        explicit Actuator(ActuatorConfig conf) : m_config(std::move(conf)) {};

        virtual auto get_contribution_matrix()
            -> decltype(m_contribution_matrix) final {
            return m_contribution_matrix;
        }

        virtual auto get_constraint_matrix() -> decltype(m_constraint_matrix) final {
            return m_contribution_matrix;
        }

        virtual auto get_upper_limit_vector() -> decltype(m_upper_limit) final {
            return m_upper_limit;
        }

        virtual auto get_lower_limit_vector() -> decltype(m_lower_limit) final {
            return m_lower_limit;
        }

        virtual auto get_config() -> decltype(m_config) final {
            return m_config;
        }

        virtual void compute_contribution(const Eigen::Isometry3d &t) = 0;

        virtual void compute_constraints() = 0;

        virtual void initialize() = 0;

        virtual bool command(const Eigen::Vector3d& N) = 0;
    };

    class Thruster : public Actuator {
    private:

    public:

        Thruster() = default;

        explicit Thruster(ActuatorConfig conf) :
            Actuator(std::move(conf)) {}

        void compute_contribution(const Eigen::Isometry3d &t) override;

        void compute_constraints() override;

        void initialize() override;

        bool command(const Eigen::Vector3d &N) override;
    };

    class AzimuthThruster : public Actuator {
    private:

    public:

        AzimuthThruster() = default;

        explicit AzimuthThruster(ActuatorConfig conf) :
            Actuator(std::move(conf)) {}

        void compute_contribution(const Eigen::Isometry3d &t) override;

        void compute_constraints() override;

        void initialize() override;

        bool command(const Eigen::Vector3d& N) override;
    };

}