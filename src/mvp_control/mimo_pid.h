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

#include "Eigen/Dense"
#include "deque"
#include "functional"
#include "memory"

namespace ctrl {

    class MimoPID {
    private:

        /** @brief Delta Time
         *
         * This variable holds the time difference between each iteration
         */
        double m_dt;

        /** @brief Integral window time
         *
         * This variable hold integral time window
         */
        double m_dt_i;

        /**
         * @brief Maximum value
         *
         * This variable holds the maximum value of a gain
         */
        Eigen::ArrayXd m_i_max;

        /**
         * @brief Minimum value
         *
         * This variable holds the minimum value of a gain
         */
        Eigen::ArrayXd m_i_min;

        //! @brief Proportional gain
        Eigen::ArrayXd m_kp;

        //! @brief Derivation gain
        Eigen::ArrayXd m_kd;

        //! @brief Integration gain
        Eigen::ArrayXd m_ki;

        //! @brief Error from the previous iteration
        Eigen::ArrayXd m_pe;

        //! @brief Integral queue
        std::deque<Eigen::ArrayXd> m_integral_queue;

        Eigen::ArrayXd m_i;

        std::function<Eigen::ArrayXd(const Eigen::ArrayXd &desired,
                                     const Eigen::ArrayXd &current)> m_error_function;

    public:

        MimoPID();

        /** @brief Calculates PID gain with given desired and current state
         *
         * @param u         Resulting control output
         * @param desired   A vector defines the desired state of the system
         * @param current   A vector defines the current state of the system
         * @param dt        Time difference between readings
         * @return          true if its not the first run
         */
        bool calculate(Eigen::VectorXd *u, const Eigen::ArrayXd &desired,
                       const Eigen::ArrayXd &current, double dt);

        //! @brief Generic shared pointer
        typedef std::shared_ptr<MimoPID> Ptr;

        //! @brief Default getter for proportional gain
        auto get_kp() -> decltype(m_kp);

        /*! @brief Default setter for proportional gain
         *
         * @param gain
         */
        void set_kp(const decltype(m_kp) &gain);

        //! @brief Default getter for integral gain
        auto get_ki() -> decltype(m_ki);

        /*! @brief Default setter for integral gain
         *
         * @param gain
         */
        void set_ki(const decltype(m_ki) &gain);

        //! @brief Default getter for derivative gain
        auto get_kd() -> decltype(m_kd);

        /*! @brief Default setter for derivative gain
         *
         * @param gain
         */
        void set_kd(const decltype(m_kd) &gain);

        //! @brief Default getter for delta time
        auto get_dt() const -> decltype(m_dt);

        /*! @brief Default setter for delta time
         *
         * @param gain
         */
        void set_dt(const decltype(m_dt) &gain);

        //! @brief Default getter for integral time window
        auto get_dt_i() const -> decltype(m_dt_i);

        /*! @brief Default setter for integral time window
         *
         * @param gain
         */
        void set_dt_i(const decltype(m_dt_i) &gain);

        //! @brief Default getter for max
        auto get_i_max() -> decltype(m_i_max);

        /*! @brief Default setter for max
         *
         * @param gain
         */
        void set_i_max(const decltype(m_i_max) &gain);

        //! @brief Default getter for min
        auto get_i_min() -> decltype(m_i_min);

        /*! @brief Default setter for min
         *
         * @param gain
         */
        void set_i_min(const decltype(m_i_min) &gain);

        //! @brief Default getter for error function
        auto get_error_function() -> decltype(m_error_function);

        /*! @brief Default setter for error function
         *
         * @param func
         */
        void set_error_function(const decltype(m_error_function) &func);
    };

}