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

#include "stdexcept"

namespace ctrl {
    /** @brief Exception class for Alpha ROS class
     *
     */
    class control_ros_exception : public std::runtime_error {
    public:
        explicit control_ros_exception(const std::string &message)
            : std::runtime_error(message) {}
    };

    /** @brief Exception class for Alpha Control
     *
     */
    class control_exception : public std::runtime_error {
    public:
        explicit control_exception(const std::string &message)
            : std::runtime_error(message) {}
    };

}