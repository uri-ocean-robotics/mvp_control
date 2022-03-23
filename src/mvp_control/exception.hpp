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