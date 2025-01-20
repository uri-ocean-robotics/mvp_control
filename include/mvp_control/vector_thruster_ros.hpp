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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "Eigen/Dense"
#include "mvp_control/polynomial_solver.hpp"

#include "chrono"
#include "thread"
#include <fstream> 


namespace ctrl {

    class MvpControlROS;

    /** @brief Thruster class for managing data
     *
     */
    class VectorThrusterROS {
    private:
        // rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logger_;
        friend MvpControlROS;
        // rclcpp::Node::SharedPtr node_;
        
        //! @brief Thruster ID
        std::string m_id;

        //! @brief Thrust command topic ID
        std::string m_thrust_command_topic_id;

        //! @brief Thruster force topic id
        std::string m_thrust_force_topic_id;

        //! @brief Thruster_servo_joint_id
        std::string m_thruster_servo_topic_id;

        //! @brief Thruster_servo_joint_id
        std::string m_thruster_servo_joint_id;

        //! @brief thruster link id
        std::string m_link_id;

        double m_force_max;

        double m_force_min;

        double m_servo_angle_max;

        double m_servo_angle_min;

        double m_servo_angle;

        double m_servo_speed;

        double m_thruster_direction;

        /** @brief Thruster contribution vector
         *
         * This vector defines a column in control allocation matrix.
         * Each element in the vector describes contribution on
         * vehicle motion of the thruster in each degree of freedom
         */
        Eigen::VectorXd m_contribution_vector;

        //! @brief Polynomial solver
        PolynomialSolver::Ptr m_poly_solver;

        

        //! @brief Thrust publisher
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_thrust_publisher;
        //! @brief Thrust force publisher
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_force_publisher;
        //! @brief Thrust angle publisher
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_angle_publisher;

    public:

        //! @brief Default constructor
        VectorThrusterROS();

        /** @brief Thruster ROS class constructor.
         *
         * This constructor should be used in normal operation.
         * Initializes Thruster ID, Topic ID and contribution vector
         *
         * @param id
         * @param topic_id
         * @param contribution_vector
         */
        VectorThrusterROS(std::string id, std::string topic_id, std::string servo_topic_id,
                        Eigen::VectorXd contribution_vector);
        /** @brief Initializes publishers and subscribers
         *
         */
        void initialize();

        /** @brief Trivial getter for topic id
         *
         * @return #ThrusterROS::m_thrust_command_topic_id
         */
        auto
        get_thrust_command_topic_id() -> decltype(m_thrust_command_topic_id);

        /** @brief Default Setter for topic id
         *
         * @param topic_id
         */
        void set_thrust_command_topic_id(
            const decltype(m_thrust_command_topic_id) &topic_id);

        /** @brief Trivial getter for force topic id
         *
         * @return #ThrusterROS::m_thrust_force_topic_id
         */
        auto get_thrust_force_topic_id() -> decltype(m_thrust_force_topic_id);

        /** @brief Default Setter force for topic id
         *
         * @param topic_id
         */
        void set_thrust_force_topic_id(const decltype(m_thrust_force_topic_id) &topic_id);

        /** @brief Default getter thruster servo topic id
         *
         * @param topic_id
         */
        auto get_thruster_servo_topic_id() -> decltype(m_thruster_servo_topic_id);

         /** @brief Default Setter thruster servo topic id
         *
         * @param topic_id
         */
        void set_thruster_servo_topic_id(const decltype(m_thruster_servo_topic_id) &topic_id);


        /** @brief Default Setter thruster servo joint id
         *
         * @param joint_id
         */
        auto get_thruster_servo_joint_id() -> decltype(m_thruster_servo_joint_id);

        void set_thruster_servo_joint_id(const decltype(m_thruster_servo_joint_id) &joint_id);

        auto get_thruster_servo_speed() -> decltype(m_servo_speed);

        void set_thruster_servo_speed(const decltype(m_servo_speed) &servo_speed);

        auto get_thruster_direction() -> decltype(m_thruster_direction);

        void set_thruster_direction(const decltype(m_thruster_direction) &thruster_direction);


        /** @brief Default getter servo angle
         *
         * @param topic_id
         */
        auto get_thruster_servo_angle() -> decltype(m_servo_angle);

         /** @brief Default Setter thruster servo angle
         *
         * @param topic_id
         */
        void set_thruster_servo_angle(const decltype(m_servo_angle) &servo_angle);


        /** @brief Trivial getter for link id
         *
         * @return #ThrusterROS::m_link_id
         */
        auto get_link_id() -> decltype(m_link_id);

        /** @brief Trivial Setter for link id
         *
         * @param link_id
         */
        void set_link_id(const decltype(m_link_id) &link_id);


        /** @brief Trivial getter for thruster id
         *
         * @return #ThrusterROS::m_id
         */
        auto get_id() -> decltype(m_id);

        /** @brief Trivial Setter for topic id
         *
         * @param thruster_id
         */
        void set_id(const decltype(m_id) &thruster_id);

        /** @brief Trivial getter for contribution vector
         *
         * @return #ThrusterROS::m_contribution_vector
         */
        auto get_contribution_vector() -> decltype(m_contribution_vector);

        /** @brief Trivial Setter for contribution vector
         *
         * @param contribution Contribution vector for the thruster
         */
        void set_contribution_vector(const decltype(m_contribution_vector) &contribution_vector);

        /** @brief Trivial getter for polynomial solver
         *
         * @return #ThrusterROS::m_poly_solver
         */
        auto get_poly_solver() -> decltype(m_poly_solver);

        /** @brief Trivial setter for polynomial solver
         *
         * @param solver
         */
        void set_poly_solver(decltype(m_poly_solver) solver);

        //! @brief Generic typedef for shared pointer
        typedef std::shared_ptr<VectorThrusterROS> Ptr;

        /** @brief Request force from thruster
         *
         * This method gets input \p N as Newton and applies it to a polynomial solver
         * that is defined with #PolynomialSolver::m_coeff.
         *
         * @param N force as newton
         * @return true if polynomial is solved, false if polynomial isn't solved.
         */

        bool request_command(double fx, double fy, double current_angle, double &command, double &new_angle);
        
    };

}