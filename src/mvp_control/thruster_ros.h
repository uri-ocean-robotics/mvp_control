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

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "Eigen/Dense"
#include "polynomial_solver.h"
#include "sensor_msgs/JointState.h"

namespace ctrl {

    class MvpControlROS;

    /** @brief Thruster class for managing data
     *
     */
    class ThrusterROS {
    private:

        friend MvpControlROS;

        //! @brief Public node handler
        ros::NodeHandle m_nh;

        //! @brief Private node handler for accessing private parameters
        ros::NodeHandle m_pnh;
        
        //! @brief Thruster ID
        std::string m_id;

        //! @brief Thrust command topic ID
        std::string m_thrust_command_topic_id;

        //! @brief Thruster force topic id
        std::string m_thrust_force_topic_id;

        //! @brief Servo command topic id
        std::string m_servo_command_topic_id;
        
        //! @brief thruster link id
        std::string m_link_id;

        //! @brief servo link id
        std::string m_servo_link_id;

        //! @brief thruster frame id
        std::string m_frame_id;

        std::vector<double> servo_speeds; 

        //! @brief Transform prefix
        std::string m_tf_prefix_thruster;

        /** @brief Thruster contribution vector
         *
         * This vector defines a column in control allocation matrix.
         * Each element in the vector describes contribution on
         * vehicle motion of the thruster in each degree of freedom
         */
        Eigen::VectorXd m_contribution_vector;

        /** @brief Stores the IDs of servo joints associated with this thruster.
         *
         * This vector contains the IDs of all servo joints that are associated
         * with a particular thruster. Each thruster may control or affect multiple
         * servo joints, and this variable maps a thruster to its corresponding
         * servo joints. 
         */
        std::vector<std::string> servo_joints; 

        //! @brief Thrust publisher
        ros::Publisher m_thrust_publisher;

        //! @brief Servo angle command publisher
        ros::Publisher m_servo_command_publisher;

        //! @brief Thrust force publisher
        ros::Publisher m_force_publisher;

        //! @brief Servo Joint publisher
        ros::Publisher m_joint_state_publisher;

        //! @brief Joint state topic ID
        std::string m_joint_state_topic_id;

        //! @brief Joint state desired topic ID
        std::string m_joint_state_desired_topic_id;

        //! @brief Polynomial solver
        PolynomialSolver::Ptr m_poly_solver;

        //! @brief Servo calibration coefficients
        std::vector<double> servo_coeff_;  
        
        //! @brief Servo speed rad/s
        double m_omega;

        //! @brief Maximum force limit
        double m_force_max;

        //! @brief Minimum force limit
        double m_force_min;

        //! @brief Maximum angle limit
        double m_angle_max;

        //! @brief Minimum angle limit
        double m_angle_min;

        //! @brief If the thruster is articulated
        int is_articulated;
        
    public:

        //! @brief Default constructor
        ThrusterROS();

        /** @brief Gets whether the thruster is articulated.
         *
         * @return True if the thruster is articulated, false otherwise.
         */
        int get_is_articulated() const;

        /** @brief Sets whether the thruster is articulated.
         *
         * @param value True to set the thruster as articulated, false to set it as non-articulated.
         */
        void set_is_articulated(int value);
        
        /** @brief Sets the servo joints associated with the thruster.
         *
         * @param joints A vector of strings containing the IDs of the servo joints associated with the thruster.
         */
        void set_servo_joints(const std::vector<std::string>& joints);

        /** @brief Gets the servo joints associated with the thruster.
         *
         * @return A vector of strings containing the IDs of the servo joints associated with the thruster.
         */
        std::vector<std::string> get_servo_joints() const;

        /** @brief Thruster ROS class constructor.
         *
         * This constructor should be used in normal operation.
         * Initializes Thruster ID, Topic ID and contribution vector
         *
         * @param id
         * @param topic_id
         * @param contribution_vector
         */
        ThrusterROS(std::string id, std::string topic_id,
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

        /** @brief Trivial getter for servo command topic id
         *
         * @return #ThrusterROS::m_servo_command_topic_id
         */
        auto get_servo_command_topic_id() -> decltype(m_servo_command_topic_id);

        /** @brief Default Setter servo command for topic id
         *
         * @param topic_id
         */
        void set_servo_command_topic_id(const decltype(m_servo_command_topic_id) &topic_id);

        /** @brief Trivial getter for joint state topic id
         *
         * @return The topic ID for publishing joint states
         */
        auto get_joint_state_topic_id() -> decltype(m_joint_state_topic_id);

        /** @brief Default Setter for joint state topic id
         *
         * @param topic_id The topic ID to set for publishing joint states
         */
        void set_joint_state_topic_id(const decltype(m_joint_state_topic_id) &topic_id);

        /** @brief Trivial getter for joint state topic id
         *
         * @return The topic ID for publishing joint states
         */
        auto get_joint_state_desired_topic_id() -> decltype(m_joint_state_topic_id);

        /** @brief Default Setter for joint state topic id
         *
         * @param topic_id The topic ID to set for publishing joint states
         */
        void set_joint_state_desired_topic_id(const decltype(m_joint_state_topic_id) &topic_id);

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

        /** @brief Trivial getter for servo_link id
         *
         * @return #ThrusterROS::m_servo_link_id
         */
        auto get_servo_link_id() -> decltype(m_servo_link_id);

        /** @brief Trivial Setter for link id
         *
         * @param servo_link_id
         */
        void set_servo_link_id(const decltype(m_servo_link_id) &servo_link_id);

        /** @brief Trivial getter for frame id
         *
         * @return #ThrusterROS::m_frame_id
         */
        auto get_frame_id() -> decltype(m_frame_id);

        /** @brief Trivial Setter for frame id
         *
         * @param frame_id
         */
        void set_frame_id(const decltype(m_frame_id) &frame_id);

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
        void set_contribution_vector(
            const decltype(m_contribution_vector) &contribution_vector);

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
  
        /** @brief Trivial setter for servo coefficients
         *
         * @param coeff A vector containing the servo calibration coefficients
         */
        void set_servo_coeff(const std::vector<double>& coeff);

        /** @brief Trivial getter for servo coefficients
         *
         * @return A vector containing the servo calibration coefficients
         */
        const std::vector<double>& get_servo_coeff() const;

        //! @brief Generic typedef for shared pointer
        typedef std::shared_ptr<ThrusterROS> Ptr;

        /** @brief Trivial getter for servo speeds
         *
         * @return A constant reference to the vector of servo speeds
         */
        const std::vector<double>& getServoSpeeds() const;

        /** @brief Trivial setter for servo speeds
         *
         * @param speeds The vector of servo speeds to set
         */
        void setServoSpeeds(const std::vector<double>& speeds);

        /** @brief Publish thruster command
         *
         * Thuster command should be between -1 and 1
         *
         * @param cmd
         */
        void command(double cmd);

        /** @brief Publish servo command
         *
         * Servo command should be between -1 and 1
         *
         * @param normalized_angle
         */
        void servo_joint_command(double normalized_angle);

        /** @brief Request force from thruster
         *
         * This method gets input \p N as Newton and applies it to a polynomial solver
         * that is defined with #PolynomialSolver::m_coeff.
         *
         * @param N force as newton
         * @return true if polynomial is solved, false if polynomial isn't solved.
         */
        bool request_force(double N);

        /**
         * @brief Publishes the requested angles for multiple joints.
         *
         * This method publishes the desired joint angles to servo hardware for all specified joints.
         *
         * @param joint_names A vector containing the names of the joints.
         * @param requested_angles A vector containing the requested angles for each joint in radians.
         * @return true if the joint states are successfully published.
         */
        bool request_joint_angles(const std::vector<std::string>& joint_names, const std::vector<double>& requested_angles);

        /** @brief Normalize the angle to PWM using the servo coefficients
         *
         * @param angle The input angle
         * @return The normalized PWM value
         */
        double normalize_angle(double angle) const;
        
    };

}