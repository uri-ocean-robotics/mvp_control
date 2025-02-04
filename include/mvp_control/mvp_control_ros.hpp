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

/*******************************************************************************
 * STL
 */
#include "functional"
#include "thread"
#include <string>
/*******************************************************************************
 * ROS
 */
#include "rclcpp/rclcpp.hpp"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" 


#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
// #include "dynamic_reconfigure/server.h"

// #include "mvp_control/PIDConfig.h"

#include "mvp_msgs/msg/pid_gains.hpp"
#include "mvp_msgs/msg/control_process.hpp"
#include "mvp_msgs/msg/control_modes.hpp"
#include "mvp_msgs/srv/get_control_mode.hpp"
#include "mvp_msgs/srv/get_control_modes.hpp"
#include "mvp_msgs/srv/set_control_point.hpp"

/*******************************************************************************
 * MVP
 */

#include "mvp_control/mvp_control.hpp"
#include "mvp_control/thruster_ros.hpp"
#include "mvp_control/vector_thruster_ros.hpp"


/*******************************************************************************
 * Eigen
 */

#include "Eigen/Dense"

/*******************************************************************************
 * BOOST
 */
#include "boost/thread/recursive_mutex.hpp"


namespace ctrl {
/** @brief ROS wrapper for MvpControl
 *
 *  This package wraps mvp control class and utilizes its
 *  functionality.
 *
 *  @see MvpControl
 */
    class MvpControlROS : public rclcpp::Node
    {
    public:
        /** @brief Default constructor
         *
         */
        MvpControlROS(std::string name = "mvp_control");
        /** @brief Initializer for Mvp Control ROS
         *
         * This function initializes control allocation matrix and
         *
         */
        void initialize();

        //! @brief Generic typedef for shared pointer
        // typedef std::shared_ptr<MvpControlROS> Ptr;
        // std::shared_ptr<rclcpp::Node> m_nh;

    private:
        /*! @brief Generator Type enum class
         *
         */
        enum class GeneratorType {
            TF,
            USER,
            UNKNOWN
        };

        //! @brief Public node handler
        // std::shared_ptr<rclcpp::Node> m_nh;

        //! @brief Private node handler
        

        //! @brief Thruster list
        std::vector<ThrusterROS::Ptr> m_thrusters;

        std::vector<VectorThrusterROS::Ptr> m_vector_thrusters;

        /**! @brief Control Allocation Matrix
         *
         *  Control allocation matrix is generated from individual
         *  configurations of the thrusters.
         */
        Eigen::MatrixXd m_control_allocation_matrix;

        Eigen::MatrixXd m_direction_cost_matrix;

        Eigen::VectorXd m_direction_cost_matrix_c;

        std::string generator_type;  //tf generate type
        //! @brief Center of gravity link id
        std::string m_cg_link_id;

        std::string m_child_link_id_initial;

        std::string m_child_link_id;

        //! @brief World link id
        std::string m_world_link_id;

        std::string m_world_link_id_initial;

        //! @brief mvp_control config yaml file
        std::string m_control_config_file;

        //! @brief Transform buffer for TF2
        std::unique_ptr<tf2_ros::Buffer> m_transform_buffer;
        // std::unique_ptr<tf2_ros::Buffer> m_transform_buffer;

        //! @brief Transform listener for TF2
        std::unique_ptr<tf2_ros::TransformListener> m_transform_listener;
        
        //! @brief Control allocation matrix generator type
        GeneratorType m_generator_type;

        //! @brief Transform prefix
        std::string m_tf_prefix;

        //! @brief Mvp Control object
        MvpControl::Ptr m_mvp_control;

        //! @brief Process values
        Eigen::VectorXd m_process_values;

        //! @brief Set point
        Eigen::VectorXd m_set_point;

        //! @brief Controller frequency
        double m_controller_frequency;

        //! @brief setpoint disable timeout
        double m_no_setpoint_timeout;

        //! @brief Setpoint timeer
        double setpoint_timer;

        bool m_vector_thruster_auto_direction;

        double m_total_force_cost_factor;

        //! @brief Get control modes ros service server
        // ros::ServiceServer m_get_control_modes_server;
        rclcpp::Service<mvp_msgs::srv::GetControlModes>::SharedPtr m_get_control_modes_server;

        //! @brief Set control point ros service server
        // ros::ServiceServer m_set_control_point_server;
        rclcpp::Service<mvp_msgs::srv::SetControlPoint>::SharedPtr m_set_control_point_server;

        //! @brief Enable controller ros service server
        // ros::ServiceServer m_enable_controller_server;
        // rclcpp::Service<std_srvs::srv::Empty>::SharedPtr m_enable_controller_server;

        //! @brief Disable controller ros service server
        // ros::ServiceServer m_disable_controller_server;
        // rclcpp::Service<std_srvs::srv::Empty>::SharedPtr m_disable_controller_server;

        //! @brief Get current controller state enabled or disabled
        // ros::ServiceServer m_get_controller_state_server;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_get_controller_state_server;

        //! @brief Active mode getter ros service server
        // ros::ServiceServer m_get_active_mode_server;
        rclcpp::Service<mvp_msgs::srv::GetControlMode>::SharedPtr m_get_active_mode_server;

        //! @brief Set controller service server
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr m_set_controller_server;

        //! @brief Trivial subscriber
        // ros::Subscriber m_odometry_subscriber;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odometry_subscriber;

        //! @brief Process value publisher
        // ros::Publisher m_process_value_publisher;
        rclcpp::Publisher<mvp_msgs::msg::ControlProcess>::SharedPtr m_process_value_publisher;

        //! @brief Set point subscriber
        // ros::Subscriber m_set_point_subscriber;
        rclcpp::Subscription<mvp_msgs::msg::ControlProcess>::SharedPtr m_set_point_subscriber;


        //! @brief subscribe servo joint for setting the vector thruster servo angle
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_servo_joint_subscriber;

        //! @brief subscribe thrust direction for vector thrusters
        rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr m_vector_thruster_direction_subscriber;

        //! @brief Publishes process error publisher
        // ros::Publisher m_process_error_publisher;
        rclcpp::Publisher<mvp_msgs::msg::ControlProcess>::SharedPtr m_process_error_publisher;
        
        //! @brief Publishes process error publisher
        // ros::Publisher m_process_error_publisher;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_controller_state_publisher;
        
        

        // //! @brief Thrust publisher
        // rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_thrust_publisher;

        // //! @brief Thrust force publisher
        // rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_force_publisher;

        //! @brief Holder for latest odometry msg
        nav_msgs::msg::Odometry m_odometry_msg;

        //! @brief Control modes stored as ros message
        mvp_msgs::msg::ControlModes m_control_modes;

        //! @brief Desired state of the system, set point of the controller
        mvp_msgs::msg::ControlProcess m_set_point_msg;

        //! @brief Current control mode
        std::string m_control_mode;

        //! @brief Protects odometry_msg during changes
        std::recursive_mutex m_odom_lock;

        /**
         * @brief Protects dynamic reconfigure server from dead locks
         * @todo In the future, boost might be replaced with std
         */
        boost::recursive_mutex m_config_lock;

        //! @brief Controller worker
        std::thread m_controller_worker;

        //! @brief A variable holds the controller status
        bool m_enabled;

        //! @brief Dynamic configure server for PID configuration
        // std::shared_ptr<dynamic_reconfigure::Server<mvp_control::PIDConfig>>
            // m_dynconf_pid_server;

        /** @brief Generates control allocation matrix from transform tree
         *
         *  This method is called if generator_type is 'tf'
         */
        void f_generate_control_allocation_from_tf();

        /**!
         * @brief Generates control allocation matrix
         * This method reads the thruster configuration and generates the
         * initial matrix. It meant to be called once.
         */
        void f_generate_control_allocation_matrix();

        /**
         * @brief
         * @return
         */
        bool f_update_control_allocation_matrix();
        /**
         * @brief initial tf check to make sure the tf is up running
         * @return
         */
        bool f_initial_tf_check();


        /** @brief Generate thrusters
         *
         */
        void f_generate_thrusters();

        /** @brief Reads ros param server and generates control modes
         *
         */
        void f_read_control_modes();
        void f_load_control_config();

        /** @brief Compute process values
         *
         */
        bool f_compute_process_values();

        /** @brief Control Loop
         *
         *
         */
        void f_control_loop();

        void f_update_osqp_matrix();

        void f_update_osqp_matrix_auto_direction();


        /** @brief Convert prq to world_frame angular rate:
         *  Eq.(2.12), Eq.(2.14) from Thor I. Fossen, Guidance and Control of Ocean Vehicles, Page 10
         *
         */
        Eigen::MatrixXd f_angular_velocity_transform(const geometry_msgs::msg::TransformStamped& tf);

        /** @brief Amends changes to Dynamic reconfigure server
         *
         * After reading the static configuration file, applies configuration to
         * dynamic reconfigure server.
         */
        void f_amend_dynconf();

        /** @brief Amends the control mode
         *
         * Changes the active control mode of the controller. Checks if request mode
         * is exist or not. Returns false if the operation requested is invalid.
         *
         * @param mode
         * @return
         */
        bool f_amend_control_mode(std::string mode);

        /** @brief Amends controllers set point
         *
         * Returns false if set point mode is invalid.
         * See #MvpControlROS::f_amend_control_mode
         *
         * @param set_point
         * @return
         */

        bool f_amend_set_point(const mvp_msgs::msg::ControlProcess::SharedPtr set_point);

        /** @brief Trivial subscriber
         *
         * @param msg
         */
        void f_cb_msg_odometry(const nav_msgs::msg::Odometry::SharedPtr msg);

        /** @brief Trivial set point callback
         *
         * @param msg
         */
        void f_cb_srv_set_point(
            const mvp_msgs::msg::ControlProcess::SharedPtr msg);

        /** @brief Trivial servo joint callback
         *
         * @param msg
         */
        void f_cb_servo_joint(const sensor_msgs::msg::JointState::SharedPtr msg);


        /** @brief Trivial thruster direction for vector thruster
         *
         * @param msg
         */
        void f_cb_vector_thruster_direction(const std_msgs::msg::Int16MultiArray::SharedPtr msg);


        /** @brief Dynamic reconfigure server callback
         *
         * @param config
         * @param level
         */
        // void f_cb_dynconf_pid(mvp_control::PIDConfig &config, uint32_t level);

        /** @brief Trivial ros service server callback for get control modes
         *
         * This service returns configured control modes to ros service client
         *
         * @param req
         * @param resp
         * @return Success of the operation
         */
        bool f_cb_srv_get_control_modes(
            const std::shared_ptr<mvp_msgs::srv::GetControlModes::Request> req,
            const std::shared_ptr<mvp_msgs::srv::GetControlModes::Response> resp);

        /** @brief Trivial ros service server callback for set control point
         *
         * @param req
         * @param resp
         * @return Success of the operation
         */
        bool f_cb_srv_set_control_point(
            const std::shared_ptr<mvp_msgs::srv::SetControlPoint::Request> req,
            const std::shared_ptr<mvp_msgs::srv::SetControlPoint::Response> resp);

        bool f_cb_srv_set_controller(
            const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
            const std::shared_ptr<std_srvs::srv::SetBool::Response> resp);

        /**
         * @brief get controller state
         *
         * @param req Trivial empty request
         * @param resp Trivial empty response
         * @return true
         * @return false
         */
        bool f_cb_srv_get_controller_state(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
            const std::shared_ptr<std_srvs::srv::Trigger::Response> resp);

        


        /**
         * @brief Get active control mode
         * @param req Empty request
         * @param resp Response with current control mode
         * @return true Always returns true
         */
        bool f_cb_srv_get_active_mode(
            const std::shared_ptr<mvp_msgs::srv::GetControlMode::Request> req,
            const std::shared_ptr<mvp_msgs::srv::GetControlMode::Response> resp);
    };

}