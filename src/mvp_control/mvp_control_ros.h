#pragma once

#include "thread"
#include "ros/ros.h"
#include "mvp_control.h"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_ros/transform_listener.h"

#include "std_msgs/Float32.h"
#include "std_srvs/Empty.h"
#include "nav_msgs/Odometry.h"
#include "dynamic_reconfigure/server.h"

/**
 * custom dynamic reconfigure
 */
#include "mvp_control/PIDConfig.h"

/**
 * Custom message types
 */
#include "mvp_control/PIDgains.h"
#include "mvp_control/ControlState.h"
#include "mvp_control/ControlModes.h"

/**
 * Custom services
 */
#include "mvp_control/GetControlMode.h"
#include "mvp_control/GetControlModes.h"
#include "mvp_control/SetControlPoint.h"


#include "Eigen/Dense"

#include "thruster_ros.h"

#include "boost/shared_ptr.hpp"
#include "boost/thread.hpp"
#include "boost/bind.hpp"

namespace ctrl {
/** @brief ROS wrapper for MvpControl
 *
 *  This package wraps mvp control class and utilizes its
 *  functionality.
 *
 *  @see MvpControl
 */
    class MvpControlROS {
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
        ros::NodeHandle m_nh;

        //! @brief Private node handler
        ros::NodeHandle m_pnh;

        //! @brief Thruster list
        std::vector<ThrusterROS::Ptr> m_thrusters;

        /**! @brief Control Allocation Matrix
         *
         *  Control allocation matrix is generated from individual
         *  configurations of the thrusters.
         */
        Eigen::MatrixXd m_control_allocation_matrix;

        //! @brief Control allocation matrix generator type
        GeneratorType m_generator_type;

        //! @brief Center of gravity link id
        std::string m_cg_link_id;

        //! @brief World link id
        std::string m_world_link_id;

        //! @brief Transform buffer for TF2
        tf2_ros::Buffer m_transform_buffer;

        //! @brief Transform listener for TF2
        tf2_ros::TransformListener m_transform_listener;

        //! @brief Transform prefix
        std::string m_tf_prefix;

        //! @brief Mvp Control object
        MvpControl::Ptr m_mvp_control;

        //! @brief System state
        Eigen::VectorXd m_system_state;

        //! @brief Desired state
        Eigen::VectorXd m_desired_state;

        //! @brief Controller frequency
        double m_controller_frequency;

        //! @brief Get control modes ros service server
        ros::ServiceServer m_get_control_modes_server;

        //! @brief Set control point ros service server
        ros::ServiceServer m_set_control_point_server;

        //! @brief Enable controller ros service server
        ros::ServiceServer m_enable_controller_server;

        //! @brief Disable controller ros service server
        ros::ServiceServer m_disable_controller_server;

        //! @brief Active mode getter ros service server
        ros::ServiceServer m_get_active_mode_server;

        //! @brief Trivial subscriber
        ros::Subscriber m_odometry_subscriber;

        //! @brief Current state publisher
        ros::Publisher m_current_state_publisher;

        //! @brief Desired state subscriber
        ros::Subscriber m_desired_state_subscriber;

        //! @brief Publishes error in the state
        ros::Publisher m_error_state_publisher;

        //! @brief Holder for latest odometry msg
        nav_msgs::Odometry m_odometry_msg;

        //! @brief Control modes stored as ros message
        mvp_control::ControlModes m_control_modes;

        //! @brief Desired state message
        mvp_control::ControlState m_desired_state_msg;

        //! @brief Current control mode
        std::string m_control_mode;

        //! @brief Protects odometry_msg during changes
        boost::recursive_mutex m_odom_lock;

        //! @brief Protects dynamic reconfigure server from dead locks
        boost::recursive_mutex m_config_lock;

        //! @brief Controller worker
        boost::thread m_controller_worker;

        //! @brief A variable holds the controller status
        bool m_enabled;

        //! @brief Dynamic configure server for PID configuration
        boost::shared_ptr<dynamic_reconfigure::Server<mvp_control::PIDConfig>>
            m_dynconf_pid_server;

        /** @brief Generates control allocation matrix from transform tree
         *
         *  This method is called if generator_type is 'tf'
         */
        void f_generate_control_allocation_from_tf();

        /** @brief Generates control allocation matrix from user input
         *
         *  This method is called if generator_type is 'user'
         */
        void f_generate_control_allocation_from_user();

        /**! @brief Generates control allocation matrix
         *
         */
        void f_generate_control_allocation_matrix();

        /** @brief Generate thrusters
         *
         */
        void f_generate_thrusters();

        /** @brief Reads ros param server and generates control modes
         *
         */
        void f_read_control_modes();

        /** @brief computes state of the system
         *
         */
        bool f_compute_state();

        /** @brief Control Loop
         *
         *
         */
        void f_control_loop();

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

        /** @brief Amends the desired stateUpdates
         *
         * Returns false if desired state mode is invalid.
         * See #MvpControlROS::f_amend_control_mode
         *
         * @param state
         * @return
         */
        bool f_amend_desired_state(const mvp_control::ControlState &state);

        /** @brief Trivial subscriber
         *
         * @param msg
         */
        void f_cb_msg_odometry(const nav_msgs::Odometry::ConstPtr &msg);

        /** @brief Trivial desired state callback
         *
         * @param msg
         */
        void f_cb_srv_desired_state(
            const mvp_control::ControlState::ConstPtr &msg);

        /** @brief Dynamic reconfigure server callback
         *
         * @param config
         * @param level
         */
        void f_cb_dynconf_pid(mvp_control::PIDConfig &config, uint32_t level);

        /** @brief Trivial ros service server callback for get control modes
         *
         * This service returns configured control modes to ros service client
         *
         * @param req
         * @param resp
         * @return Success of the operai_min = -10}, pid_sway = {kp = 0, ki = 0, kd = 0, k_i_max = 0, k_i_min = 0}, pid_heave = {kp = 0, ki = 0, kd = 0, k_i_max = 0, k_i_min = 0}}tion
         */
        bool f_cb_srv_get_control_modes(
            mvp_control::GetControlModes::Request &req,
            mvp_control::GetControlModes::Response &resp);

        /** @brief Trivial ros service server callback for set control point
         *
         * @param req
         * @param resp
         * @return Success of the operation
         */
        bool f_cb_srv_set_control_point(
            mvp_control::SetControlPoint::Request req,
            mvp_control::SetControlPoint::Response resp);

        /**
         * @brief Enables the controller
         *
         * @param req Trivial empty request
         * @param resp Trivial empty response
         * @return true
         * @return false
         */
        bool f_cb_srv_enable(
            std_srvs::Empty::Request req,
            std_srvs::Empty::Response resp);

        /**
         * @brief disable the controller
         *
         * @param req Trivial empty request
         * @param resp Trivial empty response
         * @return true
         * @return false
         */
        bool f_cb_srv_disable(
            std_srvs::Empty::Request req,
            std_srvs::Empty::Response resp);

        /**
         * @brief Get active control mode
         * @param req Empty request
         * @param resp Response with current control mode
         * @return true Always returns true
         */
        bool f_cb_srv_get_active_mode(
            mvp_control::GetControlMode::Request &req,
            mvp_control::GetControlMode::Response &resp);

    public:

        /** @brief Default constructor
         *
         */
        MvpControlROS();

        /** @brief Initializer for Mvp Control ROS
         *
         * This function initializes control allocation matrix and
         *
         */
        void initialize();

        //! @brief Generic typedef for shared pointer
        typedef boost::shared_ptr<MvpControlROS> Ptr;


    };

}