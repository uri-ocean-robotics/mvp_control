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
    Author: Mingxi ZHou
    Email: mzhou@uri.edu
    Year: 2024

    Copyright (C) 2024 Smart Ocean Systems Laboratory
*/

#include "mvp_control/mvp_control_ros.hpp"
#include "mvp_control/exception.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2/time.h"
#include "mvp_control/dictionary.hpp"

#include "boost/regex.hpp"
#include "yaml-cpp/yaml.h"
#include <chrono>
#include <functional>
#include <memory>



using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using GetControlModes = mvp_msgs::srv::GetControlModes;
using SetControlPoint = mvp_msgs::srv::SetControlPoint;
using GetControlMode = mvp_msgs::srv::GetControlMode;
using namespace ctrl;

using namespace std::chrono_literals;

MvpControlROS::MvpControlROS(std::string name) : Node(name)
{
    m_transform_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_transform_listener = std::make_unique<tf2_ros::TransformListener>(*m_transform_buffer);


    m_process_values = Eigen::VectorXd::Zero(CONTROLLABLE_DOF_LENGTH);

    m_set_point = Eigen::VectorXd::Zero(CONTROLLABLE_DOF_LENGTH);

    //ROS Params
    /**
     * Read basic configuration. Configuration regarding to thruster allocation
     * will be read later.
     */
    // // Read configuration: enabled
    this->declare_parameter(CONF_ENABLED, false);
    this->get_parameter(CONF_ENABLED, m_enabled);

    // // Read configuration: tf prefix
    std::string tf_prefix;
    this->declare_parameter(CONF_TF_PREFIX, CONF_TF_PREFIX_DEFAULT);
    this->get_parameter(CONF_TF_PREFIX, tf_prefix);
    m_tf_prefix = tf_prefix.empty() ? CONF_TF_PREFIX_DEFAULT : tf_prefix + "/";

    //DEFAULT World and child frame for initial tf checking only 
    std::string child_link_initial;
    this->declare_parameter(CONF_CHILD_LINK_INITIAL, "");
    this->get_parameter(CONF_CHILD_LINK_INITIAL, child_link_initial);
    m_child_link_id_initial = m_tf_prefix + child_link_initial;

    std::string world_link_initial;
    this->declare_parameter(CONF_WORLD_LINK_INITIAL, "");
    this->get_parameter(CONF_WORLD_LINK_INITIAL, world_link_initial);
    m_world_link_id_initial = m_tf_prefix + world_link_initial;

    //set the defaul world link and child link used in the controller
    m_child_link_id = m_child_link_id_initial;
    m_world_link_id = m_world_link_id_initial;

    // Read configuration: odometry topic id
    std::string odometry_topic;
    this->declare_parameter(CONF_ODOMETRY_SOURCE, CONF_ODOMETRY_SOURCE_DEFAULT);
    this->get_parameter(CONF_ODOMETRY_SOURCE, odometry_topic);

    this->declare_parameter(CONF_CONTROLLER_FREQUENCY, 10.0);
    this->get_parameter(CONF_CONTROLLER_FREQUENCY, m_controller_frequency);

    this->declare_parameter(CONF_NO_SETPOINT_TIMEOUT, 10.0);
    this->get_parameter(CONF_NO_SETPOINT_TIMEOUT, m_no_setpoint_timeout);

    this->declare_parameter("auto_vector_direction", true);
    this->get_parameter("auto_vector_direction", m_vector_thruster_auto_direction);

    this->declare_parameter("total_force_cost_factor", 0.0);
    this->get_parameter("total_force_cost_factor", m_total_force_cost_factor);

    //control config file location
    this->declare_parameter("config_file", "control.yaml");
    this->get_parameter("config_file", m_control_config_file);

    //tf generate type
    this->declare_parameter(CONF_GENERATOR_TYPE, CONF_GENERATOR_TYPE_OPT_TF);
    this->get_parameter(CONF_GENERATOR_TYPE, generator_type);

    this->declare_parameter("use_restoring_effort", false);
    this->get_parameter("use_restoring_effort", m_restoring_effort_flag);

    //gravity and buoyancy param
    if (m_restoring_effort_flag)
    {
        this->declare_parameter("gravity", 0.0);
        this->get_parameter("gravity", m_gravity);

        this->declare_parameter("gravity_link", "cg_link");
        this->get_parameter("gravity_link", m_gravity_link);
        m_gravity_link = m_tf_prefix + m_gravity_link;

        this->declare_parameter("buoyancy", 0.0);
        this->get_parameter("buoyancy", m_buoyancy);

        this->declare_parameter("buoyancy_link", "cb_link");
        this->get_parameter("buoyancy_link", m_buoyancy_link);
        m_buoyancy_link = m_tf_prefix + m_buoyancy_link;

    }
    //End of ROS Params

    /**
     * Initialize Subscribers
     */
    m_odometry_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
                                odometry_topic, 
                                10, 
                                std::bind(&MvpControlROS::f_cb_msg_odometry, this, _1) 
                                );

    m_set_point_subscriber = this->create_subscription<mvp_msgs::msg::ControlProcess>(
                                TOPIC_CONTROL_PROCESS_SET_POINT, 
                                10, 
                                std::bind(&MvpControlROS::f_cb_srv_set_point, this, _1) 
                                );

    m_servo_joint_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
                                    "servo_joint_topic", 10, 
                                    std::bind(&MvpControlROS::f_cb_servo_joint, this, _1));

    m_vector_thruster_direction_subscriber = this->create_subscription<std_msgs::msg::Int16MultiArray>(
                                                    "vector_thruster_direction", 2,
                                                    std::bind(&MvpControlROS::f_cb_vector_thruster_direction, this, _1));
                                                    
    
    /**
     * Initialize publishers
     */
    m_process_value_publisher = this->create_publisher<mvp_msgs::msg::ControlProcess>(TOPIC_CONTROL_PROCESS_VALUE, 10);
    m_process_error_publisher = this->create_publisher<mvp_msgs::msg::ControlProcess>(TOPIC_CONTROL_PROCESS_ERROR, 10);
    m_controller_state_publisher = this->create_publisher<std_msgs::msg::Bool>(TOPIC_CONTROLLER_STATE, 10);

    m_d_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("controller/process/d_value", 10);
    m_v_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("controller/process/v_value", 10);
    m_i_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("controller/process/i_value", 10);
    m_p_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("controller/process/p_value", 10);


    /**
     * Initialize services
     */
    m_get_control_modes_server = this->create_service<GetControlModes>(
        SERVICE_GET_CONTROL_MODES,
        std::bind(&MvpControlROS::f_cb_srv_get_control_modes, this, _1, _2)
        );

    m_set_control_point_server = this->create_service<SetControlPoint>(
        SERVICE_SET_CONTROL_POINT,
        std::bind(&MvpControlROS::f_cb_srv_set_control_point, this, _1, _2)
        );

    m_set_controller_server = this->create_service<std_srvs::srv::SetBool>(
        SERVICE_SET_CONTROLLER,
        std::bind(&MvpControlROS::f_cb_srv_set_controller, this, _1, _2)
    );

    m_get_controller_state_server = this->create_service<std_srvs::srv::Trigger>(
        SERVICE_GET_CONTROLLER_STATE,
        std::bind(&MvpControlROS::f_cb_srv_get_controller_state, this, _1, _2)
        );


    m_get_active_mode_server = this->create_service<GetControlMode>(
        SERVICE_GET_ACTIVE_MODE,
        std::bind(&MvpControlROS::f_cb_srv_get_active_mode, this, _1, _2)
        );

    // /**
    //  * Initialize the actual controller
    //  */
    m_mvp_control.reset(new MvpControl());

    //set integral terms to zero
    //set the integral terms to zeros
    Eigen::VectorXd m_i(CONTROLLABLE_DOF_LENGTH);
    m_i.setZero();
    m_mvp_control->get_pid()->set_m_i(m_i);
    // m_i = m_mvp_control->get_pid()->get_m_i();

}


bool MvpControlROS::f_cb_srv_get_controller_state(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
        const std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
    resp->success = true;
    if(m_enabled){
         resp->message = "enabled";
    }
    else{
        resp->message = "disabled";
    }

    return true;
}


void MvpControlROS::f_generate_control_allocation_matrix() {

    // Read generator type
    // Parse the control allocation generator type and save it as enum type
    if(generator_type == CONF_GENERATOR_TYPE_OPT_TF) {
        m_generator_type = GeneratorType::TF;
        f_generate_control_allocation_from_tf();
    } else if (generator_type == CONF_GENERATOR_TYPE_OPT_USER) {
        m_generator_type = GeneratorType::USER;
        throw control_ros_exception(
            "this option is depreciated please use the TF option"
        );
    } else {
        m_generator_type = GeneratorType::UNKNOWN;
        throw control_ros_exception(
            "control allocation generation method unspecified"
        );
    }

    // Conduct some checks to see if everything is ready to be initialized
    if(m_thrusters.empty()) {
        // throw control_ros_exception("no thruster specified");
        RCLCPP_WARN_STREAM(this->get_logger(), "!!! No thruster specified !!!");

    }
    else{
        // Control allocation matrix is generated based on each thruster. Each
        // thruster must have equal number of elements in their contribution matrix.
        // Code below checks the validity of the contribution vectors for each
        // thruster.
        for(unsigned int i = 0 ; i < m_thrusters.size() - 1 ; i++ ) {
            if (m_thrusters[i]->get_contribution_vector().size() !=
                m_thrusters[i + 1]->get_contribution_vector().size()) {
                throw control_ros_exception(
                    "contribution vector sizes doesn't match"
                );
            }
        }

    }
    
   if(m_vector_thrusters.empty()) {
        // throw control_ros_exception("no vector thruster specified");
        RCLCPP_WARN_STREAM(this->get_logger(), "!!! No vector thruster specified !!!");
    }
    else{
        //vector thruster
        for(unsigned int i = 0 ; i < m_vector_thrusters.size() - 1 ; i++ ) {
            if (m_vector_thrusters[i]->get_contribution_vector().size() !=
                m_vector_thrusters[i + 1]->get_contribution_vector().size()) {
                throw control_ros_exception(
                    "contribution vector sizes doesn't match"
                );
            }
        }
    }

    if(m_thrusters.empty()&&m_vector_thrusters.empty())
    {
        throw control_ros_exception("no thruster specified");

    }
    // Initialize the control allocation matrix based on zero matrix.
    // M by N matrix. M -> number of all controllable DOF, N -> number of
    // thrusters
    //total column number is N_thruster + 2*N_vector_thruster
    int col_num;
    if(m_vector_thrusters.empty())
    {
        col_num = (int) m_thrusters.size();
    }
    else{
        col_num = (int) m_thrusters.size() + 
                (int)m_vector_thrusters[0]->get_thruster_force_count()*(int)m_vector_thrusters.size();
    }  
    m_control_allocation_matrix = Eigen::MatrixXd::Zero(CONTROLLABLE_DOF_LENGTH, col_num);

    m_direction_cost_matrix = Eigen::MatrixXd::Zero(CONTROLLABLE_DOF_LENGTH, col_num);
    m_direction_cost_matrix_c = Eigen::VectorXd::Zero(col_num);
    
    // Until this point, all the allocation matrix related issued must be
    // solved or exceptions thrown.

    printf("passing control allocation values\r\n");

    // Acquire DOF per actuator. Register it to control allocation matrix.
    // Only DOF::X, DOF::Y and DOF::Z are left unregistered. They are computed
    // online after each iteration.
    int count = 0;

    for (uint64_t i = 0; i < m_thrusters.size(); i++) {
        for(const auto& j :
            {DOF::X, DOF::Y, DOF::Z, 
             DOF::ROLL, DOF::PITCH, DOF::YAW,
             DOF::U, DOF::V, DOF::W,
             DOF::P, DOF::Q, DOF::R
             })
        {
            m_control_allocation_matrix(j, i) = m_thrusters[i]->get_contribution_vector()(j);
        }
        
    }

    count = m_thrusters.size();
    //vector thruster
    for (uint64_t i = 0; i < m_vector_thrusters.size(); i++) {

        // std::cout << "Matrix size: " << m_control_allocation_matrix.rows() << " x " << m_control_allocation_matrix.cols() << std::endl;
        // std::cout << "Matrix size: " << m_vector_thrusters[i]->get_contribution_vector().rows() << " x " << m_vector_thrusters[i]->get_contribution_vector().cols() << std::endl;
        // printf("count =%d\r\n", count);
        for(const auto& j :
            {DOF::X, DOF::Y, DOF::Z, 
             DOF::ROLL, DOF::PITCH, DOF::YAW,
             DOF::U, DOF::V, DOF::W,
             DOF::P, DOF::Q, DOF::R
             })
        {
            // each thruster has two rows
            m_control_allocation_matrix(j, count) = m_vector_thrusters[i]->get_contribution_vector()(j, 0);
            m_control_allocation_matrix(j, count+1) = m_vector_thrusters[i]->get_contribution_vector()(j, 1);

            if(m_vector_thruster_auto_direction)
            {
                m_control_allocation_matrix(j, count+2) = m_vector_thrusters[i]->get_contribution_vector()(j, 0);
                // m_control_allocation_matrix(j, count+3) = 0*m_vector_thrusters[i]->get_contribution_vector()(j, 0);
               
            }

        }


        if(m_vector_thruster_auto_direction)
        {
            // m_direction_cost_matrix(count + 3, count + 3) = 1;
            // m_direction_cost_matrix_c(count+3) = 1;
           
        }
        count = count + (int)m_vector_thrusters[i]->get_thruster_force_count();
        
    }

    printf("allocation matrix initialized\r\n");

    
    // std::cout<< m_control_allocation_matrix<<std::endl;

    // Finally, set the control allocation matrix for the controller object.
    m_mvp_control->set_control_allocation_matrix(m_control_allocation_matrix);
    m_mvp_control->set_direction_cost_matrix(m_direction_cost_matrix);
    m_mvp_control->set_direction_cost_matrix_c(m_direction_cost_matrix_c);
    m_mvp_control->set_total_force_cost_factor(m_total_force_cost_factor);

}


//this function is called whenever the frame has changedin the setpoint.
void MvpControlROS::f_generate_control_allocation_from_tf() {
    // double time; //used for tf lookup
    auto steady_clock = rclcpp::Clock();
    rclcpp::Time now = this->get_clock()->now();

    // For each thruster look up transformation
    for(const auto& t : m_thrusters) {
        now = this->get_clock()->now();

        Eigen::VectorXd contribution_vector(CONTROLLABLE_DOF_LENGTH);
        

        try {
            Eigen::Isometry3d eigen_tf;
            /////////////////////////////////////////////////////////
            /////////////Local motion///////////////////////////////
            /////////////////////////////////////////////////////////
            //find the tf between thruster and the child link
            geometry_msgs::msg::TransformStamped tf_cg_thruster = m_transform_buffer->lookupTransform(
                m_child_link_id,
                t->get_link_id(),
                tf2::TimePointZero,
                10ms
                );
                
            /////////////////////////////////////////////////////////
            //update the contribution matrix element for U,V,W///////
            
            eigen_tf = tf2::transformToEigen(tf_cg_thruster);

            // thruster only use a axis (e.g., X-axis) for forece
            //R*[1 0 0], so we only get the first element in each row for thrusters.
            Eigen::Vector3d f_t(1.0, 0.0, 0.0); //thruster only generate force in x axis in thruster frame
            // F = R*F_t
            Eigen::Vector3d f_uvw = eigen_tf.rotation() * f_t;
            contribution_vector(DOF::U) = f_uvw.x();
            contribution_vector(DOF::V) = f_uvw.y();
            contribution_vector(DOF::W) = f_uvw.z();
            
            /////////////////////////////////////////////////////////
            //update the contribution matrix element for P, Q, R////

            auto trans_xyz = eigen_tf.translation();
            auto t_pqr = trans_xyz.cross(f_uvw);
            // body frame p,q,r
            contribution_vector(DOF::P) = t_pqr.x();
            contribution_vector(DOF::Q) = t_pqr.y();
            contribution_vector(DOF::R) = t_pqr.z();

            /////////////////////////////////////////////////////////
            /////////////world frame motion//////////////////////////
            /////////////////////////////////////////////////////////
            //get tf between child and world frame
            ////////earth frame 
            now = this->get_clock()->now();
            geometry_msgs::msg::TransformStamped tf_child_world = m_transform_buffer->lookupTransform(
                m_world_link_id,
                m_child_link_id,
                tf2::TimePointZero,
                10ms
            );

            
            /////////////////////////////////////////////////////////
            //update the contribution matrix element for x,y,z///////
            ////////////////////////////////////////////////////////
            eigen_tf = tf2::transformToEigen(tf_child_world);
            Eigen::Vector3d f_xyz = eigen_tf.rotation() * f_uvw;

            contribution_vector(DOF::X) = f_xyz.x();
            contribution_vector(DOF::Y) = f_xyz.y();
            contribution_vector(DOF::Z) = f_xyz.z();

            /////////////////////////////////////////////////////////
            //update the contribution matrix element for euler angle///////
            ////////////////////////////////////////////////////////
             //! Eq.(2.12), Eq.(2.14) from Thor I. Fossen, Guidance and Control of Ocean Vehicles, Page 10
             //get relative orientation for angular transform matrix
            Eigen::Matrix3d ang_vel_tranform = f_angular_velocity_transform(tf_child_world);

            auto t_rpy = ang_vel_tranform * t_pqr;
        
            contribution_vector(DOF::ROLL) = t_rpy.x();
            contribution_vector(DOF::PITCH) = t_rpy.y();
            contribution_vector(DOF::YAW) = t_rpy.z();
        

            t->set_contribution_vector(contribution_vector);

        } catch (const tf2::TransformException & e) {
            RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), steady_clock, 1000, 
                                        std::string("mvp_control allocation matrix generation error:") + e.what());
          return;

        }
 
    }
    printf("#########regular thruster allocation generated ##############\r\n");
    //vectoor thruster initial allocation matrix
    for(const auto& t : m_vector_thrusters) {
        now = this->get_clock()->now();

        Eigen::MatrixXd contribution_vector(CONTROLLABLE_DOF_LENGTH, 2);

        try {
            Eigen::Isometry3d eigen_tf;
            /////////////////////////////////////////////////////////
            /////////////Local motion///////////////////////////////
            /////////////////////////////////////////////////////////
            //find the tf between thruster and the child link
            geometry_msgs::msg::TransformStamped tf_cg_thruster = m_transform_buffer->lookupTransform(
                m_child_link_id,
                t->get_link_id(),
                tf2::TimePointZero,
                10ms
                );
            
            /////////////////////////////////////////////////////////
            //update the contribution matrix element for U,V,W///////
            eigen_tf = tf2::transformToEigen(tf_cg_thruster);

            // thruster only use a axis (e.g., X-axis) for forece
            //for Fx 
            Eigen::Vector3d fx(1.0, 0.0, 0.0); //thruster only generate force in x axis in thruster frame
            Eigen::Vector3d fy(0.0, 1.0, 0.0); //thruster only generate force in x axis in thruster frame

            // F = R*F_t
            Eigen::Vector3d fx_uvw = eigen_tf.rotation() * fx;
            Eigen::Vector3d fy_uvw = eigen_tf.rotation() * fy;

            contribution_vector(DOF::U, 0) = fx_uvw.x();
            contribution_vector(DOF::V, 0) = fx_uvw.y();
            contribution_vector(DOF::W, 0) = fx_uvw.z();
            contribution_vector(DOF::U, 1) = fy_uvw.x();
            contribution_vector(DOF::V, 1) = fy_uvw.y();
            contribution_vector(DOF::W, 1) = fy_uvw.z();

            /////////////////////////////////////////////////////////
            //update the contribution matrix element for P, Q, R////
            auto trans_xyz = eigen_tf.translation();
            auto tx_pqr = trans_xyz.cross(fx_uvw);
            auto ty_pqr = trans_xyz.cross(fy_uvw);
            // body frame p,q,r
            contribution_vector(DOF::P, 0) = tx_pqr.x();
            contribution_vector(DOF::Q, 0) = tx_pqr.y();
            contribution_vector(DOF::R, 0) = tx_pqr.z();
            contribution_vector(DOF::P, 1) = ty_pqr.x();
            contribution_vector(DOF::Q, 1) = ty_pqr.y();
            contribution_vector(DOF::R, 1) = ty_pqr.z();
            
            /////////////////////////////////////////////////////////
            /////////////world frame motion//////////////////////////
            /////////////////////////////////////////////////////////
            //get tf between child and world frame
            ////////earth frame 
            now = this->get_clock()->now();
            geometry_msgs::msg::TransformStamped tf_child_world = m_transform_buffer->lookupTransform(
                m_world_link_id,
                m_child_link_id,
                tf2::TimePointZero,
                10ms
            );
            /////////////////////////////////////////////////////////
            //update the contribution matrix element for x,y,z///////
            ////////////////////////////////////////////////////////
            eigen_tf = tf2::transformToEigen(tf_child_world);
            Eigen::Vector3d fx_xyz = eigen_tf.rotation() * fx_uvw;
            Eigen::Vector3d fy_xyz = eigen_tf.rotation() * fy_uvw;
            contribution_vector(DOF::X, 0) = fx_xyz.x();
            contribution_vector(DOF::Y, 0) = fx_xyz.y();
            contribution_vector(DOF::Z, 0) = fx_xyz.z();
            contribution_vector(DOF::X, 1) = fy_xyz.x();
            contribution_vector(DOF::Y, 1) = fy_xyz.y();
            contribution_vector(DOF::Z, 1) = fy_xyz.z();

            /////////////////////////////////////////////////////////
            //update the contribution matrix element for euler angle///////
            ////////////////////////////////////////////////////////
            //! Eq.(2.12), Eq.(2.14) from Thor I. Fossen, Guidance and Control of Ocean Vehicles, Page 10
            //get relative orientation for angular transform matrix
            Eigen::Matrix3d ang_vel_tranform = f_angular_velocity_transform(tf_child_world);

            auto tx_rpy = ang_vel_tranform * tx_pqr;
            auto ty_rpy = ang_vel_tranform * ty_pqr;
        
            contribution_vector(DOF::ROLL, 0) = tx_rpy.x();
            contribution_vector(DOF::PITCH, 0) = tx_rpy.y();
            contribution_vector(DOF::YAW, 0) = tx_rpy.z();
            contribution_vector(DOF::ROLL, 1) = ty_rpy.x();
            contribution_vector(DOF::PITCH, 1) = ty_rpy.y();
            contribution_vector(DOF::YAW, 1) = ty_rpy.z();

            t->set_contribution_vector(contribution_vector);


        } catch (const tf2::TransformException & e) {
            RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), steady_clock, 1000, 
                                        std::string("mvp_control allocation matrix generation error:") + e.what());
        return;

        }
    }
    printf("#########vector thruster allocation generated ##############\r\n");


}

bool MvpControlROS::f_initial_tf_check(){
    auto steady_clock = rclcpp::Clock();
    
    RCLCPP_INFO_STREAM(this->get_logger(), "MVP_control_node initial TF checking");
    // printf("###### thruster numer = %d, vector thruster number = %d #########\r\n", m_thrusters.size(), m_vector_thrusters.size());
    //check world link to cg link is up
    try {
            // Transform center of gravity to world
            geometry_msgs::msg::TransformStamped tf_torque = m_transform_buffer->lookupTransform(
                m_world_link_id_initial,
                m_child_link_id_initial,
                tf2::TimePointZero,
                10ms
            );

        } catch(tf2::TransformException &e) {
            RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), steady_clock, 1000, std::string("Can't find TF between world and cg: ") + e.what());
            return false;
        }
    RCLCPP_INFO_STREAM(this->get_logger(), "initial world_link to child_link found");

    //check thruster to cg_link is up.
    // For each thruster look up transformation
    RCLCPP_INFO_STREAM(this->get_logger(), "checking regular thrusters");
    for(const auto& t : m_thrusters) {
        try {
            geometry_msgs::msg::TransformStamped tf_cg_thruster = m_transform_buffer->lookupTransform(
                m_child_link_id_initial,
                t->get_link_id(),
                tf2::TimePointZero,
                10ms
                );

        } catch (const tf2::TransformException & e) {
            RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), steady_clock, 1000, std::string("Can't find TF for thrusters: ") + e.what());
            RCLCPP_INFO( this->get_logger(), "Could not transform %s to %s: %s",
                         t->get_link_id().c_str(), m_child_link_id_initial.c_str(), e.what() ); 
          return false;
        }
    }
    
    RCLCPP_INFO_STREAM(this->get_logger(), "checking vector thrusters");
    //checking vector thrusters
    for(const auto& t : m_vector_thrusters) {
        try {
            geometry_msgs::msg::TransformStamped tf_cg_thruster = m_transform_buffer->lookupTransform(
                m_child_link_id_initial,
                t->get_link_id(),
                tf2::TimePointZero,
                10ms
                );

        } catch (const tf2::TransformException & e) {
            RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), steady_clock, 1000, std::string("Can't find TF for thrusters: ") + e.what());
            RCLCPP_INFO( this->get_logger(), "Could not transform %s to %s: %s",
                        t->get_link_id().c_str(), m_child_link_id_initial.c_str(), e.what() ); 
        return false;
        }
    }

    if(m_restoring_effort_flag)
    {
        //checking restoring force and moment
        try {
            geometry_msgs::msg::TransformStamped tf_cg = m_transform_buffer->lookupTransform(
                m_world_link_id_initial,
                m_gravity_link,
                tf2::TimePointZero,
                10ms
                );

        } catch (const tf2::TransformException & e) {
            RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), steady_clock, 1000, std::string("Can't find TF for gravity: ") + e.what());
            RCLCPP_INFO( this->get_logger(), "Could not transform %s to %s: %s",
                    m_gravity_link.c_str(), m_world_link_id_initial.c_str(), e.what() ); 
        }

        try {
            geometry_msgs::msg::TransformStamped tf_cb = m_transform_buffer->lookupTransform(
                m_world_link_id_initial,
                m_buoyancy_link,
                tf2::TimePointZero,
                10ms
                );

        } catch (const tf2::TransformException & e) {
            RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), steady_clock, 1000, std::string("Can't find TF for gravity: ") + e.what());
            RCLCPP_INFO( this->get_logger(), "Could not transform %s to %s: %s",
                m_buoyancy_link.c_str(), m_world_link_id_initial.c_str(), e.what() ); 
        }
    }
    printf("setting restoring matrix\r\n");
    Eigen::VectorXd m_restore_matrix;
    m_restore_matrix =  Eigen::VectorXd::Zero(CONTROLLABLE_DOF_LENGTH);

    m_mvp_control->set_restoring_force_matrix(m_restore_matrix);
    printf("setting restoring matrix\r\n");

    return true;

}

void MvpControlROS::initialize() {

    // Read configured control modes from the ROS parameter server
    f_load_control_config();
    RCLCPP_INFO(this->get_logger(), "#### Control config file %s loaded ####", m_control_config_file.c_str());

    // Initialize thruster objects.
    std::for_each(m_thrusters.begin(),m_thrusters.end(),
        [](const ThrusterROS::Ptr& t){
            t->initialize();
        }
    );

    //initialize vector thruster objects
    std::for_each(m_vector_thrusters.begin(),m_vector_thrusters.end(),
        [](const VectorThrusterROS::Ptr& t){
            t->initialize();
        }
    );

    RCLCPP_INFO_STREAM(this->get_logger(), "#### Thruster initialized ####");

    // Generate thrusters with the given configuration
    while(!f_initial_tf_check())
    {
        sleep(1);
    };

    RCLCPP_INFO_STREAM(this->get_logger(), "#### TF checking done ####");

    // Generate control allocation matrix with defined method
    f_generate_control_allocation_matrix();

    RCLCPP_INFO_STREAM(this->get_logger(), "#### Allocation matrix generated ####");

    m_mvp_control->set_desired_state(m_set_point);

    m_mvp_control->set_system_state(m_process_values);

    m_controller_worker = std::thread([this] { f_control_loop(); });

    m_controller_worker.detach();

    printf("#### MVP Control initialized #####\r\n ");

}

//only update earth frame related elements
bool MvpControlROS::f_update_control_allocation_matrix() {

    // update control allocation based on actuators as well
    //thruster allocation in local frame is already determined from f_generate_allocation function when the frame has changed.
    //here we only update the earth frame related elements.
    rclcpp::Time now = this->get_clock()->now();

    try {
        // update the thruster allocation for X,Y,Z, and U,V,W
        geometry_msgs::msg::TransformStamped cg_world = m_transform_buffer->lookupTransform(
            m_world_link_id,
            m_child_link_id,
            tf2::TimePointZero,
            10ms
        );

        auto tf_eigen = tf2::transformToEigen(cg_world); //used for rotating linear velocity

        //rotation matrix for angular stuff
        Eigen::Matrix3d ang_vel_tranform = f_angular_velocity_transform(cg_world);

        // for each thruster compute contribution in earth frame
        for (int j =0; j<m_thrusters.size(); j ++)
        {
            Eigen::Vector3d f_uvw;
            f_uvw <<
                m_control_allocation_matrix(DOF::U, j),
                m_control_allocation_matrix(DOF::V, j),
                m_control_allocation_matrix(DOF::W, j);

            //F_xyz = R*F_uwv
            Eigen::Vector3d f_xyz = tf_eigen.rotation() * f_uvw;

            m_control_allocation_matrix(DOF::X, j) = f_xyz.x()*m_thrusters[j]->m_xyz_flag_vector.x();
            m_control_allocation_matrix(DOF::Y, j) = f_xyz.y()*m_thrusters[j]->m_xyz_flag_vector.y();
            m_control_allocation_matrix(DOF::Z, j) = f_xyz.z()*m_thrusters[j]->m_xyz_flag_vector.z();
            
            // Convert prq to world_frame angular rate:
            //  Eq.(2.12), Eq.(2.14) from Thor I. Fossen, Guidance and Control of Ocean Vehicles, Page 10
            Eigen::Vector3d t_pqr;
            t_pqr <<
                m_control_allocation_matrix(DOF::P, j),
                m_control_allocation_matrix(DOF::Q, j),
                m_control_allocation_matrix(DOF::R, j);                


            auto t_rpy = ang_vel_tranform * t_pqr;
            m_control_allocation_matrix(DOF::ROLL, j) = t_rpy.x();
            m_control_allocation_matrix(DOF::PITCH, j) = t_rpy.y();
            m_control_allocation_matrix(DOF::YAW, j) = t_rpy.z();             
        }


        int count = m_thrusters.size();

        ///vector thrusters
    
        for (int j = 0; j<m_vector_thrusters.size(); j++)
        {
            Eigen::Isometry3d eigen_local;
     
            geometry_msgs::msg::TransformStamped tf_cg_thruster = m_transform_buffer->lookupTransform(
                m_child_link_id,
                m_vector_thrusters[j]->get_link_id(),
                tf2::TimePointZero,
                10ms
                );
            eigen_local = tf2::transformToEigen(tf_cg_thruster);

            // thruster only use a axis (e.g., X-axis) for forece
            //for Fx 
            Eigen::Vector3d fx(1.0, 0.0, 0.0); //thruster only generate force in x axis in thruster frame
            Eigen::Vector3d fy(0.0, 1.0, 0.0); //thruster only generate force in x axis in thruster frame

            // F = R*F_t
            Eigen::Vector3d fx_uvw = eigen_local.rotation() * fx;
            Eigen::Vector3d fy_uvw = eigen_local.rotation() * fy;

            m_control_allocation_matrix(DOF::U, count) = fx_uvw.x();
            m_control_allocation_matrix(DOF::V, count) = fx_uvw.y();
            m_control_allocation_matrix(DOF::W, count) = fx_uvw.z();
            m_control_allocation_matrix(DOF::U, count+1) = fy_uvw.x();
            m_control_allocation_matrix(DOF::V, count+1) = fy_uvw.y();
            m_control_allocation_matrix(DOF::W, count+1) = fy_uvw.z();

            /////////////////////////////////////////////////////////
            //update the contribution matrix element for P, Q, R////
            auto trans_xyz = eigen_local.translation();
            auto tx_pqr = trans_xyz.cross(fx_uvw);
            auto ty_pqr = trans_xyz.cross(fy_uvw);
            // body frame p,q,r
            m_control_allocation_matrix(DOF::P, count) = tx_pqr.x();
            m_control_allocation_matrix(DOF::Q, count) = tx_pqr.y();
            m_control_allocation_matrix(DOF::R, count) = tx_pqr.z();
            m_control_allocation_matrix(DOF::P, count+1) = ty_pqr.x();
            m_control_allocation_matrix(DOF::Q, count+1) = ty_pqr.y();
            m_control_allocation_matrix(DOF::R, count+1) = ty_pqr.z();

            //world frame

            Eigen::Vector3d fx_xyz = tf_eigen.rotation() * fx_uvw;
            Eigen::Vector3d fy_xyz = tf_eigen.rotation() * fy_uvw;

            m_control_allocation_matrix(DOF::X, count) = fx_xyz.x();
            m_control_allocation_matrix(DOF::Y, count) = fx_xyz.y();
            m_control_allocation_matrix(DOF::Z, count) = fx_xyz.z();
            // m_control_allocation_matrix(DOF::Z, count) = 0; //remove vector thruster on depth
            
            // printf("thruster->%s = ", m_vector_thrusters[j]->get_link_id().c_str());
            // std::cout<<fx_xyz<<std::endl;

            m_control_allocation_matrix(DOF::X, count+1) = fy_xyz.x();
            m_control_allocation_matrix(DOF::Y, count+1) = fy_xyz.y();
            m_control_allocation_matrix(DOF::Z, count+1) = fy_xyz.z(); 
            // m_control_allocation_matrix(DOF::Z, count+1) = 0;   //remove vector thruster on depth

            // Convert prq to world_frame angular rate:

            auto tx_rpy = ang_vel_tranform * tx_pqr;
            auto ty_rpy = ang_vel_tranform * ty_pqr;
            
            m_control_allocation_matrix(DOF::ROLL, count) = tx_rpy.x();
            m_control_allocation_matrix(DOF::PITCH, count) = tx_rpy.y();
            m_control_allocation_matrix(DOF::YAW, count) = tx_rpy.z();      

            m_control_allocation_matrix(DOF::ROLL, count+1) = ty_rpy.x();
            m_control_allocation_matrix(DOF::PITCH, count+1) = ty_rpy.y();
            m_control_allocation_matrix(DOF::YAW, count+1) = ty_rpy.z();  

            if(m_vector_thruster_auto_direction)
            {
                m_control_allocation_matrix.col(count + 2) = m_control_allocation_matrix.col(count); //Fx- has the same allocation as Fx+
                // m_control_allocation_matrix.col(count + 3) = 0*m_control_allocation_matrix.col(count);
            }
            count = count + m_vector_thrusters[j]->get_thruster_force_count();

        }


    } catch(tf2::TransformException& e) {
        auto steady_clock = rclcpp::Clock();
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), steady_clock, 1000, std::string("Can't update control allocation matrix ") + e.what());
        return false;
    }


    m_mvp_control->update_control_allocation_matrix(
        m_control_allocation_matrix
    );

    // if(m_vector_thruster_auto_direction)
    // {
        // printf("auto direction \r\n");
    f_update_osqp_matrix_auto_direction();
    // }
    // else{
    //     f_update_osqp_matrix();
    // }

    

    return true;
}

void MvpControlROS::f_update_osqp_matrix_auto_direction()
{
    int constraints_per_thruster, forces_per_thruster;
    if (m_vector_thrusters.empty())
    {
        constraints_per_thruster = 0;
        forces_per_thruster = 0;
    }
    else{
        constraints_per_thruster= m_vector_thrusters[0]->get_thruster_constraint_count();
        forces_per_thruster = m_vector_thrusters[0]->get_thruster_force_count();
    }
     
    int row_num;  //number of constraints
    int col_num;  //number of forces

    row_num = m_thrusters.size() + constraints_per_thruster*m_vector_thrusters.size(); //each vector thruster has 4 constraints
    col_num = m_thrusters.size() + forces_per_thruster*m_vector_thrusters.size(); //each vector thruster has 2 forces
    ///prepare OSQP matrix
    Eigen::VectorXd upper_limit(row_num);
    Eigen::VectorXd lower_limit(row_num);
    Eigen::SparseMatrix<double> constraints_matrix(row_num,col_num);
    constraints_matrix.setZero();

    double P_INFINITY =  std::numeric_limits<double>::infinity();
    double N_INFINITY = -std::numeric_limits<double>::infinity();

    int row_count=0;
    int col_count = 0;
    //regular thrusters
    for(uint64_t i = 0 ; i < m_thrusters.size() ; i++) {
        upper_limit[row_count] = std::min(m_thrusters[i]->m_force_max, m_thrusters[i]->m_current_force + m_thrusters[i]->m_force_delta_limit);
        lower_limit[row_count] = std::max(m_thrusters[i]->m_force_min, m_thrusters[i]->m_current_force - m_thrusters[i]->m_force_delta_limit);
        

        constraints_matrix.insert(row_count, col_count) = 1; //diagnoal element set to 1
        row_count ++;
        col_count ++;
    }

    //vector thrusters
    double alpha_u;
    double alpha_l;    
    
    //vector thrusters
    for(uint64_t i = 0; i<m_vector_thrusters.size(); i ++){
        //find the alpha u and alpah l;
        alpha_u = std::min(m_vector_thrusters[i]->m_servo_angle_max - m_vector_thrusters[i]->get_thruster_servo_angle(), 
                            m_vector_thrusters[i]->m_servo_speed/m_controller_frequency);
        alpha_l = std::max(m_vector_thrusters[i]->m_servo_angle_min - m_vector_thrusters[i]->get_thruster_servo_angle(), 
                           -m_vector_thrusters[i]->m_servo_speed/m_controller_frequency);

        // printf("thruster [%d]:%lf,%lf\r\n", i, alpha_u, alpha_l);
        double sin_u = std::sin(alpha_u);
        double cos_u = std::cos(alpha_u);
        double sin_l = std::sin(alpha_l);
        double cos_l = std::cos(alpha_l);
        
        if(m_vector_thruster_auto_direction)
        {
            lower_limit[row_count] = 0;
            lower_limit[row_count+1] = N_INFINITY;
            lower_limit[row_count+2] = 0;
            lower_limit[row_count+3] = 0;
            lower_limit[row_count+4] = N_INFINITY;
            lower_limit[row_count+5] = 0;
            // lower_limit[row_count+6] = m_vector_thrusters[i]->m_force_min*cos_u;
            // lower_limit[row_count+7] = m_vector_thrusters[i]->m_force_min*cos_l;
            lower_limit[row_count+6] = m_vector_thrusters[i]->m_force_min;
            lower_limit[row_count+7] = m_vector_thrusters[i]->m_force_min;
            // lower_limit[row_count+8] = N_INFINITY;
            // lower_limit[row_count+9] = 0;

            upper_limit[row_count] = P_INFINITY;
            upper_limit[row_count+1] = 0;
            // upper_limit[row_count+2] = m_vector_thrusters[i]->m_force_max*cos_u;
            // upper_limit[row_count+3] = m_vector_thrusters[i]->m_force_max*cos_l;
            upper_limit[row_count+2] = m_vector_thrusters[i]->m_force_max;
            upper_limit[row_count+3] = m_vector_thrusters[i]->m_force_max;
            upper_limit[row_count+4] = 0;
            upper_limit[row_count+5] = P_INFINITY;
            upper_limit[row_count+6] = 0;
            upper_limit[row_count+7] = 0;
            // upper_limit[row_count+8] = 0;
            // upper_limit[row_count+9] = P_INFINITY;
            
            //tan(alpha_l)*(Fx+ + Fx-) < Fy < tan(alpha_u)*(Fx+ + Fx -)
            constraints_matrix.insert(row_count, col_count) = std::tan(alpha_u);
            constraints_matrix.insert(row_count, col_count +1) = -1;
            constraints_matrix.insert(row_count, col_count +2) = 0;
            
            constraints_matrix.insert(row_count+1, col_count) = std::tan(alpha_l);
            constraints_matrix.insert(row_count+1, col_count +1) = -1;
            constraints_matrix.insert(row_count+1, col_count +2) = 0;

            constraints_matrix.insert(row_count+2, col_count) = 1;
            constraints_matrix.insert(row_count+2, col_count +1) = std::tan(alpha_u/2); //0
            constraints_matrix.insert(row_count+2, col_count +2) = 0;

            constraints_matrix.insert(row_count+3, col_count) = 1;
            constraints_matrix.insert(row_count+3, col_count +1) = std::tan(alpha_l/2); //0;
            constraints_matrix.insert(row_count+3, col_count +2) = 0;

            constraints_matrix.insert(row_count+4, col_count) = 0;
            constraints_matrix.insert(row_count+4, col_count +1) = -1;
            constraints_matrix.insert(row_count+4, col_count +2) = std::tan(alpha_u);
        
            constraints_matrix.insert(row_count+5, col_count) = 0;
            constraints_matrix.insert(row_count+5, col_count +1) = -1;
            constraints_matrix.insert(row_count+5, col_count +2) = std::tan(alpha_l);

            constraints_matrix.insert(row_count+6, col_count) = 0;
            constraints_matrix.insert(row_count+6, col_count +1) = std::tan(alpha_l/2); //0;
            constraints_matrix.insert(row_count+6, col_count +2) = 1;

            constraints_matrix.insert(row_count+7, col_count) = 0;
            constraints_matrix.insert(row_count+7, col_count +1) = std::tan(alpha_u/2); //0;
            constraints_matrix.insert(row_count+7, col_count +2) = 1;

            // constraints_matrix.insert(row_count+8, col_count) = std::tan(alpha_l);
            // constraints_matrix.insert(row_count+8, col_count +1) = -1;
            // constraints_matrix.insert(row_count+8, col_count +2) = std::tan(alpha_l);
            
            // constraints_matrix.insert(row_count+9, col_count) = std::tan(alpha_u);
            // constraints_matrix.insert(row_count+9, col_count +1) = -1;
            // constraints_matrix.insert(row_count+9, col_count +2) = std::tan(alpha_u);


        }
        //manual direction//
        else{
            if(m_vector_thrusters[i]->get_thruster_direction()>0){
                lower_limit[row_count] = 0;
                lower_limit[row_count+1] = N_INFINITY;
                lower_limit[row_count+2] = 0;
                lower_limit[row_count+3] = 0;

                upper_limit[row_count] = P_INFINITY;
                upper_limit[row_count+1] = 0;
                // upper_limit[row_count+2] = m_vector_thrusters[i]->m_force_max*cos_u;
                // upper_limit[row_count+3] = m_vector_thrusters[i]->m_force_max*cos_l;
                upper_limit[row_count+2] = m_vector_thrusters[i]->m_force_max;
                upper_limit[row_count+3] = m_vector_thrusters[i]->m_force_max;


                constraints_matrix.insert(row_count, col_count) = std::tan(alpha_u);
                constraints_matrix.insert(row_count, col_count +1) = -1;

                constraints_matrix.insert(row_count+1, col_count) = std::tan(alpha_l);
                constraints_matrix.insert(row_count+1, col_count +1) = -1;

                constraints_matrix.insert(row_count+2, col_count) = 1;
                constraints_matrix.insert(row_count+2, col_count +1) =std::tan(alpha_u/2); //0;
    
                constraints_matrix.insert(row_count+3, col_count) = 1;
                constraints_matrix.insert(row_count+3, col_count +1) = std::tan(alpha_l/2); //0;

            }
            else{
                // printf("negative thrust direction\r\n");
                lower_limit[row_count] = N_INFINITY;
                lower_limit[row_count+1] = 0;
                // lower_limit[row_count+2] = m_vector_thrusters[i]->m_force_min *cos_u;
                // lower_limit[row_count+3] = m_vector_thrusters[i]->m_force_min *cos_l;
                lower_limit[row_count+2] = m_vector_thrusters[i]->m_force_min;
                lower_limit[row_count+3] = m_vector_thrusters[i]->m_force_min;

                upper_limit[row_count] = 0;
                upper_limit[row_count+1] = P_INFINITY;
                upper_limit[row_count+2] = 0;
                upper_limit[row_count+3] = 0;

                constraints_matrix.insert(row_count, col_count) = std::tan(alpha_u);
                constraints_matrix.insert(row_count, col_count +1) = -1;
    
                constraints_matrix.insert(row_count+1, col_count) = std::tan(alpha_l);
                constraints_matrix.insert(row_count+1, col_count +1) = -1;
    
                constraints_matrix.insert(row_count+2, col_count) =  1;
                constraints_matrix.insert(row_count+2, col_count +1) = std::tan(alpha_l/2);//0;
        
                constraints_matrix.insert(row_count+3, col_count) = 1;
                constraints_matrix.insert(row_count+3, col_count +1) = std::tan(alpha_u/2); //0;
            }
        }

        row_count = row_count + constraints_per_thruster; 
        col_count = col_count + forces_per_thruster;
    }

    m_mvp_control->set_lower_limit(lower_limit);
    m_mvp_control->set_upper_limit(upper_limit);
    m_mvp_control->set_constraint_matrix(constraints_matrix);

}


void MvpControlROS::f_update_restoring_matrix(){

    //update gravity frame 
    Eigen::Vector3d g;
    g.x() = 0;
    g.y() = 0;
    g.z() = m_gravity;
    std::string restore_global_link = m_tf_prefix + "world_ned";
    Eigen::VectorXd m_g_restore;
    m_g_restore = Eigen::VectorXd::Zero(CONTROLLABLE_DOF_LENGTH);
    //convert the force into the world frame
    try {
        //convert force in to the world frame
        geometry_msgs::msg::TransformStamped tf_cg = m_transform_buffer->lookupTransform(
            m_world_link_id,
            restore_global_link,
            tf2::TimePointZero,
            10ms
            );
        
        auto tf_eigen = tf2::transformToEigen(tf_cg);
        Eigen::Vector3d g_xyz = tf_eigen.rotation()*g;
        m_g_restore(DOF::X) = g_xyz.x();
        m_g_restore(DOF::Y) = g_xyz.y();
        m_g_restore(DOF::Z) = g_xyz.z();
        

        //convert force into the local frame
        geometry_msgs::msg::TransformStamped tf_w2c = m_transform_buffer->lookupTransform(
            m_child_link_id,
            restore_global_link,
            tf2::TimePointZero,
            10ms
            );        

        tf_eigen = tf2::transformToEigen(tf_w2c);
        Eigen::Vector3d g_uvw = tf_eigen.rotation()*g;
        m_g_restore(DOF::U) = g_uvw.x();
        m_g_restore(DOF::V) = g_uvw.y();
        m_g_restore(DOF::W) = g_uvw.z();

        //get torque in pqr
        geometry_msgs::msg::TransformStamped tf_local = m_transform_buffer->lookupTransform(
            m_child_link_id,
            m_gravity_link,
            tf2::TimePointZero,
            10ms
            );

        tf_eigen = tf2::transformToEigen(tf_local);

        auto trans_xyz = tf_eigen.translation();
        auto t_pqr = trans_xyz.cross(g_uvw);
        m_g_restore(DOF::P) = t_pqr.x();
        m_g_restore(DOF::Q) = t_pqr.y();
        m_g_restore(DOF::R) = t_pqr.z();

        //get torque in roll pitch yaw
        geometry_msgs::msg::TransformStamped tf_child_world = m_transform_buffer->lookupTransform(
            m_world_link_id,
            m_child_link_id,
            tf2::TimePointZero,
            10ms
        );

        Eigen::Matrix3d ang_vel_tranform = f_angular_velocity_transform(tf_child_world);

        auto t_rpy = ang_vel_tranform * t_pqr;
        m_g_restore(DOF::ROLL) = t_rpy.x();
        m_g_restore(DOF::PITCH) = t_rpy.y();
        m_g_restore(DOF::YAW) = t_rpy.z();

    } catch (const tf2::TransformException & e) {
        auto steady_clock = rclcpp::Clock();

        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), steady_clock, 1000, std::string("Can't find TF for gravity: ") + e.what());
    }


    //update buoyancy frame 
    Eigen::Vector3d b;
    b.x() = 0;
    b.y() = 0;
    b.z() = m_buoyancy;

    Eigen::VectorXd m_b_restore;
    m_b_restore = Eigen::VectorXd::Zero(CONTROLLABLE_DOF_LENGTH);
    //convert the force into the world frame
    try {
        //convert force in to the world frame
        geometry_msgs::msg::TransformStamped tf_cb = m_transform_buffer->lookupTransform(
            m_world_link_id,
            restore_global_link,
            tf2::TimePointZero,
            10ms
            );
        
        auto tf_eigen = tf2::transformToEigen(tf_cb);
        Eigen::Vector3d b_xyz = tf_eigen.rotation()*b;
        m_b_restore(DOF::X) = b_xyz.x();
        m_b_restore(DOF::Y) = b_xyz.y();
        m_b_restore(DOF::Z) = b_xyz.z();
        

        //convert force into the local frame
        geometry_msgs::msg::TransformStamped tf_w2c = m_transform_buffer->lookupTransform(
            m_child_link_id,
            restore_global_link,
            tf2::TimePointZero,
            10ms
            );

        tf_eigen = tf2::transformToEigen(tf_w2c);
        Eigen::Vector3d b_uvw = tf_eigen.rotation()*b;
        m_b_restore(DOF::U) = b_uvw.x();
        m_b_restore(DOF::V) = b_uvw.y();
        m_b_restore(DOF::W) = b_uvw.z();

        //get torque in pqr
        geometry_msgs::msg::TransformStamped tf_local = m_transform_buffer->lookupTransform(
            m_child_link_id,
            m_buoyancy_link,
            tf2::TimePointZero,
            10ms
            );

        tf_eigen = tf2::transformToEigen(tf_local);

        auto trans_xyz = tf_eigen.translation();
        auto t_pqr = trans_xyz.cross(b_uvw);
        m_b_restore(DOF::P) = t_pqr.x();
        m_b_restore(DOF::Q) = t_pqr.y();
        m_b_restore(DOF::R) = t_pqr.z();

        //get torque in roll pitch yaw
        geometry_msgs::msg::TransformStamped tf_child_world = m_transform_buffer->lookupTransform(
            m_world_link_id,
            m_child_link_id,
            tf2::TimePointZero,
            10ms
        );
        Eigen::Matrix3d ang_vel_tranform = f_angular_velocity_transform(tf_child_world);

        auto t_rpy = ang_vel_tranform * t_pqr;
        m_b_restore(DOF::ROLL) = t_rpy.x();
        m_b_restore(DOF::PITCH) = t_rpy.y();
        m_b_restore(DOF::YAW) = t_rpy.z();

    } catch (const tf2::TransformException & e) {
        auto steady_clock = rclcpp::Clock();

        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), steady_clock, 1000, std::string("Can't find TF for buoyancy: ") + e.what());
    }

    m_mvp_control->set_restoring_force_matrix(m_b_restore+m_g_restore);
}

bool MvpControlROS::f_compute_process_values() {
    auto steady_clock = rclcpp::Clock();
    rclcpp::Time now = this->get_clock()->now();

    f_update_control_allocation_matrix();
    
    if(m_restoring_effort_flag){
        f_update_restoring_matrix();
    }

    try {
        //xyz and roll pitch yaw can be obtained directly from the TF
        // Transform child frame to world
        geometry_msgs::msg::TransformStamped odom_world = m_transform_buffer->lookupTransform(
            m_world_link_id,
            m_child_link_id,
            tf2::TimePointZero,
            10ms
        );
        //get x,y,z in world link
        m_process_values(DOF::X) = odom_world.transform.translation.x;
        m_process_values(DOF::Y) = odom_world.transform.translation.y;
        m_process_values(DOF::Z) = odom_world.transform.translation.z;

        tf2::Quaternion quat;
        tf2::fromMsg(odom_world.transform.rotation, quat);
        
        //get current state roll pitch yaw
        // Convert to Euler angles (roll, pitch, yaw)
        tf2::Matrix3x3(quat).getRPY(
            m_process_values(DOF::ROLL),
            m_process_values(DOF::PITCH),
            m_process_values(DOF::YAW)
        );

    } catch(tf2::TransformException &e) {
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), steady_clock, 1000, std::string("Can't compute process values: ") + e.what());
        return false;
    }

        // Transform from odom to world
    try{
        //transform child frame velocity terms from odometry to our controlled child link
        std::scoped_lock lock(m_odom_lock);

        geometry_msgs::msg::TransformStamped cg_odom = m_transform_buffer->lookupTransform(
                m_child_link_id,
                m_odometry_msg.child_frame_id,
                tf2::TimePointZero,
                10ms
        );

        auto cg_odom_eigen = tf2::transformToEigen(cg_odom);

        // convert angular velocity from odom_child_link to cg_link
        Eigen::Matrix3d ang_vel_transform = f_angular_velocity_transform(cg_odom);

        // convert linear velocity from odomtery child frame to child frame
        Eigen::Vector3d uvw;
        uvw.x() = m_odometry_msg.twist.twist.linear.x;
        uvw.y() = m_odometry_msg.twist.twist.linear.y;
        uvw.z() = m_odometry_msg.twist.twist.linear.z;

        uvw = cg_odom_eigen.rotation()  * uvw;

        m_process_values(DOF::U) = uvw.x();
        m_process_values(DOF::V) = uvw.y();
        m_process_values(DOF::W) = uvw.z();

        Eigen::Vector3d angular_rate;
        angular_rate.x() = m_odometry_msg.twist.twist.angular.x;
        angular_rate.y() = m_odometry_msg.twist.twist.angular.y;
        angular_rate.z() = m_odometry_msg.twist.twist.angular.z;

        angular_rate = ang_vel_transform * angular_rate;

        m_process_values(DOF::P) = angular_rate.x();
        m_process_values(DOF::Q) = angular_rate.y();
        m_process_values(DOF::R) = angular_rate.z();

    } catch(tf2::TransformException &e) {
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), steady_clock, 1000, std::string("Can't compute process values!, check odometry!: ") + e.what());
        return false;
    }

    mvp_msgs::msg::ControlProcess s;
    s.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now(),
    s.header.frame_id = m_world_link_id;
    s.control_mode = m_control_mode;
    s.child_frame_id = m_child_link_id;
    s.position.x = m_process_values(DOF::X);
    s.position.y = m_process_values(DOF::Y);
    s.position.z = m_process_values(DOF::Z);
    s.orientation.x = m_process_values(DOF::ROLL);
    s.orientation.y = m_process_values(DOF::PITCH);
    s.orientation.z = m_process_values(DOF::YAW);
    s.velocity.x = m_process_values(DOF::U);
    s.velocity.y = m_process_values(DOF::V);
    s.velocity.z = m_process_values(DOF::W);
    // body frame angular velocity pqr
    s.angular_rate.x = m_process_values(DOF::P);
    s.angular_rate.y = m_process_values(DOF::Q);
    s.angular_rate.z = m_process_values(DOF::R);

    m_mvp_control->set_system_state(m_process_values);

    m_process_value_publisher->publish(s);

    mvp_msgs::msg::ControlProcess e;

    Eigen::VectorXd error_state = m_mvp_control->get_state_error();
    e.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now(),
    e.header.frame_id = m_world_link_id;
    e.control_mode = m_control_mode;
    e.child_frame_id = m_child_link_id;
    e.position.x = error_state(DOF::X);
    e.position.y = error_state(DOF::Y);
    e.position.z = error_state(DOF::Z);
    e.orientation.x = error_state(DOF::ROLL);
    e.orientation.y = error_state(DOF::PITCH);
    e.orientation.z = error_state(DOF::YAW);
    e.velocity.x = error_state(DOF::U);
    e.velocity.y = error_state(DOF::V);
    e.velocity.z = error_state(DOF::W);
    e.angular_rate.x = error_state(DOF::P);
    e.angular_rate.y = error_state(DOF::Q);
    e.angular_rate.z = error_state(DOF::R);

    m_process_error_publisher->publish(e);

    return true;
}


void MvpControlROS::f_control_loop() {

    double pt = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
    // setpoint_timer = rclcpp::Clock(RCL_ROS_TIME).now().nanoseconds() / 1000000000.0;

    auto r = rclcpp::Rate(m_controller_frequency);

    while(rclcpp::ok()) {
        /**
         * Thread may not be able to sleep properly. This may happen using
         * simulated time.
         */
        /**
         * Record the time that loop ends. Later, it will feed the PID
         * controller.
         */
        if(!r.sleep()) {
            continue;
        }
        double dt = rclcpp::Clock(RCL_ROS_TIME).now().seconds() - pt;
        pt = rclcpp::Clock(RCL_ROS_TIME).now().seconds();


        /**
         * Compute the state of the system. Continue on failure. This may
         * happen when transform tree is not ready.
         */
        if(not f_compute_process_values()) {
            continue;
        }

        /**
         * Check if controller is enabled or not.
         */
        
        double time_since_last_setpoint = rclcpp::Clock(RCL_ROS_TIME).now().seconds() - setpoint_timer;
        
        if(!m_enabled || time_since_last_setpoint > m_no_setpoint_timeout) {
             for(uint64_t i = 0 ; i < m_thrusters.size() ; i++) {
                // m_thrusters.at(i)->command(0);
                std_msgs::msg::Float64 msg;
                msg.data = 0.0;
                m_thrusters.at(i)->m_current_force = 0.0;
                m_thrusters.at(i)->m_thrust_publisher->publish(msg);
            }

            for(uint64_t i = 0 ; i < m_vector_thrusters.size() ; i++) {
                std_msgs::msg::Float64 msg;
                msg.data = 0.0;
                m_vector_thrusters.at(i)->m_current_force = 0.0;

                m_vector_thrusters.at(i)->m_thrust_publisher->publish(msg);
                // m_vector_thrusters.at(i)->m_angle_publisher->publish(msg);
            }
            continue;
        }

        

        Eigen::VectorXd needed_forces;

        /**
         * Get time difference to feed PID controller
         */
        // double dt = rclcpp::Clock(RCL_ROS_TIME).now().seconds() - pt;
        
        /**
         * Calculate forces to be requested from thrusters. If operation fails,
         * do not send commands to thrusters.
         */
        if(m_mvp_control->calculate_needed_forces(&needed_forces, dt)) {
            
            //regular thrusters
            
            for(uint64_t i = 0 ; i < m_thrusters.size() ; i++) {
                double command;
                std_msgs::msg::Float64 Nmsg;
                Nmsg.data = needed_forces(i);
                // printf("###force for thruster %d: %f\r\n", i, needed_forces(i));
                m_thrusters.at(i)->m_current_force = Nmsg.data;

                m_thrusters.at(i)->m_force_publisher->publish(Nmsg);
                
                if (m_thrusters.at(i)->request_command(needed_forces(i), command)){
                        std_msgs::msg::Float64 msg;
                        msg.data = command;
                        m_thrusters.at(i)->m_thrust_publisher->publish(msg);
                }
                
            }

            int count = m_thrusters.size();
            //vector thrusters
            for(uint64_t i = 0 ; i < m_vector_thrusters.size() ; i++) {
                double fxp, fxn, fy, angle, command, new_angle;
                double fs;
                double fx;
                std_msgs::msg::Float64 Nmsg;
                fxn = 0;

                fxp = needed_forces(count);
                count++;
                fy = needed_forces(count);
                count++;
                // printf("auto_direction =%d",(int)m_vector_thruster_auto_direction);

                if(m_vector_thruster_auto_direction)
                {
                    fxn = needed_forces(count);
                    count++;
                    // fs = needed_forces(count);
                    // count++;
                }
                // printf("thruster %d results: %lf, %lf, %lf, \r\n", fxp, fxn, fy);

                fx = fxp+fxn;
                angle = m_vector_thrusters[i]->get_thruster_servo_angle();

                Nmsg.data = std::copysign(1.0, fx) * std::sqrt(std::pow(fx, 2) + std::pow(fy, 2));

                m_vector_thrusters.at(i)->m_current_force = Nmsg.data;

                m_vector_thrusters.at(i)->m_force_publisher->publish(Nmsg);

                if (m_vector_thrusters.at(i)->request_command(fx, fy, angle, command, new_angle) )
                {
                        std_msgs::msg::Float64 msg, ang_msg;
                        msg.data = command;
                        
                        // new_angle = std::max(m_vector_thrusters[i]->m_servo_angle_min, std::min(m_vector_thrusters[i]->m_servo_angle_max, new_angle));
                        ang_msg.data = new_angle;
                        m_vector_thrusters.at(i)->m_thrust_publisher->publish(msg);
                        m_vector_thrusters.at(i)->m_angle_publisher->publish(ang_msg);
                }


            }

        }
        else{
            //stop 
            for(uint64_t i = 0 ; i < m_thrusters.size() ; i++) {
                std_msgs::msg::Float64 msg;
                msg.data = 0;
                m_thrusters.at(i)->m_thrust_publisher->publish(msg);
                }
            for(uint64_t i = 0 ; i < m_vector_thrusters.size() ; i++) {
                std_msgs::msg::Float64 msg;
                msg.data = 0;
                m_vector_thrusters.at(i)->m_thrust_publisher->publish(msg);
            }
                
        }


        //grab PIDV vvalues and publish
        Eigen::VectorXd m_d(CONTROLLABLE_DOF_LENGTH);
        Eigen::VectorXd m_v(CONTROLLABLE_DOF_LENGTH);
        Eigen::VectorXd m_i(CONTROLLABLE_DOF_LENGTH);
        Eigen::VectorXd m_p(CONTROLLABLE_DOF_LENGTH);


        m_d = m_mvp_control->get_pid()->get_m_d();
        m_v = m_mvp_control->get_pid()->get_m_v();
        m_i = m_mvp_control->get_pid()->get_m_i();
        m_p = m_mvp_control->get_pid()->get_m_p();


        std_msgs::msg::Float64MultiArray msg;

        //propotional
        msg.data.resize(m_p.size());
        for (int i = 0; i < m_p.size(); ++i)
        {
            msg.data[i] = m_p(i);
        }
        m_p_publisher->publish(msg);


        //derivative
        msg.data.resize(m_d.size());
        for (int i = 0; i < m_d.size(); ++i)
        {
            msg.data[i] = m_d(i);
        }
        m_d_publisher->publish(msg);

        //velocity
        msg.data.resize(m_v.size());
        for (int i = 0; i < m_v.size(); ++i)
        {
            msg.data[i] = m_v(i);
        }
        m_v_publisher->publish(msg);

        //integral
        msg.data.resize(m_i.size());
        for (int i = 0; i < m_i.size(); ++i)
        {
            msg.data[i] = m_i(i);
        }
        m_i_publisher->publish(msg);


        // /**
        //  * Record the time that loop ends. Later, it will feed the PID
        //  * controller.
        //  */
        // pt = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
    }
}

void MvpControlROS::f_cb_msg_odometry(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::scoped_lock lock(m_odom_lock);
    m_odometry_msg = *msg;

}

void MvpControlROS::f_cb_srv_set_point(
    const mvp_msgs::msg::ControlProcess::SharedPtr msg) {

    // printf("got setpoint msgs\r\n");
    setpoint_timer = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
    f_amend_set_point(msg);
}

void MvpControlROS::f_cb_servo_joint(
            const sensor_msgs::msg::JointState::SharedPtr msg)
{
    //Set vector thruster servo angle from msg
    for(unsigned int i = 0 ; i < m_vector_thrusters.size() ; i++ ) {
        // printf("joint id=%s\r\n", m_vector_thrusters[i]->get_thruster_servo_joint_id().c_str());
        auto it = std::find(msg->name.begin(), msg->name.end(), m_vector_thrusters[i]->get_thruster_servo_joint_id());
        if (it != msg->name.end()) {
            int ind = std::distance(msg->name.begin(), it);
            m_vector_thrusters[i]->set_thruster_servo_angle(msg->position[ind]);
        }
        else{
            printf("joint not found\r\n");
        }
    }
}

void MvpControlROS::f_cb_vector_thruster_direction(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
{
    if(msg->data.size()== m_vector_thrusters.size()){
        for(unsigned int i =0; i < m_vector_thrusters.size(); i++)
        {
            double result = (msg->data[i] >= 0) ? 1.0 : -1.0; //make it -1 or 1
            m_vector_thrusters[i]->set_thruster_direction(result);
            printf("%s direction set to %d\r\n", m_vector_thrusters[i]->get_link_id().c_str(), msg->data[i]);
        }
    }
    else{
        printf("direction array size incorrect\r\n");
    }
}


void MvpControlROS::f_load_control_config()
{
    YAML::Node map = YAML::LoadFile(m_control_config_file);
    std::vector<std::string> modes;

    //load control modes
    if(map["control_modes"])
    {
        //parse control modes
        // printf("size = %d\r\n", map["control_modes"].size() );
        //iterate through control modes
        for(YAML::const_iterator it=map["control_modes"].begin();it != map["control_modes"].end(); ++it) 
        {
            std::string key = it->first.as<std::string>();       // <- key
            modes.push_back(key);
            
        }
        
        std::map<std::string, std::set<int>> mode_rules;

        //loop through modes
        for (const auto &mode: modes) 
        {
            mvp_msgs::msg::ControlMode m;

            m.name = mode;

            // printf("mode_name = %s\r\n", mode.c_str() );
            //loop through DOF to create ros params
            for(YAML::const_iterator it=map["control_modes"][mode].begin();it != map["control_modes"][mode].end(); ++it) 
            {
                std::string param_name;
                std::string dof_name = it->first.as<std::string>();
                // printf("    dof_name = %s\r\n", dof_name.c_str());
                //get PID values
                for(const auto& key : {"p", "i", "d", "v", "pid_max", "pid_min"})
                {
                    param_name = "control_modes/" + mode + "/" + dof_name + "/" + key;
                    // printf("         param = %s\r\n", param_name.c_str());
                    this->declare_parameter( param_name, map["control_modes"][mode][dof_name][key].as<float>() );
                }

                if(dof_name.compare(CONF_DOF_X) == 0){
                    param_name = "control_modes/" + mode + "/" + dof_name + "/";
                    this->get_parameter(param_name + CONF_PID_P, m.pid_x.kp);
                    this->get_parameter(param_name + CONF_PID_I, m.pid_x.ki);
                    this->get_parameter(param_name + CONF_PID_D, m.pid_x.kd);
                    this->get_parameter(param_name + CONF_PID_V, m.pid_x.kv);
                    this->get_parameter(param_name + CONF_PID_MAX, m.pid_x.pid_max);
                    this->get_parameter(param_name + CONF_PID_MIN, m.pid_x.pid_min);
                }
                if(dof_name.compare(CONF_DOF_Y) == 0){
                    param_name = "control_modes/" + mode + "/" + dof_name + "/";
                    this->get_parameter(param_name + CONF_PID_P, m.pid_y.kp);
                    this->get_parameter(param_name + CONF_PID_I, m.pid_y.ki);
                    this->get_parameter(param_name + CONF_PID_D, m.pid_y.kd);
                    this->get_parameter(param_name + CONF_PID_V, m.pid_y.kv);
                    this->get_parameter(param_name + CONF_PID_MAX, m.pid_y.pid_max);
                    this->get_parameter(param_name + CONF_PID_MIN, m.pid_y.pid_min);
                }
                if(dof_name.compare(CONF_DOF_Z) == 0){
                    param_name = "control_modes/" + mode + "/" + dof_name + "/";
                    this->get_parameter(param_name + CONF_PID_P, m.pid_z.kp);
                    this->get_parameter(param_name + CONF_PID_I, m.pid_z.ki);
                    this->get_parameter(param_name + CONF_PID_D, m.pid_z.kd);
                    this->get_parameter(param_name + CONF_PID_V, m.pid_z.kv);
                    this->get_parameter(param_name + CONF_PID_MAX, m.pid_z.pid_max);
                    this->get_parameter(param_name + CONF_PID_MIN, m.pid_z.pid_min);
                }
                if(dof_name.compare(CONF_DOF_ROLL) == 0){
                    param_name = "control_modes/" + mode + "/" + dof_name + "/";
                    this->get_parameter(param_name + CONF_PID_P, m.pid_roll.kp);
                    this->get_parameter(param_name + CONF_PID_I, m.pid_roll.ki);
                    this->get_parameter(param_name + CONF_PID_D, m.pid_roll.kd);
                    this->get_parameter(param_name + CONF_PID_V, m.pid_roll.kv);
                    this->get_parameter(param_name + CONF_PID_MAX, m.pid_roll.pid_max);
                    this->get_parameter(param_name + CONF_PID_MIN, m.pid_roll.pid_min);
                }
                if(dof_name.compare(CONF_DOF_PITCH) == 0){
                    param_name = "control_modes/" + mode + "/" + dof_name + "/";
                    this->get_parameter(param_name + CONF_PID_P, m.pid_pitch.kp);
                    this->get_parameter(param_name + CONF_PID_I, m.pid_pitch.ki);
                    this->get_parameter(param_name + CONF_PID_D, m.pid_pitch.kd);
                    this->get_parameter(param_name + CONF_PID_V, m.pid_pitch.kv);
                    this->get_parameter(param_name + CONF_PID_MAX, m.pid_pitch.pid_max);
                    this->get_parameter(param_name + CONF_PID_MIN, m.pid_pitch.pid_min);
                }
                if(dof_name.compare(CONF_DOF_YAW) == 0){
                    param_name = "control_modes/" + mode + "/" + dof_name + "/";
                    this->get_parameter(param_name + CONF_PID_P, m.pid_yaw.kp);
                    this->get_parameter(param_name + CONF_PID_I, m.pid_yaw.ki);
                    this->get_parameter(param_name + CONF_PID_D, m.pid_yaw.kd);
                    this->get_parameter(param_name + CONF_PID_V, m.pid_yaw.kv);
                    this->get_parameter(param_name + CONF_PID_MAX, m.pid_yaw.pid_max);
                    this->get_parameter(param_name + CONF_PID_MIN, m.pid_yaw.pid_min);
                }
                if(dof_name.compare(CONF_DOF_U) == 0){
                    param_name = "control_modes/" + mode + "/" + dof_name + "/";
                    this->get_parameter(param_name + CONF_PID_P, m.pid_u.kp);
                    this->get_parameter(param_name + CONF_PID_I, m.pid_u.ki);
                    this->get_parameter(param_name + CONF_PID_D, m.pid_u.kd);
                    this->get_parameter(param_name + CONF_PID_V, m.pid_u.kv);
                    this->get_parameter(param_name + CONF_PID_MAX, m.pid_u.pid_max);
                    this->get_parameter(param_name + CONF_PID_MIN, m.pid_u.pid_min);
                }
                if(dof_name.compare(CONF_DOF_V) == 0){
                    param_name = "control_modes/" + mode + "/" + dof_name + "/";
                    this->get_parameter(param_name + CONF_PID_P, m.pid_v.kp);
                    this->get_parameter(param_name + CONF_PID_I, m.pid_v.ki);
                    this->get_parameter(param_name + CONF_PID_D, m.pid_v.kd);
                    this->get_parameter(param_name + CONF_PID_V, m.pid_v.kv);
                    this->get_parameter(param_name + CONF_PID_MAX, m.pid_v.pid_max);
                    this->get_parameter(param_name + CONF_PID_MIN, m.pid_v.pid_min);
                }

                if(dof_name.compare(CONF_DOF_W) == 0){
                    param_name = "control_modes/" + mode + "/" + dof_name + "/";
                    this->get_parameter(param_name + CONF_PID_P, m.pid_w.kp);
                    this->get_parameter(param_name + CONF_PID_I, m.pid_w.ki);
                    this->get_parameter(param_name + CONF_PID_D, m.pid_w.kd);
                    this->get_parameter(param_name + CONF_PID_V, m.pid_w.kv);
                    this->get_parameter(param_name + CONF_PID_MAX, m.pid_w.pid_max);
                    this->get_parameter(param_name + CONF_PID_MIN, m.pid_w.pid_min);
                }

                if(dof_name.compare(CONF_DOF_P) == 0){
                    param_name = "control_modes/" + mode + "/" + dof_name + "/";
                    this->get_parameter(param_name + CONF_PID_P, m.pid_p.kp);
                    this->get_parameter(param_name + CONF_PID_I, m.pid_p.ki);
                    this->get_parameter(param_name + CONF_PID_D, m.pid_p.kd);
                    this->get_parameter(param_name + CONF_PID_V, m.pid_p.kv);
                    this->get_parameter(param_name + CONF_PID_MAX, m.pid_p.pid_max);
                    this->get_parameter(param_name + CONF_PID_MIN, m.pid_p.pid_min);
                }
                if(dof_name.compare(CONF_DOF_Q) == 0){
                    param_name = "control_modes/" + mode + "/" + dof_name + "/";
                    this->get_parameter(param_name + CONF_PID_P, m.pid_q.kp);
                    this->get_parameter(param_name + CONF_PID_I, m.pid_q.ki);
                    this->get_parameter(param_name + CONF_PID_D, m.pid_q.kd);
                    this->get_parameter(param_name + CONF_PID_V, m.pid_q.kv);
                    this->get_parameter(param_name + CONF_PID_MAX, m.pid_q.pid_max);
                    this->get_parameter(param_name + CONF_PID_MIN, m.pid_q.pid_min);
                }
                if(dof_name.compare(CONF_DOF_R) == 0){
                    param_name = "control_modes/" + mode + "/" + dof_name + "/";
                    this->get_parameter(param_name + CONF_PID_P, m.pid_r.kp);
                    this->get_parameter(param_name + CONF_PID_I, m.pid_r.ki);
                    this->get_parameter(param_name + CONF_PID_D, m.pid_r.kd);
                    this->get_parameter(param_name + CONF_PID_V, m.pid_r.kv);
                    this->get_parameter(param_name + CONF_PID_MAX, m.pid_r.pid_max);
                    this->get_parameter(param_name + CONF_PID_MIN, m.pid_r.pid_min);
                }
                //check dof enabled
                auto found =std::find_if(CONF_DOF_LOOKUP.begin(), CONF_DOF_LOOKUP.end(),
                    [dof_name](const std::pair<const char *, int> &t) -> bool {
                        return std::strcmp(dof_name.c_str(),t.first) == 0;
                    }
                );

                if (found != CONF_DOF_LOOKUP.end()) {
                    mode_rules[mode].insert(found->second);
                } else {
                    throw control_ros_exception(
                            "Unknown freedom name passed '" + dof_name + "'"
                            "Possible values are "
                        "'x, y, z, roll, pitch, yaw, surge, sway, heave"
                    );
                }

            }
            m.dofs = std::vector<int>(mode_rules[mode].begin(), mode_rules[mode].end());
            m_control_modes.modes.emplace_back(m);
        }

    }

    //load thruster params
    if(map["thruster_ids"])
    {
        std::vector<std::string> thruster_id_list;
        // printf("#######################################################\r\n");
        //load the thruster name
        for(YAML::const_iterator it=map["thruster_ids"].begin();it != map["thruster_ids"].end(); ++it) 
        {

            std::string t_name = it->first.as<std::string>();       // thruster_name
            // printf("###Thruster id =%s\r\n", t_name.c_str());
            ThrusterROS::Ptr t(new ThrusterROS());
            t->set_id(t_name);
            

            std::string param_name; 

            //get thruster parameters:
            std::string link_id;
            param_name = map["thruster_ids"][t_name]["control_tf"].as<std::string>();
            this->declare_parameter(std::string()+CONF_CONTROL_TF + "/" + t_name + "_thruster_link", param_name);
            this->get_parameter(std::string()+CONF_CONTROL_TF + "/" + t_name + "_thruster_link", link_id);
            t->set_link_id(m_tf_prefix + link_id);

            param_name = map["thruster_ids"][t_name]["command_topic"].as<std::string>();
            // printf("    command_topic: %s\r\n", topic_name.c_str());
            this->declare_parameter(std::string()+CONF_THRUST_COMMAND_TOPICS + "/" + t_name, param_name);
            t->set_thrust_command_topic_id(param_name);
            t->m_thrust_publisher = this->create_publisher<std_msgs::msg::Float64>(param_name, 10);
            printf("####Thruster: %s, topic name: %s\r\n", t_name.c_str(), param_name.c_str());
            
            param_name = map["thruster_ids"][t_name]["force_topic"].as<std::string>();
            // printf("    command_topic: %s\r\n", topic_name.c_str());
            this->declare_parameter(std::string()+CONF_THRUSTER_FORCE_TOPICS + "/" + t_name, param_name);
            t->set_thrust_force_topic_id(param_name);
            t->m_force_publisher= this->create_publisher<std_msgs::msg::Float64>(param_name, 10);
            printf("####Thruster: %s, force_topic name: %s\r\n", t_name.c_str(), param_name.c_str());


            std::vector<float> min_max;
            min_max = map["thruster_ids"][t_name]["limits"].as<std::vector<float> >();
            // printf("    MAX: %f to %f\r\n", min_max[0], min_max[1]);
            this->declare_parameter(std::string()+CONF_THRUSTER_LIMITS + "/" + t_name + "/" + CONF_THRUSTER_MIN, min_max[0]);
            this->get_parameter(std::string()+CONF_THRUSTER_LIMITS + "/" + t_name + "/" + CONF_THRUSTER_MIN, t->m_force_min);

            this->declare_parameter(std::string()+CONF_THRUSTER_LIMITS + "/" + t_name + "/" + CONF_THRUSTER_MAX, min_max[1]);
            this->get_parameter(std::string()+CONF_THRUSTER_LIMITS + "/" + t_name + "/" + CONF_THRUSTER_MAX, t->m_force_max);

            double delta_limit;
            delta_limit = map["thruster_ids"][t_name]["delta_limit"].as<double>();
            // delta_limit = map["thruster_ids"][t_name]["delta_limit"].as<double>(200.0);

            this->declare_parameter(std::string()+CONF_THRUSTER_LIMITS + "/" + t_name + "/" + CONF_THRUSTER_D_LIMIT, delta_limit);
            this->get_parameter(std::string()+CONF_THRUSTER_LIMITS + "/" + t_name + "/" + CONF_THRUSTER_D_LIMIT, t->m_force_delta_limit);

            std::vector<double> poly_coef;
            poly_coef = map["thruster_ids"][t_name]["polynomials"].as<std::vector<double> >();
            // std::cout<<poly_coef<<std::endl;
            this->declare_parameter(std::string()+CONF_THRUSTER_POLY + "/" + t_name, poly_coef);
            t->get_poly_solver()->set_coeff(poly_coef);


            std::vector<float> xyz_flag;
            // xyz_flag = map["thruster_ids"][t_name]["xyz_flag"].as<std::vector<float> >();
            if (map["thruster_ids"][t_name]["xyz_flag"]) {
                xyz_flag = map["thruster_ids"][t_name]["xyz_flag"].as<std::vector<float>>();
            } else {
                xyz_flag = {1.0f, 1.0f, 1.0f};  // default value
            }
            this->declare_parameter(std::string()+CONF_THRUSTER_XYZ_FLAG + "/" + t_name, xyz_flag);
            Eigen::Vector3d xyz_flag_vector(xyz_flag[0], xyz_flag[1], xyz_flag[2]);
            t->m_xyz_flag_vector = xyz_flag_vector;

            m_thrusters.emplace_back(t);
        }


    }

    //load vector thruster
    if(map["vector_thruster_ids"])
    {
        std::vector<std::string> vector_thruster_id_list;
        // printf("#######################################################\r\n");
        //load the thruster name
        for(YAML::const_iterator it=map["vector_thruster_ids"].begin();it != map["vector_thruster_ids"].end(); ++it) 
        {

            std::string t_name = it->first.as<std::string>();       // thruster_name
            // printf("###Thruster id =%s\r\n", t_name.c_str());
            VectorThrusterROS::Ptr t(new VectorThrusterROS());
            t->set_id(t_name);
        
            std::string param_name; 

            //get thruster parameters:
            std::string link_id;
            param_name = map["vector_thruster_ids"][t_name]["control_tf"].as<std::string>();
            this->declare_parameter(std::string()+CONF_CONTROL_TF + "/" + t_name + "vector_thruster_link", param_name);
            this->get_parameter(std::string()+CONF_CONTROL_TF + "/" + t_name + "vector_thruster_link", link_id);
            t->set_link_id(m_tf_prefix + link_id);

            param_name = map["vector_thruster_ids"][t_name]["command_topic"].as<std::string>();
            // printf("    command_topic: %s\r\n", topic_name.c_str());set_thruster_direction
            this->declare_parameter(std::string()+CONF_THRUST_COMMAND_TOPICS + "/" + t_name, param_name);
            t->set_thrust_command_topic_id(param_name);
            t->m_thrust_publisher = this->create_publisher<std_msgs::msg::Float64>(param_name, 10);
            printf("####Vector Thruster: %s, topic name: %s\r\n", t_name.c_str(), param_name.c_str());
            
            param_name = map["vector_thruster_ids"][t_name]["force_topic"].as<std::string>();
            // printf("    command_topic: %s\r\n", topic_name.c_str());
            this->declare_parameter(std::string()+CONF_THRUSTER_FORCE_TOPICS + "/" + t_name, param_name);
            t->set_thrust_force_topic_id(param_name);
            t->m_force_publisher= this->create_publisher<std_msgs::msg::Float64>(param_name, 10);
            printf("####Vector Thruster: %s, force_topic name: %s\r\n", t_name.c_str(), param_name.c_str());

            std::vector<float> min_max;
            min_max = map["vector_thruster_ids"][t_name]["limits"].as<std::vector<float> >();
            // printf("    MAX: %f to %f\r\n", min_max[0], min_max[1]);
            this->declare_parameter(std::string()+CONF_THRUSTER_LIMITS + "/" + t_name + "/" + CONF_THRUSTER_MIN, min_max[0]);
            this->get_parameter(std::string()+CONF_THRUSTER_LIMITS + "/" + t_name + "/" + CONF_THRUSTER_MIN, t->m_force_min);

            this->declare_parameter(std::string()+CONF_THRUSTER_LIMITS + "/" + t_name + "/" + CONF_THRUSTER_MAX, min_max[1]);
            this->get_parameter(std::string()+CONF_THRUSTER_LIMITS + "/" + t_name + "/" + CONF_THRUSTER_MAX, t->m_force_max);

            double delta_limit;
            delta_limit = map["vector_thruster_ids"][t_name]["delta_limit"].as<double>();
            this->declare_parameter(std::string()+CONF_THRUSTER_LIMITS + "/" + t_name + "/" + CONF_THRUSTER_D_LIMIT, delta_limit);
            this->get_parameter(std::string()+CONF_THRUSTER_LIMITS + "/" + t_name + "/" + CONF_THRUSTER_D_LIMIT, t->m_force_delta_limit);

            std::vector<double> poly_coef;
            poly_coef = map["vector_thruster_ids"][t_name]["polynomials"].as<std::vector<double> >();
            // std::cout<<poly_coef<<std::endl;
            this->declare_parameter(std::string()+CONF_THRUSTER_POLY + "/" + t_name, poly_coef);
            t->get_poly_solver()->set_coeff(poly_coef);

            ///servo stuff
            param_name = map["vector_thruster_ids"][t_name]["servo_topic"].as<std::string>();
            // printf("    command_topic: %s\r\n", topic_name.c_str());
            this->declare_parameter(std::string()+CONF_THRUSTER_SERVO_TOPIC + "/" + t_name, param_name);
            t->set_thruster_servo_topic_id(param_name);
            t->m_angle_publisher= this->create_publisher<std_msgs::msg::Float64>(param_name, 10);
            printf("####Vector Thruster: %s, servo_topic name: %s\r\n", t_name.c_str(), param_name.c_str());

            //joint need namespace prefix
            param_name = map["vector_thruster_ids"][t_name]["servo_joint"].as<std::string>();

            std::string m_ns = this->get_namespace();
            if (!m_ns.empty() && m_ns[0] == '/') {
                m_ns = m_ns.substr(1);
            }
            std::string joint_name = m_ns + "/" + param_name;
            // printf("    command_topic: %s\r\n", topic_name.c_str());
            this->declare_parameter(std::string()+CONF_THRUSTER_SERVO_JOINT + "/" + t_name, joint_name);
            t->set_thruster_servo_joint_id(joint_name);
            printf("####Vector Thruster: %s, servo_joint name: %s\r\n", t_name.c_str(), joint_name.c_str());

            double speed = map["vector_thruster_ids"][t_name]["servo_speed"].as<float>();
            // printf("    command_topic: %s\r\n", topic_name.c_str());
            this->declare_parameter(std::string()+CONF_THRUSTER_SERVO_SPEED + "/" + t_name, speed);
            t->set_thruster_servo_speed(speed);
            t->m_servo_angle_step = t->get_thruster_servo_speed()/m_controller_frequency;  //get angle step.


            std::vector<float> angle_min_max;
            angle_min_max = map["vector_thruster_ids"][t_name]["angle_limits"].as<std::vector<float> >();
            this->declare_parameter(std::string()+CONF_THRUSTER_SERVO_LIMITS + "/" + t_name + "/" + CONF_THRUSTER_MIN, angle_min_max[0]);
            this->get_parameter(std::string()+CONF_THRUSTER_SERVO_LIMITS + "/" + t_name + "/" + CONF_THRUSTER_MIN, t->m_servo_angle_min);

            this->declare_parameter(std::string()+CONF_THRUSTER_SERVO_LIMITS + "/" + t_name + "/" + CONF_THRUSTER_MAX, angle_min_max[1]);
            this->get_parameter(std::string()+CONF_THRUSTER_SERVO_LIMITS + "/" + t_name + "/" + CONF_THRUSTER_MAX, t->m_servo_angle_max);


            //set servo angle to zero
            t->set_thruster_servo_angle(0.0);
            t->set_thruster_direction(1.0);
            t->set_thruster_auto_mode(m_vector_thruster_auto_direction);

            m_vector_thrusters.emplace_back(t);
        }


    }

    f_amend_control_mode(*modes.begin());

}


bool MvpControlROS::f_cb_srv_get_control_modes(
    const std::shared_ptr<GetControlModes::Request> req,
    const std::shared_ptr<GetControlModes::Response> resp) {

    if(!m_control_modes.modes.empty()) {
        resp->modes = m_control_modes.modes;
        return true;
    } else {
        return false;
    }

}

bool MvpControlROS::f_cb_srv_set_control_point(
    const std::shared_ptr<SetControlPoint::Request> req,
    const std::shared_ptr<SetControlPoint::Response> resp) {
    
    mvp_msgs::msg::ControlProcess::SharedPtr msg = std::make_shared<mvp_msgs::msg::ControlProcess>(req->setpoint);
    Eigen::VectorXd m_i(CONTROLLABLE_DOF_LENGTH);
    m_i.setZero();
    m_mvp_control->get_pid()->set_m_i(m_i);
    return f_amend_set_point(msg);

}


bool MvpControlROS::f_cb_srv_set_controller(
            const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
            const std::shared_ptr<std_srvs::srv::SetBool::Response> resp){
                
    m_enabled = req->data;
    resp->success = true;
    resp->message = m_enabled ? "Controller enabled." : "Controller disabled.";

    RCLCPP_INFO_STREAM(this->get_logger(), resp->message.c_str());

    std_msgs::msg::Bool msg;
    msg.data = m_enabled;

    // Publish the message
    m_controller_state_publisher->publish(msg);
    Eigen::VectorXd m_i(CONTROLLABLE_DOF_LENGTH);
    m_i.setZero();
    m_mvp_control->get_pid()->set_m_i(m_i);
    return true;
}

bool MvpControlROS::f_cb_srv_get_active_mode(
    const std::shared_ptr<GetControlMode::Request> req,
    const std::shared_ptr<GetControlMode::Response> resp) {

    auto found =
        std::find_if(
            m_control_modes.modes.begin(),
            m_control_modes.modes.end(),
            [this](const mvp_msgs::msg::ControlMode &t) -> bool {
                 if(this->m_control_mode == t.name) {
                     return true;
                 }
                 return false;
            }
        );

    resp->mode = *found;

    return true;
}

Eigen::MatrixXd MvpControlROS::f_angular_velocity_transform(const geometry_msgs::msg::TransformStamped& tf) 
{
    
    tf2::Quaternion quat;
    quat.setW(tf.transform.rotation.w);
    quat.setX(tf.transform.rotation.x);
    quat.setY(tf.transform.rotation.y);
    quat.setZ(tf.transform.rotation.z);

    Eigen::Vector3d orientation;
    tf2::Matrix3x3(quat).getRPY(orientation.x(), orientation.y(), orientation.z());

    Eigen::Matrix3d transform = Eigen::Matrix3d::Zero();

    double cosy = cos(orientation.y());
    double tany = tan(orientation.y());

    if(cosy >-0.0001 && cosy <0.0001){
        cosy = 0.0001;
    }

    tany = std::min(std::max(tany, -1000.0), 1000.0);

    transform(0,0) = 1.0;
    transform(0,1) = sin(orientation.x()) * tany;
    transform(0,2) = cos(orientation.x()) * tany;
    transform(1,0) = 0.0;
    transform(1,1) = cos(orientation.x());
    transform(1,2) = -sin(orientation.x());
    transform(2,0) = 0.0;
    transform(2,1) = sin(orientation.x()) / cosy;
    transform(2,2) = cos(orientation.x()) / cosy;


    return transform;
}

bool MvpControlROS::f_amend_control_mode(std::string mode) {
    if(!mode.empty()) {
        if(mode == m_control_mode) {
            // nothing should change. Operation valid
            return true;
        }

        auto found = std::find_if(
                m_control_modes.modes.begin(),
                m_control_modes.modes.end(),
                [mode](const mvp_msgs::msg::ControlMode& m) -> bool {
                if(m.name == mode) {
                    return true;
                }
                return false;
            }
        );

        if(found == m_control_modes.modes.end()) {
            RCLCPP_WARN_STREAM(this->get_logger(), "Requested mode [" << mode << "] doesn't exist. ");
            // mode doesn't exist. Operation invalid
            return false;
        }

        // update PID gains
        Eigen::VectorXd p(CONTROLLABLE_DOF_LENGTH);
        Eigen::VectorXd i(CONTROLLABLE_DOF_LENGTH);
        Eigen::VectorXd d(CONTROLLABLE_DOF_LENGTH);
        Eigen::VectorXd v(CONTROLLABLE_DOF_LENGTH);

        Eigen::VectorXd pid_max(CONTROLLABLE_DOF_LENGTH);
        Eigen::VectorXd pid_min(CONTROLLABLE_DOF_LENGTH);

        p <<
                found->pid_x.kp,
                found->pid_y.kp,
                found->pid_z.kp,
                found->pid_roll.kp,
                found->pid_pitch.kp,
                found->pid_yaw.kp,
                found->pid_u.kp,
                found->pid_v.kp,
                found->pid_w.kp,
                found->pid_p.kp,
                found->pid_q.kp,
                found->pid_r.kp;

        i <<
                found->pid_x.ki,
                found->pid_y.ki,
                found->pid_z.ki,
                found->pid_roll.ki,
                found->pid_pitch.ki,
                found->pid_yaw.ki,
                found->pid_u.ki,
                found->pid_v.ki,
                found->pid_w.ki,
                found->pid_p.ki,
                found->pid_q.ki,
                found->pid_r.ki;

        d <<
                found->pid_x.kd,
                found->pid_y.kd,
                found->pid_z.kd,
                found->pid_roll.kd,
                found->pid_pitch.kd,
                found->pid_yaw.kd,
                found->pid_u.kd,
                found->pid_v.kd,
                found->pid_w.kd,
                found->pid_p.kd,
                found->pid_q.kd,
                found->pid_r.kd;

        v <<
                found->pid_x.kv,
                found->pid_y.kv,
                found->pid_z.kv,
                found->pid_roll.kv,
                found->pid_pitch.kv,
                found->pid_yaw.kv,
                found->pid_u.kv,
                found->pid_v.kv,
                found->pid_w.kv,
                found->pid_p.kv,
                found->pid_q.kv,
                found->pid_r.kv;

        pid_max <<
                found->pid_x.pid_max,
                found->pid_y.pid_max,
                found->pid_z.pid_max,
                found->pid_roll.pid_max,
                found->pid_pitch.pid_max,
                found->pid_yaw.pid_max,
                found->pid_u.pid_max,
                found->pid_v.pid_max,
                found->pid_w.pid_max,
                found->pid_p.pid_max,
                found->pid_q.pid_max,
                found->pid_r.pid_max;
        pid_min <<
                found->pid_x.pid_min,
                found->pid_y.pid_min,
                found->pid_z.pid_min,
                found->pid_roll.pid_min,
                found->pid_pitch.pid_min,
                found->pid_yaw.pid_min,
                found->pid_u.pid_min,
                found->pid_v.pid_min,
                found->pid_w.pid_min,
                found->pid_p.pid_min,
                found->pid_q.pid_min,
                found->pid_r.pid_min;

        m_mvp_control->get_pid()->set_kp(p);
        m_mvp_control->get_pid()->set_ki(i);
        m_mvp_control->get_pid()->set_kd(d);
        m_mvp_control->get_pid()->set_kv(v);

        m_mvp_control->get_pid()->set_pid_max(pid_max);
        m_mvp_control->get_pid()->set_pid_min(pid_min);

        m_control_mode = mode;

        m_mvp_control->update_freedoms(found->dofs);
        
        RCLCPP_INFO_STREAM(this->get_logger(), "Controller mode changed to " << mode);

        // mode is not empty. mode is in the modes list. operation is valid.
        return true;
    } else {

        // its empty, operation valid.
        return true;
    }
}


bool MvpControlROS::f_amend_set_point(
    const mvp_msgs::msg::ControlProcess::SharedPtr set_point) {
    rclcpp::Time now = this->get_clock()->now();

    auto steady_clock = rclcpp::Clock();    

    if(!f_amend_control_mode(set_point->control_mode)) {
        return false;
    }

    if( set_point->header.frame_id.empty() || set_point->child_frame_id.empty() ) {
        // no decision can be made
        RCLCPP_WARN_STREAM(this->get_logger(), "no frame id provided for the setpoint!");

        return false;
    }

    //if there is a frame id change we regenerate the allocation matrix
    if(set_point->header.frame_id != m_world_link_id || 
       set_point->child_frame_id != m_child_link_id )
    {
        RCLCPP_WARN_STREAM(this->get_logger(), "control frame changed, will regenerate allocation matrix");
        m_child_link_id = set_point->child_frame_id;
        m_world_link_id = set_point->header.frame_id;
        f_generate_control_allocation_matrix();

    }
    
    Eigen::VectorXd new_set_point(CONTROLLABLE_DOF_LENGTH);
    Eigen::VectorXd m_i(CONTROLLABLE_DOF_LENGTH);
    m_i = m_mvp_control->get_pid()->get_m_i();

    new_set_point(mvp_msgs::msg::ControlMode::DOF_X) = set_point->position.x;
    new_set_point(mvp_msgs::msg::ControlMode::DOF_Y) = set_point->position.y;
    new_set_point(mvp_msgs::msg::ControlMode::DOF_Z) = set_point->position.z;
    new_set_point(mvp_msgs::msg::ControlMode::DOF_ROLL) = set_point->orientation.x;
    new_set_point(mvp_msgs::msg::ControlMode::DOF_PITCH) = set_point->orientation.y;
    new_set_point(mvp_msgs::msg::ControlMode::DOF_YAW) = set_point->orientation.z;
    new_set_point(mvp_msgs::msg::ControlMode::DOF_U) = set_point->velocity.x;
    new_set_point(mvp_msgs::msg::ControlMode::DOF_V) = set_point->velocity.y;
    new_set_point(mvp_msgs::msg::ControlMode::DOF_W) = set_point->velocity.z;
    new_set_point(mvp_msgs::msg::ControlMode::DOF_P) = set_point->angular_rate.x;
    new_set_point(mvp_msgs::msg::ControlMode::DOF_Q) = set_point->angular_rate.y;
    new_set_point(mvp_msgs::msg::ControlMode::DOF_R) = set_point->angular_rate.z;

    //check if set point has changed for any dof
    for (int i = 0; i< m_set_point.size(); ++i)
    {
        //if has changed, we zero the integral term for that dof.
        if(m_set_point[i] != new_set_point[i])
        //if the set point is not so close
        // if( abs( m_set_point[i] - new_set_point[i] ) > 0.001) $maybe
        {
            m_i[i]=0;
        }
    }
    //set integral 
    m_mvp_control->get_pid()->set_m_i(m_i);
    //update the setpoint
    m_set_point = new_set_point;
    //set the desired state to the controller
    m_mvp_control->update_desired_state(m_set_point);

    m_set_point_msg = *set_point;

    return true;
}
