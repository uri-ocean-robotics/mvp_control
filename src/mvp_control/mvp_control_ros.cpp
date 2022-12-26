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

#include "mvp_control_ros.hpp"
#include "exception.hpp"
#include "tf2_eigen/tf2_eigen.h"

#include "mvp_control/dictionary.h"
#include "regex"
#include "boost/regex.hpp"

using namespace mvp;

MvpControlROS::MvpControlROS() {

    m_nh = ros::NodeHandle();

    m_pnh = ros::NodeHandle("~");

    m_transform_listener.reset(new tf2_ros::TransformListener(m_transform_buffer));

    m_process_values = Eigen::VectorXd::Zero(CONTROLLABLE_DOF_LENGTH);

    m_set_point = Eigen::VectorXd::Zero(CONTROLLABLE_DOF_LENGTH);

    f_read_configuration();

    f_initialize_ros_interface();

    /**
     * Initialize the actual controller
     */
    m_control_allocation.reset(new ControlAllocation());

}

void MvpControlROS::f_read_configuration() {
    /**
      * Read basic configuration. Configuration regarding to thruster allocation
      * will be read later.
      */

    // Read configuration: enabled
    m_pnh.param<bool>(CONF_ENABLED, m_config.enabled, false);

    // Read configuration: tf prefix
    m_pnh.param<std::string>(CONF_TF_PREFIX, m_config.tf_prefix, CONF_TF_PREFIX_DEFAULT);
    m_config.tf_prefix = m_config.tf_prefix.empty() ? CONF_TF_PREFIX_DEFAULT : m_config.tf_prefix + "/";

    // Read configuration: center of gravity link
    m_pnh.param<std::string>(CONF_CG_LINK, m_config.cg_link_id, CONF_CG_LINK_DEFAULT);
    m_config.cg_link_id = m_config.tf_prefix + m_config.cg_link_id;

    // Read configuration: world link
    m_pnh.param<std::string>(
        CONF_WORLD_LINK,
        m_config.world_link,
        CONF_WORLD_LINK_DEFAULT
    );

    // Read configuration: odometry topic id
    m_pnh.param<std::string>(
        CONF_ODOMETRY_SOURCE,
        m_config.odom_topic_id,
        CONF_ODOMETRY_SOURCE_DEFAULT
    );

    m_pnh.param<double>(
        CONF_CONTROLLER_FREQUENCY,
        m_config.frequency,
        10.0
    );

    m_pnh.param<bool>(
        "periodic_control_allocation_update",
        m_config.period_control_allocation_update,
        false
    );
}

void MvpControlROS::f_check_configuration() {

    // Conduct some checks to see if everything is ready to be initialized
    if(m_actuators.empty()) {
        throw control_exception("no thruster specified");
    }
}

void MvpControlROS::f_initialize_ros_interface() {
    /**
     * Initialize Subscribers
     */
    m_odometry_subscriber = m_nh.subscribe(
        m_config.odom_topic_id,
        100,
        &MvpControlROS::f_cb_msg_odometry,
        this
    );

    m_set_point_subscriber = m_nh.subscribe(
        TOPIC_CONTROL_PROCESS_SET_POINT,
        100,
        &MvpControlROS::f_cb_set_point,
        this
    );

    /**
     * Initialize publishers
     */
    m_process_value_publisher = m_nh.advertise<mvp_msgs::ControlProcess>(
        TOPIC_CONTROL_PROCESS_VALUE,
        100
    );

    m_process_error_publisher = m_nh.advertise<mvp_msgs::ControlProcess>(
        TOPIC_CONTROL_PROCESS_ERROR,
        100
    );
    /**
     * Initialize services
     */
    m_get_control_modes_server = m_nh.advertiseService
        <mvp_msgs::GetControlModes::Request,
            mvp_msgs::GetControlModes::Response>
        (
            SERVICE_GET_CONTROL_MODES,
            std::bind(
                &MvpControlROS::f_cb_srv_get_control_modes,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )
        );

    m_enable_controller_server = m_nh.advertiseService
        <std_srvs::Empty::Request,
            std_srvs::Empty::Response>
        (
            SERVICE_CONTROL_ENABLE,
            std::bind(
                &MvpControlROS::f_cb_srv_enable,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )
        );

    m_disable_controller_server = m_nh.advertiseService
        <std_srvs::Empty::Request,
            std_srvs::Empty::Response>
        (
            SERVICE_CONTROL_DISABLE,
            std::bind(
                &MvpControlROS::f_cb_srv_disable,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )
        );

    m_get_active_mode_server = m_nh.advertiseService
        <mvp_msgs::GetControlMode::Request,
            mvp_msgs::GetControlMode::Response>
        (
            SERVICE_GET_ACTIVE_MODE,
            std::bind(
                &MvpControlROS::f_cb_srv_get_active_mode,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )
        );

    /**
     * Initialize dynamic reconfigure server
     */
    m_dynconf_pid_server.reset(
        new dynamic_reconfigure::Server<mvp_control::PIDConfig>(
            m_config_lock)
    );
}

void MvpControlROS::f_generate_control_allocation_matrix() {

    f_generate_control_allocation_from_tf(false);

    // Initialize the control allocation matrix based on zero matrix.
    // M by N matrix. M -> number of all controllable DOF, N -> number of
    // thrusters times 3. This is because, control allocation matrix will take
    // thrusters contribution to the motion in all 3 axis.
    Eigen::MatrixXd T(CONTROLLABLE_DOF_LENGTH, 3 * m_actuators.size());
    for(const auto& i : m_actuators) {
        T << i->get_contribution_matrix();
    }

    // Finally, set the control allocation matrix for the controller object.
    m_control_allocation->set_control_allocation_matrix(T);

}

void MvpControlROS::initialize() {

    // Read configured control modes from the ROS parameter server
    f_read_control_modes();

    // Generate thrusters with the given configuration
    f_read_actuators();

    std::exit(EXIT_SUCCESS);

    // Generate control allocation matrix with defined method
    f_generate_control_allocation_matrix();

    // Initialize thruster objects.
    std::for_each(m_actuators.begin(),m_actuators.end(),
        [](const auto& t){
            t->initialize();
        }
    );

    m_control_allocation->set_desired_state(m_set_point);

    m_control_allocation->set_system_state(m_process_values);

    m_controller_worker = std::thread([this] { f_control_loop(); });

    m_controller_worker.detach();

    m_dynconf_pid_server->setCallback(
        std::bind(
            &MvpControlROS::f_cb_dynconf_pid,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

}

void MvpControlROS::f_generate_control_allocation_from_tf(bool update) {

    // For each thruster look up transformation
    for(const auto& i : m_actuators) {

        // loop up for the transformation
        auto tf = m_transform_buffer.lookupTransform(
            m_config.cg_link_id,
            i->get_config().link_id,
            ros::Time::now(),
            ros::Duration(10.0)
        );

        // change type to Eigen::Isometry3D
        auto eigen_tf = tf2::transformToEigen(tf);

        // compute contributions
        i->compute_contribution(eigen_tf);

        // compute constraint matrix
        i->compute_constraints();
    }

    // compose constraint matrix, allocation matrix, and bound vectors
    if(not update) {
        m_control_allocation->compose_matrices(m_actuators);
    }

}

bool MvpControlROS::f_update_control_allocation_matrix() {

    if(m_config.period_control_allocation_update) {
        f_generate_control_allocation_from_tf(true);
    }

    try {

        // Transform center of gravity to world
        auto cg_world = m_transform_buffer.lookupTransform(
            m_config.world_link,
            m_config.cg_link_id,
            ros::Time::now(),
            ros::Duration(5.0)
        );

        // get tf2 transform to eigen transform
        auto pose_transform = tf2::transformToEigen(cg_world);

        m_control_allocation->update_for_pose(pose_transform);

    } catch(tf2::TransformException& e) {
        ROS_WARN_STREAM_THROTTLE(10, std::string(
            "Can't update control allocation matrix ") + e.what());
        return false;
    }

    return true;
}

bool MvpControlROS::f_compute_process_values() {

    f_update_control_allocation_matrix();

    try {
        // Transform center of gravity to world
        auto cg_world = m_transform_buffer.lookupTransform(
            m_config.world_link,
            m_config.cg_link_id,
            ros::Time::now(),
            ros::Duration(10.0)
        );

        tf2::Quaternion quat;
        quat.setW(cg_world.transform.rotation.w);
        quat.setX(cg_world.transform.rotation.x);
        quat.setY(cg_world.transform.rotation.y);
        quat.setZ(cg_world.transform.rotation.z);

        tf2::Matrix3x3(quat).getRPY(
            m_process_values(DOF::ROLL),
            m_process_values(DOF::PITCH),
            m_process_values(DOF::YAW)
        );

        m_process_values(DOF::X) = cg_world.transform.translation.x;
        m_process_values(DOF::Y) = cg_world.transform.translation.y;
        m_process_values(DOF::Z) = cg_world.transform.translation.z;

    } catch(tf2::TransformException &e) {
        ROS_WARN_STREAM_THROTTLE(10, std::string("Can't compute process values: ") + e.what());
        return false;
    }
        // Transform from odom to world
    try{
        std::scoped_lock lock(m_odom_lock);

        auto cg_odom = m_transform_buffer.lookupTransform(
                m_config.cg_link_id,
                m_odometry_msg.child_frame_id,
                ros::Time::now(),
                ros::Duration(10)
        );

        auto cg_odom_eigen = tf2::transformToEigen(cg_odom);

        Eigen::Vector3d uvw;
        uvw(0) = m_odometry_msg.twist.twist.linear.x;
        uvw(1) = m_odometry_msg.twist.twist.linear.y;
        uvw(2) = m_odometry_msg.twist.twist.linear.z;

        uvw = cg_odom_eigen.rotation()  * uvw;

        m_process_values(DOF::SURGE) = uvw(0);
        m_process_values(DOF::SWAY) = uvw(1);
        m_process_values(DOF::HEAVE) = uvw(2);

        Eigen::Vector3d angular_rate;
        angular_rate(0) = m_odometry_msg.twist.twist.angular.x;
        angular_rate(1) = m_odometry_msg.twist.twist.angular.y;
        angular_rate(2) = m_odometry_msg.twist.twist.angular.z;

        angular_rate = cg_odom_eigen.rotation() * angular_rate;

        m_process_values(DOF::ROLL_RATE) = angular_rate(0);
        m_process_values(DOF::PITCH_RATE) = angular_rate(1);
        m_process_values(DOF::YAW_RATE) = angular_rate(2);

    } catch(tf2::TransformException &e) {
        ROS_WARN_STREAM_THROTTLE(10, std::string("Can't compute process values!, check odometry!: ") + e.what());
        return false;
    }

    mvp_msgs::ControlProcess s;
    s.header.stamp = ros::Time::now();
    s.header.frame_id = m_config.world_link;
    s.control_mode = m_control_mode;
    s.position.x = m_process_values(DOF::X);
    s.position.y = m_process_values(DOF::Y);
    s.position.z = m_process_values(DOF::Z);
    s.orientation.x = m_process_values(DOF::ROLL);
    s.orientation.y = m_process_values(DOF::PITCH);
    s.orientation.z = m_process_values(DOF::YAW);
    s.velocity.x = m_process_values(DOF::SURGE);
    s.velocity.y = m_process_values(DOF::SWAY);
    s.velocity.z = m_process_values(DOF::HEAVE);
    s.angular_rate.x = m_process_values(DOF::ROLL_RATE);
    s.angular_rate.y = m_process_values(DOF::PITCH_RATE);
    s.angular_rate.z = m_process_values(DOF::YAW_RATE);

    m_control_allocation->set_system_state(m_process_values);

    m_process_value_publisher.publish(s);

    mvp_msgs::ControlProcess e;

    Eigen::VectorXd error_state = m_control_allocation->get_state_error();
    e.header.stamp = ros::Time::now();
    e.header.frame_id = m_config.world_link;
    e.control_mode = m_control_mode;
    e.position.x = error_state(DOF::X);
    e.position.y = error_state(DOF::Y);
    e.position.z = error_state(DOF::Z);
    e.orientation.x = error_state(DOF::ROLL);
    e.orientation.y = error_state(DOF::PITCH);
    e.orientation.z = error_state(DOF::YAW);
    e.velocity.x = error_state(DOF::SURGE);
    e.velocity.y = error_state(DOF::SWAY);
    e.velocity.z = error_state(DOF::HEAVE);
    e.angular_rate.x = error_state(DOF::ROLL_RATE);
    e.angular_rate.y = error_state(DOF::PITCH_RATE);
    e.angular_rate.z = error_state(DOF::YAW_RATE);

    m_process_error_publisher.publish(e);

    return true;
}


void MvpControlROS::f_control_loop() {

    auto r = ros::Rate(m_config.frequency);

    while(ros::ok()) {

        /**
         * Thread may not be able to sleep properly. This may happen using
         * simulated time.
         */
        if(!r.sleep()) {
            continue;
        }

        /**
         * Check if controller is enabled or not.
         */
        if(!m_config.enabled) {
            continue;
        }

        /**
         * Compute the state of the system. Continue on failure. This may
         * happen when transform tree is not ready.
         */
        if(not f_compute_process_values()) {
            continue;
        }

        /**
         * Calculate forces to be requested from thrusters. If operation fails,
         * do not send commands to thrusters.
         */
        Eigen::VectorXd needed_forces;
        if(m_control_allocation->calculate_forces(&needed_forces, r.cycleTime().toSec())) {
            for(int i = 0 ; i < m_actuators.size() ; i++) {
                m_actuators.at(i)->command(
                    // read forces as Eigen::Vector3d
                    needed_forces.segment(3 * i, 3)
                );
            }
        }
    }
}

void MvpControlROS::f_cb_msg_odometry(
        const nav_msgs::Odometry::ConstPtr &msg) {
    std::scoped_lock lock(m_odom_lock);
    m_odometry_msg = *msg;
}

void MvpControlROS::f_cb_set_point(
        const mvp_msgs::ControlProcess::ConstPtr &msg) {
    f_amend_set_point(*msg);
}

void MvpControlROS::f_cb_dynconf_pid(
        mvp_control::PIDConfig &config, uint32_t level) {

    Eigen::VectorXd p(CONTROLLABLE_DOF_LENGTH);
    Eigen::VectorXd i(CONTROLLABLE_DOF_LENGTH);
    Eigen::VectorXd d(CONTROLLABLE_DOF_LENGTH);
    Eigen::VectorXd i_max(CONTROLLABLE_DOF_LENGTH);
    Eigen::VectorXd i_min(CONTROLLABLE_DOF_LENGTH);


    p <<
            config.x_p,
            config.y_p,
            config.z_p,
            config.roll_p,
            config.pitch_p,
            config.yaw_p,
            config.surge_p,
            config.sway_p,
            config.heave_p,
            config.roll_rate_p,
            config.pitch_rate_p,
            config.yaw_rate_p;

    i <<
            config.x_i,
            config.y_i,
            config.z_i,
            config.roll_i,
            config.pitch_i,
            config.yaw_i,
            config.surge_i,
            config.sway_i,
            config.heave_i,
            config.roll_rate_i,
            config.pitch_rate_i,
            config.yaw_rate_i;

    d <<
            config.x_d,
            config.y_d,
            config.z_d,
            config.roll_d,
            config.pitch_d,
            config.yaw_d,
            config.surge_d,
            config.sway_d,
            config.heave_d,
            config.roll_rate_d,
            config.pitch_rate_d,
            config.yaw_rate_d;

    i_max <<
            config.x_i_max,
            config.y_i_max,
            config.z_i_max,
            config.roll_i_max,
            config.pitch_i_max,
            config.yaw_i_max,
            config.surge_i_max,
            config.sway_i_max,
            config.heave_i_max,
            config.roll_rate_i_max,
            config.pitch_rate_i_max,
            config.yaw_rate_i_max;

    i_min <<
            config.x_i_min,
            config.y_i_min,
            config.z_i_min,
            config.roll_i_min,
            config.pitch_i_min,
            config.yaw_i_min,
            config.surge_i_min,
            config.sway_i_min,
            config.heave_i_min,
            config.roll_rate_i_min,
            config.pitch_rate_i_min,
            config.yaw_rate_i_min;

    m_control_allocation->get_pid()->set_kp(p);
    m_control_allocation->get_pid()->set_ki(i);
    m_control_allocation->get_pid()->set_kd(d);
    m_control_allocation->get_pid()->set_i_max(i_max);
    m_control_allocation->get_pid()->set_i_min(i_min);

}

void MvpControlROS::f_amend_dynconf() {

    std::scoped_lock lock(m_config_lock);

    auto pid = m_control_allocation->get_pid();

    mvp_control::PIDConfig conf;

    conf.x_p = pid->get_kp()(DOF::X);
    conf.y_p = pid->get_kp()(DOF::Y);
    conf.z_p = pid->get_kp()(DOF::Z);
    conf.roll_p = pid->get_kp()(DOF::ROLL);
    conf.pitch_p = pid->get_kp()(DOF::PITCH);
    conf.yaw_p = pid->get_kp()(DOF::YAW);
    conf.surge_p = pid->get_kp()(DOF::SURGE);
    conf.sway_p = pid->get_kp()(DOF::SWAY);
    conf.heave_p = pid->get_kp()(DOF::HEAVE);
    conf.roll_rate_p = pid->get_kp()(DOF::ROLL_RATE);
    conf.pitch_rate_p = pid->get_kp()(DOF::PITCH_RATE);
    conf.yaw_rate_p = pid->get_kp()(DOF::YAW_RATE);

    conf.x_i = pid->get_ki()(DOF::X);
    conf.y_i = pid->get_ki()(DOF::Y);
    conf.z_i = pid->get_ki()(DOF::Z);
    conf.roll_i = pid->get_ki()(DOF::ROLL);
    conf.pitch_i = pid->get_ki()(DOF::PITCH);
    conf.yaw_i = pid->get_ki()(DOF::YAW);
    conf.surge_i = pid->get_ki()(DOF::SURGE);
    conf.sway_i = pid->get_ki()(DOF::SWAY);
    conf.heave_i = pid->get_ki()(DOF::HEAVE);
    conf.roll_rate_i = pid->get_ki()(DOF::ROLL_RATE);
    conf.pitch_rate_i = pid->get_ki()(DOF::PITCH_RATE);
    conf.yaw_rate_i = pid->get_ki()(DOF::YAW_RATE);

    conf.x_d = pid->get_kd()(DOF::X);
    conf.y_d = pid->get_kd()(DOF::Y);
    conf.z_d = pid->get_kd()(DOF::Z);
    conf.roll_d = pid->get_kd()(DOF::ROLL);
    conf.pitch_d = pid->get_kd()(DOF::PITCH);
    conf.yaw_d = pid->get_kd()(DOF::YAW);
    conf.surge_d = pid->get_kd()(DOF::SURGE);
    conf.sway_d = pid->get_kd()(DOF::SWAY);
    conf.heave_d = pid->get_kd()(DOF::HEAVE);
    conf.roll_rate_d = pid->get_kd()(DOF::ROLL_RATE);
    conf.pitch_rate_d = pid->get_kd()(DOF::PITCH_RATE);
    conf.yaw_rate_d = pid->get_kd()(DOF::YAW_RATE);

    conf.x_i_max = pid->get_i_max()(DOF::X);
    conf.y_i_max = pid->get_i_max()(DOF::Y);
    conf.z_i_max = pid->get_i_max()(DOF::Z);
    conf.roll_i_max = pid->get_i_max()(DOF::ROLL);
    conf.pitch_i_max = pid->get_i_max()(DOF::PITCH);
    conf.yaw_i_max = pid->get_i_max()(DOF::YAW);
    conf.surge_i_max = pid->get_i_max()(DOF::SURGE);
    conf.sway_i_max = pid->get_i_max()(DOF::SWAY);
    conf.heave_i_max = pid->get_i_max()(DOF::HEAVE);
    conf.roll_rate_i_max = pid->get_i_max()(DOF::ROLL_RATE);
    conf.pitch_rate_i_max = pid->get_i_max()(DOF::PITCH_RATE);
    conf.yaw_rate_i_max = pid->get_i_max()(DOF::YAW_RATE);

    conf.x_i_min = pid->get_i_min()(DOF::X);
    conf.y_i_min = pid->get_i_min()(DOF::Y);
    conf.z_i_min = pid->get_i_min()(DOF::Z);
    conf.roll_i_min = pid->get_i_min()(DOF::ROLL);
    conf.pitch_i_min = pid->get_i_min()(DOF::PITCH);
    conf.yaw_i_min = pid->get_i_min()(DOF::YAW);
    conf.surge_i_min = pid->get_i_min()(DOF::SURGE);
    conf.sway_i_min = pid->get_i_min()(DOF::SWAY);
    conf.heave_i_min = pid->get_i_min()(DOF::HEAVE);
    conf.roll_rate_i_min = pid->get_i_min()(DOF::ROLL_RATE);
    conf.pitch_rate_i_min = pid->get_i_min()(DOF::PITCH_RATE);
    conf.yaw_rate_i_min = pid->get_i_min()(DOF::YAW_RATE);

    m_dynconf_pid_server->updateConfig(conf);

}

void MvpControlROS::f_prep_parameter(std::string *param) {

}

void MvpControlROS::f_read_actuators() {
    std::vector<std::string> params;

    m_pnh.getParamNames(params);

    // Get the list of thrusters
    std::set<std::string> actuators;
    for (const auto &i: params) {
        boost::regex e{
            std::string() + "(?<=" + CONF_ACTUATORS + "/)(\\w+)"};
        boost::smatch w;
        if (boost::regex_search(i, w, e)) {
            actuators.insert(w[0]);
        }
    }

    // create a vector to hold the actuator options
    std::vector<ActuatorConfig> actuator_config;

    // read & write the options
    for(const auto&i : actuators) {
        ActuatorConfig config;

        config.name = i;

        std::string sopt;
        std::string param;

        // Read actuator type
        param = std::string(CONF_ACTUATORS) + "/" + i + "/type";
        if(!m_pnh.getParam(param, sopt) ) {
            throw control_exception("actuator without type!");
        }
        if(sopt == CONF_ACTUATOR_TYPE_THRUSTER) {
            config.type = ActuatorType::THRUSTER;
        } else if (sopt == CONF_ACTUATOR_TYPE_AZIMUTH_THRUSTER) {
            config.type = ActuatorType::AZIMUTH_THRUSTER;
        } else if (sopt == CONF_ACTUATOR_TYPE_RUDDER) {
            config.type = ActuatorType::RUDDER;
        } else {
            config.type = ActuatorType::UNDEFINED;
            std::cerr << "Actuator type " << sopt << " for "
                      << config.name << " is not valid" << std::endl;
            continue;
        }

        param = std::string(std::string(CONF_ACTUATORS) + "/" + i + "/link_name");
        if(!m_pnh.getParam(param, config.link_id)) {
            throw control_exception("actuator without link name!");
        }

        param = std::string(CONF_ACTUATORS) + "/" + i + "/force_topic";
        m_pnh.param(param, config.force_topic, i + "/force");

        param = std::string(CONF_ACTUATORS) + "/" + i + "/control_curve";
        m_pnh.param(param, config.control_curve, std::vector<double>());

        std::vector<double> vopt;
        param = std::string(CONF_ACTUATORS) + "/" + i + "/force_limits";
        if(m_pnh.hasParam(param)) {
            m_pnh.param(param, vopt, std::vector<double>());
            config.force_limits[0] = *std::min_element(vopt.begin(), vopt.end());
            config.force_limits[1] = *std::max_element(vopt.begin(), vopt.end());
        }


        param = std::string(CONF_ACTUATORS) + "/" + i + "/joint/y/limits";
        if(m_pnh.hasParam(param)){
            std::vector<double> limits;
            m_pnh.getParam(param, limits);
            std::sort(limits.begin(), limits.end());
            config.joint_y_limits = limits;
        }

        param = std::string(CONF_ACTUATORS) + "/" + i + "/joint/z/limits";
        if(m_pnh.hasParam(param)){
            std::vector<double> limits;
            m_pnh.getParam(param, limits);
            std::sort(limits.begin(), limits.end());
            config.joint_z_limits = limits;
        }

        param = std::string(CONF_ACTUATORS) + "/" + i + "/joint/y/joint_name";
        this->param(param, config.joint_y_name, i + "/joint/y");

        param = std::string(CONF_ACTUATORS) + "/" + i + "/joint/z/joint_name";
        this->param(param, config.joint_z_name, i + "/joint/z");

        actuator_config.push_back(config);
    }

    for(const auto& i : actuator_config) {
        switch(i.type) {
            case ActuatorType::THRUSTER:
                m_actuators.emplace_back(std::make_shared<Thruster>(i));
                break;
            case ActuatorType::AZIMUTH_THRUSTER:
                m_actuators.emplace_back(std::make_shared<AzimuthThruster>(i));
                break;
            default:
                // unsupported actuator
                break;
        }
    }

}

void MvpControlROS::f_read_control_modes() {
    std::vector<std::string> params;
    m_pnh.getParamNames(params);

    /**
     * Read all the modes with regex
     */
    std::set<std::string> modes;
    for (const auto &i: params) {
        boost::regex e{
            std::string() + "(?<=" + CONF_CONTROL_MODES + "/)(\\w+)"};
        boost::smatch w;
        if (boost::regex_search(i, w, e)) {
            modes.insert(w[0]);
        }
    }

    if(modes.empty()) {
        /**
         * There is no mode detected by the control mode parser.
         */
         throw control_exception(
             "No control mode configuration have been found."
         );
    }

    /**
     * Read all the degrees of freedoms by a mode
     */
    std::map<std::string, std::set<int>> mode_rules;
    for (const auto &mode: modes) {
        for (const auto &i: params) {
            if (i.find(std::string() + CONF_CONTROL_MODES + "/" + mode) ==
                std::string::npos) {
                continue;
            }
            boost::regex e{std::string() + "(?<=" + mode + "/)(\\w+)"};
            boost::smatch w;
            if (!boost::regex_search(i, w, e)) {
                continue;
            }
            std::string dof = w[0]; // dof name

            auto found =
                std::find_if(CONF_DOF_LOOKUP.begin(), CONF_DOF_LOOKUP.end(),
                    [dof](const std::pair<const char *, int> &t) -> bool {
                        return std::strcmp(dof.c_str(),t.first) == 0;
                    }
                );

            if (found != CONF_DOF_LOOKUP.end()) {
                mode_rules[mode].insert(found->second);
            } else {
                throw control_exception(
                        "Unknown freedom name passed '" + dof + "'"
                        "Possible values are "
                       "'x, y, z, roll, pitch, yaw, surge, sway, heave"
                );
            }
        }
    }

    // Loop through all the modes and break them down
    for (const auto &mode: modes) {
        mvp_msgs::ControlMode m;

        m.name = mode;

        m.dofs = std::vector<int>(
            mode_rules[mode].begin(), mode_rules[mode].end());

        for (const auto &dof: mode_rules[mode]) {
            std::string param;
            param += std::string() + CONF_CONTROL_MODES + "/" + mode + "/" +
                     DOFS[dof] + "/";
            if (dof == DOF::X) {
                m_pnh.param<double>(param + CONF_PID_P, m.pid_x.kp, 0);
                m_pnh.param<double>(param + CONF_PID_I, m.pid_x.ki, 0);
                m_pnh.param<double>(param + CONF_PID_D, m.pid_x.kd, 0);
                m_pnh.param<double>(param + CONF_PID_I_MAX, m.pid_x.k_i_max, 0);
                m_pnh.param<double>(param + CONF_PID_I_MIN, m.pid_x.k_i_min, 0);
            } else if (dof == DOF::Y) {
                m_pnh.param<double>(param + CONF_PID_P, m.pid_y.kp, 0);
                m_pnh.param<double>(param + CONF_PID_I, m.pid_y.ki, 0);
                m_pnh.param<double>(param + CONF_PID_D, m.pid_y.kd, 0);
                m_pnh.param<double>(param + CONF_PID_I_MAX, m.pid_y.k_i_max, 0);
                m_pnh.param<double>(param + CONF_PID_I_MIN, m.pid_y.k_i_min, 0);
            } else if (dof == DOF::Z) {
                m_pnh.param<double>(param + CONF_PID_P, m.pid_z.kp, 0);
                m_pnh.param<double>(param + CONF_PID_I, m.pid_z.ki, 0);
                m_pnh.param<double>(param + CONF_PID_D, m.pid_z.kd, 0);
                m_pnh.param<double>(param + CONF_PID_I_MAX, m.pid_z.k_i_max, 0);
                m_pnh.param<double>(param + CONF_PID_I_MIN, m.pid_z.k_i_min, 0);
            } else if (dof == DOF::ROLL) {
                m_pnh.param<double>(param + CONF_PID_P, m.pid_roll.kp, 0);
                m_pnh.param<double>(param + CONF_PID_I, m.pid_roll.ki, 0);
                m_pnh.param<double>(param + CONF_PID_D, m.pid_roll.kd, 0);
                m_pnh.param<double>(param + CONF_PID_I_MAX, m.pid_roll.k_i_max,
                                    0);
                m_pnh.param<double>(param + CONF_PID_I_MIN, m.pid_roll.k_i_min,
                                    0);
            } else if (dof == DOF::PITCH) {
                m_pnh.param<double>(param + CONF_PID_P, m.pid_pitch.kp, 0);
                m_pnh.param<double>(param + CONF_PID_I, m.pid_pitch.ki, 0);
                m_pnh.param<double>(param + CONF_PID_D, m.pid_pitch.kd, 0);
                m_pnh.param<double>(param + CONF_PID_I_MAX, m.pid_pitch.k_i_max,
                                    0);
                m_pnh.param<double>(param + CONF_PID_I_MIN, m.pid_pitch.k_i_min,
                                    0);
            } else if (dof == DOF::YAW) {
                m_pnh.param<double>(param + CONF_PID_P, m.pid_yaw.kp, 0);
                m_pnh.param<double>(param + CONF_PID_I, m.pid_yaw.ki, 0);
                m_pnh.param<double>(param + CONF_PID_D, m.pid_yaw.kd, 0);
                m_pnh.param<double>(param + CONF_PID_I_MAX, m.pid_yaw.k_i_max,
                                    0);
                m_pnh.param<double>(param + CONF_PID_I_MIN, m.pid_yaw.k_i_min,
                                    0);
            } else if (dof == DOF::SURGE) {
                m_pnh.param<double>(param + CONF_PID_P, m.pid_surge.kp, 0);
                m_pnh.param<double>(param + CONF_PID_I, m.pid_surge.ki, 0);
                m_pnh.param<double>(param + CONF_PID_D, m.pid_surge.kd, 0);
                m_pnh.param<double>(param + CONF_PID_I_MAX, m.pid_surge.k_i_max,
                                    0);
                m_pnh.param<double>(param + CONF_PID_I_MIN, m.pid_surge.k_i_min,
                                    0);
            } else if (dof == DOF::SWAY) {
                m_pnh.param<double>(param + CONF_PID_P, m.pid_sway.kp, 0);
                m_pnh.param<double>(param + CONF_PID_I, m.pid_sway.ki, 0);
                m_pnh.param<double>(param + CONF_PID_D, m.pid_sway.kd, 0);
                m_pnh.param<double>(param + CONF_PID_I_MAX, m.pid_sway.k_i_max,
                                    0);
                m_pnh.param<double>(param + CONF_PID_I_MIN, m.pid_sway.k_i_min,
                                    0);
            } else if (dof == DOF::HEAVE) {
                m_pnh.param<double>(param + CONF_PID_P, m.pid_heave.kp, 0);
                m_pnh.param<double>(param + CONF_PID_I, m.pid_heave.ki, 0);
                m_pnh.param<double>(param + CONF_PID_D, m.pid_heave.kd, 0);
                m_pnh.param<double>(param + CONF_PID_I_MAX, m.pid_heave.k_i_max,
                                    0);
                m_pnh.param<double>(param + CONF_PID_I_MIN, m.pid_heave.k_i_min,
                                    0);
            } else if (dof == DOF::ROLL_RATE) {
                m_pnh.param<double>(param + CONF_PID_P, m.pid_roll_rate.kp, 0);
                m_pnh.param<double>(param + CONF_PID_I, m.pid_roll_rate.ki, 0);
                m_pnh.param<double>(param + CONF_PID_D, m.pid_roll_rate.kd, 0);
                m_pnh.param<double>(param + CONF_PID_I_MAX,
                                    m.pid_roll_rate.k_i_max, 0);
                m_pnh.param<double>(param + CONF_PID_I_MIN,
                                    m.pid_roll_rate.k_i_min, 0);
            } else if (dof == DOF::PITCH_RATE) {
                m_pnh.param<double>(param + CONF_PID_P, m.pid_pitch_rate.kp, 0);
                m_pnh.param<double>(param + CONF_PID_I, m.pid_pitch_rate.ki, 0);
                m_pnh.param<double>(param + CONF_PID_D, m.pid_pitch_rate.kd, 0);
                m_pnh.param<double>(param + CONF_PID_I_MAX,
                                    m.pid_pitch_rate.k_i_max, 0);
                m_pnh.param<double>(param + CONF_PID_I_MIN,
                                    m.pid_pitch_rate.k_i_min, 0);
            } else if (dof == DOF::YAW_RATE) {
                m_pnh.param<double>(param + CONF_PID_P, m.pid_yaw_rate.kp, 0);
                m_pnh.param<double>(param + CONF_PID_I, m.pid_yaw_rate.ki, 0);
                m_pnh.param<double>(param + CONF_PID_D, m.pid_yaw_rate.kd, 0);
                m_pnh.param<double>(param + CONF_PID_I_MAX,
                                    m.pid_yaw_rate.k_i_max, 0);
                m_pnh.param<double>(param + CONF_PID_I_MIN,
                                    m.pid_yaw_rate.k_i_min, 0);
            }
        }

        m_control_modes.modes.emplace_back(m);

    }

    f_amend_control_mode(*modes.begin());
}

bool MvpControlROS::f_cb_srv_get_control_modes(
    mvp_msgs::GetControlModes::Request &req,
    mvp_msgs::GetControlModes::Response &resp) const {

    if(!m_control_modes.modes.empty()) {
        resp.modes = m_control_modes.modes;
        return true;
    } else {
        return false;
    }

}

bool MvpControlROS::f_cb_srv_enable(
        std_srvs::Empty::Request req, std_srvs::Empty::Response res) {

    ROS_INFO("Controller enabled!");
    m_config.enabled = true;

    return true;
}

bool MvpControlROS::f_cb_srv_disable(
        std_srvs::Empty::Request req, std_srvs::Empty::Response res) {

    ROS_INFO("Controller disabled!");
    m_config.enabled = false;

    return true;
}

bool MvpControlROS::f_cb_srv_get_active_mode(
    mvp_msgs::GetControlMode::Request& req,
    mvp_msgs::GetControlMode::Response& resp) {

    auto found =
        std::find_if(
            m_control_modes.modes.begin(),
            m_control_modes.modes.end(),
            [this](const mvp_msgs::ControlMode &t) -> bool {
                 if(this->m_control_mode == t.name) {
                     return true;
                 }
                 return false;
            }
        );

    resp.mode = *found;

    return true;
}

bool MvpControlROS::f_amend_control_mode(const std::string& mode) {
    if(!mode.empty()) {
        if(mode == m_control_mode) {
            // nothing should change. Operation valid
            return true;
        }

        auto found = std::find_if(
                m_control_modes.modes.begin(),
                m_control_modes.modes.end(),
                [mode](const mvp_msgs::ControlMode& m) -> bool {
                if(m.name == mode) {
                    return true;
                }
                return false;
            }
        );

        if(found == m_control_modes.modes.end()) {
            ROS_WARN_STREAM(
                    "Requested mode [" << mode << "] doesn't exist. "
            );

            // mode doesn't exist. Operation invalid
            return false;
        }

        // setting PID for requested mode
        mvp_control::PIDConfig pid_conf;
        pid_conf.x_p = found->pid_x.kp;
        pid_conf.x_i = found->pid_x.ki;
        pid_conf.x_d = found->pid_x.kd;
        pid_conf.x_i_max = found->pid_x.k_i_max;
        pid_conf.x_i_min = found->pid_x.k_i_min;

        pid_conf.y_p = found->pid_y.kp;
        pid_conf.y_i = found->pid_y.ki;
        pid_conf.y_d = found->pid_y.kd;
        pid_conf.y_i_max = found->pid_y.k_i_max;
        pid_conf.y_i_min = found->pid_y.k_i_min;

        pid_conf.z_p = found->pid_z.kp;
        pid_conf.z_i = found->pid_z.ki;
        pid_conf.z_d = found->pid_z.kd;
        pid_conf.z_i_max = found->pid_z.k_i_max;
        pid_conf.z_i_min = found->pid_z.k_i_min;

        pid_conf.roll_p = found->pid_roll.kp;
        pid_conf.roll_i = found->pid_roll.ki;
        pid_conf.roll_d = found->pid_roll.kd;
        pid_conf.roll_i_max = found->pid_roll.k_i_max;
        pid_conf.roll_i_min = found->pid_roll.k_i_min;

        pid_conf.pitch_p = found->pid_pitch.kp;
        pid_conf.pitch_i = found->pid_pitch.ki;
        pid_conf.pitch_d = found->pid_pitch.kd;
        pid_conf.pitch_i_max = found->pid_pitch.k_i_max;
        pid_conf.pitch_i_min = found->pid_pitch.k_i_min;

        pid_conf.yaw_p = found->pid_yaw.kp;
        pid_conf.yaw_i = found->pid_yaw.ki;
        pid_conf.yaw_d = found->pid_yaw.kd;
        pid_conf.yaw_i_max = found->pid_yaw.k_i_max;
        pid_conf.yaw_i_min = found->pid_yaw.k_i_min;

        pid_conf.surge_p = found->pid_surge.kp;
        pid_conf.surge_i = found->pid_surge.ki;
        pid_conf.surge_d = found->pid_surge.kd;
        pid_conf.surge_i_max = found->pid_surge.k_i_max;
        pid_conf.surge_i_min = found->pid_surge.k_i_min;

        pid_conf.sway_p = found->pid_sway.kp;
        pid_conf.sway_i = found->pid_sway.ki;
        pid_conf.sway_d = found->pid_sway.kd;
        pid_conf.sway_i_max = found->pid_sway.k_i_max;
        pid_conf.sway_i_min = found->pid_sway.k_i_min;

        pid_conf.heave_p = found->pid_heave.kp;
        pid_conf.heave_i = found->pid_heave.ki;
        pid_conf.heave_d = found->pid_heave.kd;
        pid_conf.heave_i_max = found->pid_heave.k_i_max;
        pid_conf.heave_i_min = found->pid_heave.k_i_min;

        pid_conf.roll_rate_p = found->pid_roll_rate.kp;
        pid_conf.roll_rate_i = found->pid_roll_rate.ki;
        pid_conf.roll_rate_d = found->pid_roll_rate.kd;
        pid_conf.roll_rate_i_max = found->pid_roll_rate.k_i_max;
        pid_conf.roll_rate_i_min = found->pid_roll_rate.k_i_min;

        pid_conf.pitch_rate_p = found->pid_pitch_rate.kp;
        pid_conf.pitch_rate_i = found->pid_pitch_rate.ki;
        pid_conf.pitch_rate_d = found->pid_pitch_rate.kd;
        pid_conf.pitch_rate_i_max = found->pid_pitch_rate.k_i_max;
        pid_conf.pitch_rate_i_min = found->pid_pitch_rate.k_i_min;

        pid_conf.yaw_rate_p = found->pid_yaw_rate.kp;
        pid_conf.yaw_rate_i = found->pid_yaw_rate.ki;
        pid_conf.yaw_rate_d = found->pid_yaw_rate.kd;
        pid_conf.yaw_rate_i_max = found->pid_yaw_rate.k_i_max;
        pid_conf.yaw_rate_i_min = found->pid_yaw_rate.k_i_min;

        f_cb_dynconf_pid(pid_conf, 0);

        f_amend_dynconf();

        m_control_mode = mode;

        m_control_allocation->update_freedoms(found->dofs);

        ROS_INFO_STREAM("Controller mode changed to " << mode);

        // mode is not empty. mode is in the modes list. operation is valid.
        return true;
    } else {

        // its empty, operation valid.
        return true;
    }
}


bool MvpControlROS::f_amend_set_point(
    const mvp_msgs::ControlProcess &set_point) {

    if (!f_amend_control_mode(set_point.control_mode)) {
        return false;
    }

    m_set_point(mvp_msgs::ControlMode::DOF_X) =
        set_point.position.x;
    m_set_point(mvp_msgs::ControlMode::DOF_Y) =
        set_point.position.y;
    m_set_point(mvp_msgs::ControlMode::DOF_Z) =
        set_point.position.z;
    m_set_point(mvp_msgs::ControlMode::DOF_ROLL) =
        set_point.orientation.x;
    m_set_point(mvp_msgs::ControlMode::DOF_PITCH) =
        set_point.orientation.y;
    m_set_point(mvp_msgs::ControlMode::DOF_YAW) =
        set_point.orientation.z;
    m_set_point(mvp_msgs::ControlMode::DOF_SURGE) =
        set_point.velocity.x;
    m_set_point(mvp_msgs::ControlMode::DOF_SWAY) =
        set_point.velocity.y;
    m_set_point(mvp_msgs::ControlMode::DOF_HEAVE) =
        set_point.velocity.z;
    m_set_point(mvp_msgs::ControlMode::DOF_ROLL_RATE) =
        set_point.angular_rate.x;
    m_set_point(mvp_msgs::ControlMode::DOF_PITCH_RATE) =
        set_point.angular_rate.y;
    m_set_point(mvp_msgs::ControlMode::DOF_YAW_RATE) =
        set_point.angular_rate.z;

    m_control_allocation->update_desired_state(m_set_point);

    m_set_point_msg = set_point;

    return true;
}

void MvpControlROS::run() {

    ros::spin();

}

template<class T>
bool MvpControlROS::param(const std::string& param, T &val, const T& default_val) {
    return m_pnh.param<T>(param, val, default_val);
}

template<>
bool MvpControlROS::param<std::string>(const std::string& param, std::string &val, const std::string& default_val) {

    if(m_pnh.param<std::string>(param, val, default_val) ) {
        std::cout << "val: " << val << std::endl;

        // find the templated values
        std::regex template_regex("\\{\\{(.*)\\}\\}");
        auto tparams_begin =
            std::sregex_iterator(val.begin(), val.end(), template_regex);
        auto tparams_end = std::sregex_iterator();

        for (std::sregex_iterator i = tparams_begin; i != tparams_end; ++i) {
            std::string m = i->str();

            m.erase(std::remove(m.begin(), m.end(), '{'), m.end());
            m.erase(std::remove(m.begin(), m.end(), '}'), m.end());
            m.erase(std::remove(m.begin(), m.end(), ' '), m.end());

            std::string p;
            if(!m_pnh.param(std::string("template_args/") + m, p, std::string())) {
                throw control_exception("template argument is not defined!");
            }

            size_t index = 0;
            index = val.find(i->str(), index);
            if (index == std::string::npos) break;

            val.replace(index, i->str().length(), p);

        }

        std::cout << val << std::endl;
        return true;
    }

    return false;

//    std::regex long_word_regex("(\\w{7,})");
//    std::string new_s = std::regex_replace(val, long_word_regex, "[$&]");


}