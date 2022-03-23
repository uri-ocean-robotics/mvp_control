#include "thruster_ros.h"
#include "exception.hpp"
#include "mvp_control/dictionary.h"

using namespace ctrl;

ThrusterROS::ThrusterROS() {
    m_poly_solver = boost::make_shared<PolynomialSolver>();
}

ThrusterROS::ThrusterROS(std::string id, std::string topic_id, Eigen::VectorXd contribution_vector) :
        m_id(std::move(id)),
        m_thrust_command_topic_id(std::move(topic_id)),
        m_contribution_vector(std::move(contribution_vector))
{

    m_thrust_publisher = m_nh.advertise<std_msgs::Float64>(m_thrust_command_topic_id, 10);

    m_poly_solver = boost::make_shared<PolynomialSolver>();
}

auto ThrusterROS::get_thrust_command_topic_id() -> decltype(m_thrust_command_topic_id) {
    return m_thrust_command_topic_id;
}

void ThrusterROS::set_thrust_command_topic_id(const decltype(m_thrust_command_topic_id) &topic_id) {
    m_thrust_command_topic_id = topic_id;
}

auto ThrusterROS::get_thrust_force_topic_id() -> decltype(this->m_thrust_force_topic_id) {
    return m_thrust_force_topic_id;
}

void ThrusterROS::set_thrust_force_topic_id(const decltype(m_thrust_force_topic_id) &topic_id) {
    m_thrust_force_topic_id = topic_id;
}

auto ThrusterROS::get_id() -> decltype(m_id) {
    return m_id;
}

void ThrusterROS::set_id(const decltype(m_id)& thruster_id) {
    m_id = thruster_id;
}

auto ThrusterROS::get_contribution_vector() -> decltype(m_contribution_vector) {
    return m_contribution_vector;
}

void ThrusterROS::set_contribution_vector(const decltype(m_contribution_vector)& contribution_vector) {
    m_contribution_vector = contribution_vector;
}

void ThrusterROS::initialize() {

    if(!m_thrust_command_topic_id.empty()) {
        m_thrust_publisher = m_nh.advertise<std_msgs::Float64>(m_thrust_command_topic_id, 100);
    } else {
        throw control_ros_exception("empty topic name");
    }

    if(!m_thrust_command_topic_id.empty()) {
         m_force_publisher = m_nh.advertise<std_msgs::Float64>(m_thrust_force_topic_id, 100);
    } else {
        throw control_ros_exception("empty topic name");
    }
}

auto ThrusterROS::get_link_id() -> decltype(m_link_id) {
    return m_link_id;
}

void ThrusterROS::set_link_id(const decltype(m_link_id)& link_id) {
    m_link_id = link_id;
}

void ThrusterROS::command(float cmd) {
    std_msgs::Float64 msg;
    msg.data = cmd;
    m_thrust_publisher.publish(msg);
}

auto ThrusterROS::get_poly_solver() -> decltype(m_poly_solver) {
    return m_poly_solver;
}

void ThrusterROS::set_poly_solver(decltype(m_poly_solver) solver) {
    m_poly_solver = solver;
}

bool ThrusterROS::request_force(double N) {
    std::vector<std::complex<double>> roots;


    if(fabs(N) > THRUST_LIMIT_NEWTON) {
        if(signbit(N) != 0) {
            N = -THRUST_LIMIT_NEWTON;
        } else {
            N = THRUST_LIMIT_NEWTON;
        }
    }

    std_msgs::Float64  msg;
    msg.data = N;
    m_force_publisher.publish(msg);

    if(!m_poly_solver->solve_for_y(roots, N)) {
        ROS_WARN_STREAM("No feasible command found for force: " << N);
        return false;
    }

    for(const auto& r : roots) {
        if(r.imag() != 0){
            continue;
        }

        if(r.real() >= 1 || r.real() < -1) {
            continue;
        }

        command(r.real());

        break;
    }

    return true;
}