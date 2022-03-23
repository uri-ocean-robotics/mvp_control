#include "mvp_control/mvp_control_ros.h"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "mvp_control");

    ctrl::MvpControlROS control_ros;

    control_ros.initialize();

    ros::spin();

    return 0;
}