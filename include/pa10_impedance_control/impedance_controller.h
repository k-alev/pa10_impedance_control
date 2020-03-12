#pragma once

#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <controller_interface/multi_interface_controller.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <pluginlib/class_list_macros.h>

#include <Eigen/Dense>

#include <pa10_impedance_control/array_torque_sensors.h>
#include <pa10_impedance_control/robot_impl.h>
#include <pa10_impedance_control/controller.h>

#include <ros_control_toolbox/robot.h>
#include <ros_control_toolbox/status.h>

namespace pa10_impedance_control
{

class ImpedanceController : public controller_interface::
                                MultiInterfaceController<hardware_interface::EffortJointInterface>
{
public:
    ImpedanceController(void);
    ~ImpedanceController(void);

    bool init(hardware_interface::RobotHW *robot, ros::NodeHandle &n);
    void starting(const ros::Time &time);
    void update(const ros::Time &time, const ros::Duration &duration);
    void stopping(const ros::Time &time);

    void print();

private:
    std::vector<hardware_interface::JointHandle> joints_;
    // hardware_interface::ForceTorqueSensorHandle ft_;
    // hardware_interface::ArrayTorqueSensorsHandle trq_arr_;
    //hardware_interface::EffortJointInterface *robot_;
    ros::NodeHandle node_;

    rct::robot *Mico;
    rct_dev::ImpedanceCrtl *ctrl;
    rct::Status cur_status = rct::Status();
    rct::Status ref = rct::Status();

    Eigen::VectorXd cmd_acc;
    // cmd_acc.resize(6);
    Eigen::MatrixXd cmd_acc_ee;
    Eigen::MatrixXd torque;
    Eigen::Matrix<double, 6, 6> Md;
    Eigen::Matrix<double, 6, 6> Kd;
    Eigen::Matrix<double, 6, 6> Dd;
};

} // namespace RCTcontrol
