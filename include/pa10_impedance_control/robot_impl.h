#include <ros_control_toolbox/robot.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>

template <class T>
void rct::robot::read_from_robot(const T &handle)
{
  std::cout << "reading from robot unspecialized" << std::endl;
}

namespace rct
{
template <>
void robot::read_from_robot<hardware_interface::ForceTorqueSensorHandle>(const hardware_interface::ForceTorqueSensorHandle &handle)
{
  const double *tmp_frc, *tmp_trq;

  tmp_frc = handle.getForce();
  tmp_trq = handle.getTorque();

  //ft should be expressed in base frame

  // for (unsigned int i = 0; i < 3; i++)
  // {
  //   ft.force(i) = tmp_frc[i];
  //   ft.torque(i) = tmp_trq[i];
  // }

  //Kinova Mico returns ft in different frames
  for (unsigned int i = 0; i < 3; i++)
  {
    ft_base.force(i) = -tmp_frc[i];
    ft_base.torque(i) = -tmp_trq[i];
  }
  ft_base.torque = state_frame.M.R * ft_base.torque;

  std::cout << "Reading F/T Handle " << std::endl;
}
} // namespace rct

namespace rct
{
template <>
void rct::robot::read_from_robot<std::vector<hardware_interface::JointHandle>>(const std::vector<hardware_interface::JointHandle> &joints_)
{
  //get position, velocity and torque measurements
  for (unsigned int i = 0; i < joints_.size(); i++)
  {
    q(i) = joints_[i].getPosition();
    dq(i) = joints_[i].getVelocity();
    // jnt_trq(i)=-joints_[i].getEffort();
  }
  // std::cout << "Reading Joint Handle " << std::endl;
}
} // namespace rct

template <class T>
void rct::robot::write_to_robot(T &handle)
{
  std::cout << "writing to robot unspecialized" << std::endl;
}

namespace rct
{
template <>
void robot::write_to_robot<std::vector<hardware_interface::JointHandle>>(std::vector<hardware_interface::JointHandle> &joints_)
{
  std::cout<<"Command: "<<std::endl;
  for (unsigned int i = 0; i < joints_.size(); i++)
  {
    joints_[i].setCommand(torque(i));
    std::cout<<torque(i)<<std::endl;
  }
    // joints_[1].setCommand(-4.5);
    // joints_[6].setCommand(10.0);
}

} // namespace rct