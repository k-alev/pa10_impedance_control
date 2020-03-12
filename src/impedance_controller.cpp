#include <pa10_impedance_control/impedance_controller.h>


namespace pa10_impedance_control
{
ImpedanceController::ImpedanceController(void){};
ImpedanceController::~ImpedanceController(void)
{
    delete Mico;
    delete ctrl;
};

bool ImpedanceController::init(hardware_interface::RobotHW *robot, ros::NodeHandle &n)
{

    hardware_interface::EffortJointInterface *robot_ = robot->get<hardware_interface::EffortJointInterface>();
    // hardware_interface::ForceTorqueSensorInterface *ft_sensor = robot->get<hardware_interface::ForceTorqueSensorInterface>();
    // hardware_interface::ArrayTorqueSensorsInterface *arr_trq_sensors = robot->get<hardware_interface::ArrayTorqueSensorsInterface>();
    node_ = n;
    //robot_=robot;

    XmlRpc::XmlRpcValue joint_names;
    if (!node_.getParam("joints", joint_names))
    {
        ROS_ERROR("No 'joints' in controller. (namespace: %s)",
                  node_.getNamespace().c_str());
        return false;
    }

    if (joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_ERROR("'joints' is not a struct. (namespace: %s)",
                  node_.getNamespace().c_str());
        return false;
    }

    for (int i = 0; i < joint_names.size(); i++)
    {
        XmlRpc::XmlRpcValue &name_value = joint_names[i];
        if (name_value.getType() != XmlRpc::XmlRpcValue::TypeString)
        {
            ROS_ERROR("joints are not strings. (namespace: %s)",
                      node_.getNamespace().c_str());
            return false;
        }

        hardware_interface::JointHandle j = robot_->getHandle((std::string)name_value);
        joints_.push_back(j);
    }

    // ft_ = ft_sensor->getHandle("ft_sensor");
    // trq_arr_ = arr_trq_sensors->getHandle("arr_trq_sensors");

    std::string robot_desc_string;
    if (!node_.getParam("/robot_description", robot_desc_string))
    {
        ROS_ERROR("Could not find '/robot_description'.");
        return false;
    }

    //YAML
    std::cout << "initializing robot..." << std::endl;
    std::string root = "world";
    std::string ee = "link_wrist_3"; //"link_center";
    Kd.setIdentity();
    Dd.setIdentity();
    Kd.topLeftCorner(3,3) = 600*Kd.topLeftCorner(3,3); //(20 7 5 2)
    Kd.bottomRightCorner(3,3) = 210*Kd.bottomRightCorner(3,3);
    Dd.topLeftCorner(3,3) = 800*Dd.topLeftCorner(3,3);
    Dd.bottomRightCorner(3,3) = 500*Dd.bottomRightCorner(3,3);   
    torque.resize(7,1);
    Mico = new rct::robot(robot_desc_string, root, ee, -9.81);
    ctrl = new rct_dev::ImpedanceCrtl(Md, Kd, Dd);
    // ...

    return true;
}

void ImpedanceController::starting(const ros::Time &time)
{
    ROS_INFO("Starting Controller");
    Mico->read_sensor_handles(joints_);
    ref = Mico->get_status("ee");

    // Cartesian Space Reference
    ref.frame.pos(0) = ref.frame.pos(0) +.2; //+.2;
    ref.frame.pos(1) = ref.frame.pos(1) -.3; //-.3;5
    ref.frame.pos(2) = ref.frame.pos(2) -.1; //-.1;2

    //Joint Space Reference
    // ref.q_conf(0) = ref.q_conf(0) - 1.6;
    // ref.q_conf(1) = ref.q_conf(1) - 0.5;
    // ref.q_conf(5) = ref.q_conf(5) + 0.6;

}

void ImpedanceController::update(const ros::Time &time, const ros::Duration &duration)
{
    // ROS_INFO("Update Cartesian Controller");
    // // // Mico->read_sensors<hardware_interface::ForceTorqueSensorHandle, std::vector<hardware_interface::JointHandle> >(ft_, joints_);
    // Mico->read_sensor_handles(joints_);
    // cur_status = Mico->get_status("ee"); // with options, phaps update status
    
    // ctrl->update(cur_status, ref, cmd_acc_ee);//command is accelerations perhaps in ee_frame
    // // // Mico->acc2base(cmd_acc_ee, cmd_acc); //needed only if cmd acc is in ee_frame
    // // Mico->get_inv_dynamics_cmd(cmd_acc_ee, torque); // torque is only for printing
    // std::string opt = "cartesian";
    // Mico->get_inv_dynamics_cmd(cmd_acc_ee, opt, 0.001);//should implement pinv
    // Mico->send_commands<std::vector<hardware_interface::JointHandle>>(joints_);

    ////////////////////////////////////////////////////////////////////////////////

    // ROS_INFO("Update Joint Space Controller");
    // Mico->read_sensor_handles(joints_);
    // cur_status = Mico->get_status("ee"); // with options, phaps update status
    // Eigen::MatrixXd conf_acc_cmd;
    // ctrl->update(cur_status, ref, conf_acc_cmd); //command is accelerations perhaps in ee_frame
    // std::string opt = "conf";
    // Mico->get_inv_dynamics_cmd(conf_acc_cmd, opt); //should implement pinv
    // Mico->send_commands<std::vector<hardware_interface::JointHandle>>(joints_);

    /////////////////////////////////////////////////////////////////////////////////

    ROS_INFO("Update Cartesian EE Controller");
    // // Mico->read_sensors<hardware_interface::ForceTorqueSensorHandle, std::vector<hardware_interface::JointHandle> >(ft_, joints_);
    Mico->read_sensor_handles(joints_);
    cur_status = Mico->get_status("ee"); // with options, phaps update status
    ctrl->update(cur_status, ref, cmd_acc_ee);//command is accelerations perhaps in ee_frame
    cmd_acc = Mico->acc2base(cmd_acc_ee); //needed only if cmd acc is in ee_frame
    // Mico->get_inv_dynamics_cmd(cmd_acc_ee, torque); // torque is only for printing
    Mico->get_inv_dynamics_cmd(cmd_acc, 0.01);
    Mico->send_commands<std::vector<hardware_interface::JointHandle>>(joints_);

}

void ImpedanceController::stopping(const ros::Time &time)
{
    ROS_INFO("Stopping Controller");
}

void ImpedanceController::print()
{
    ROS_INFO("Printing");
    for (unsigned int i = 0; i < 6; i++)
    {
        std::cout<<"Cart_cmd["<<i<<"]: "<<cmd_acc_ee(i)<<std::endl;
    }
    for (unsigned int i = 0; i < 7; i++)
    {
        std::cout<<"Torque["<<i<<"]: "<<torque(i)<<std::endl;
    }
}

}

PLUGINLIB_EXPORT_CLASS(pa10_impedance_control::ImpedanceController, controller_interface::ControllerBase)
