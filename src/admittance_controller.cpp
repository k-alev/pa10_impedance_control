#include <pa10_impedance_control/admittance_controller.h>

namespace pa10_impedance_control
{
AdmittanceController::AdmittanceController(void){};
AdmittanceController::~AdmittanceController(void)
{
    delete pa10;
    delete ctrl;
};

bool AdmittanceController::init(hardware_interface::RobotHW *robot, ros::NodeHandle &n)
{

    hardware_interface::VelocityJointInterface *robot_ = robot->get<hardware_interface::VelocityJointInterface>();
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

    // /////////////////////////////////////////////////
    std::string tmp_desc_string;
    std::vector<std::string> env_desc_strings;
    if (!node_.getParam("/box_description", tmp_desc_string))
    {
        ROS_ERROR("Could not find '/box_description'.");
        return false;
    }
    env_desc_strings.push_back(tmp_desc_string);
    if (!node_.getParam("/box2_description", tmp_desc_string))
    {
        ROS_ERROR("Could not find '/box_description'.");
        return false;
    }
    env_desc_strings.push_back(tmp_desc_string);
    if (!node_.getParam("/box3_description", tmp_desc_string))
    {
        ROS_ERROR("Could not find '/box_description'.");
        return false;
    }
    env_desc_strings.push_back(tmp_desc_string);
    
    // ////////////////////////////////////////////////

    std::vector<double> K_gain;
    if (!node_.getParam("k_gain", K_gain))
    {
        ROS_ERROR("No 'k_gain' in controller. (namespace: %s)",
                  node_.getNamespace().c_str());
        return false;
    }

    std::vector<double> D_gain;
    if (!node_.getParam("d_gain", D_gain))
    {
        ROS_ERROR("No 'd_gain' in controller. (namespace: %s)",
                  node_.getNamespace().c_str());
        return false;
    }

    std::vector<std::string> chain_lims;
    if (!node_.getParam("chain", chain_lims))
    {
        ROS_ERROR("No 'chain' in controller. (namespace: %s)",
                  node_.getNamespace().c_str());
        return false;
    }

    //YAML
    // std::cout << "initializing robot..." << std::endl;
    // std::string root = "world";
    // std::string ee = "link_wrist_3"; //"link_center";

    Kd.setIdentity();
    Dd.setIdentity();
    Kd.topLeftCorner(3, 3) = K_gain[0] * Kd.topLeftCorner(3, 3); //(20 7 5 2)
    Kd.bottomRightCorner(3, 3) = K_gain[1] * Kd.bottomRightCorner(3, 3);
    Dd.topLeftCorner(3, 3) = D_gain[0] * Dd.topLeftCorner(3, 3);
    Dd.bottomRightCorner(3, 3) = D_gain[1] * Dd.bottomRightCorner(3, 3);
    torque.resize(7, 1);
    pa10 = new rct::robot(robot_desc_string, chain_lims[0], chain_lims[1], -9.81, true);
    ctrl = new rct_dev::ImpedanceCrtl(Md, Kd, Dd);
    // ...

    managers.push_back(new fcl::NaiveCollisionManagerVct()); // for robot
    managers.push_back(new fcl::NaiveCollisionManagerVct()); // for envirinment
    rbtObjs = urdf2fcl::getRobotCollisionObjects(robot_desc_string, pa10->get_kdl_chain());
    managers[0]->registerVct(rbtObjs.first);
    envObjs = urdf2fcl::getEnvCollisionObjects(env_desc_strings);
    managers[1]->registerVct(envObjs.first);
    visMark = new VisualMarkerPub(node_);

    return true;
}

void AdmittanceController::starting(const ros::Time &time)
{
    ROS_INFO("Starting Controller");
    cmd_vel_ee.setZero(6, 1);
    pa10->read_sensor_handles(joints_);
    ref = pa10->get_status("ee");

    // Cartesian Space Reference
    ref.frame.pos(0) = ref.frame.pos(0) + .3; //+.2;
    ref.frame.pos(1) = ref.frame.pos(1) - .4; //-.3;5
    ref.frame.pos(2) = ref.frame.pos(2) - .2; //-.1;2

    //Joint Space Reference
    // ref.q_conf(0) = ref.q_conf(0) - 1.6;
    // ref.q_conf(1) = ref.q_conf(1) - 0.5;
    // ref.q_conf(5) = ref.q_conf(5) + 0.6;
}

void AdmittanceController::update(const ros::Time &time, const ros::Duration &duration)
{
    // ROS_INFO("Update Cartesian Controller");
    // // // Pa10->read_sensors<hardware_interface::ForceTorqueSensorHandle, std::vector<hardware_interface::JointHandle> >(ft_, joints_);
    // Pa10->read_sensor_handles(joints_);
    // cur_status = Pa10->get_status("ee"); // with options, phaps update status

    // ctrl->update(cur_status, ref, cmd_acc_ee);//command is accelerations perhaps in ee_frame
    // // // Pa10->acc2base(cmd_acc_ee, cmd_acc); //needed only if cmd acc is in ee_frame
    // // Pa10->get_inv_dynamics_cmd(cmd_acc_ee, torque); // torque is only for printing
    // std::string opt = "cartesian";
    // Pa10->get_inv_dynamics_cmd(cmd_acc_ee, opt, 0.001);//should implement pinv
    // Pa10->send_commands<std::vector<hardware_interface::JointHandle>>(joints_);

    ////////////////////////////////////////////////////////////////////////////////

    // ROS_INFO("Update Joint Space Controller");
    // Pa10->read_sensor_handles(joints_);
    // cur_status = Pa10->get_status("ee"); // with options, phaps update status
    // Eigen::MatrixXd conf_acc_cmd;
    // ctrl->update(cur_status, ref, conf_acc_cmd); //command is accelerations perhaps in ee_frame
    // std::string opt = "conf";
    // Pa10->get_inv_dynamics_cmd(conf_acc_cmd, opt); //should implement pinv
    // Pa10->send_commands<std::vector<hardware_interface::JointHandle>>(joints_);

    /////////////////////////////////////////////////////////////////////////////////

    ROS_INFO("Update Cartesian EE Controller");
    // // Pa10->read_sensors<hardware_interface::ForceTorqueSensorHandle, std::vector<hardware_interface::JointHandle> >(ft_, joints_);
    pa10->read_sensor_handles(joints_);
    cur_status = pa10->get_status("ee");       // with options, phaps update status
    ctrl->update(cur_status, ref, cmd_acc_ee); //command is accelerations perhaps in ee_frame
    ctrl->integrate(duration, cmd_acc_ee, cmd_vel_ee);
    cmd_vel = pa10->vel2base(cmd_vel_ee); //needed only if cmd acc is in ee_frame
    pa10->get_joint_vel_cmd(cmd_vel, 0.01);
    pa10->send_commands<std::vector<hardware_interface::JointHandle>>(joints_);

    /////////////////////////////////////////////////////////////////

    stata = pa10->get_links_status();
    std::vector<rct::Status> novaStata = managers[0]->removeDuplicates(stata);
    managers[0]->updateVct(novaStata, rbtObjs.second);
    std::vector<std::vector<fcl::DistanceData>> datum;
    managers[0]->distanceVct_Plus(managers[1], datum, 300, fcl::defaultDistanceFunction);

    std::cout << "size: " << datum.size() << std::endl;
    for (int i = 0; i < datum.size(); ++i)
    {
        for (std::vector<fcl::DistanceData>::iterator it = datum[i].begin(); it != datum[i].end(); ++it)
        {
            std::cout << "other_mngr" << i << ": " << (it)->result.min_distance <<" == "<<(it)->distVctWF.first.length() << std::endl;
            // std::cout << "other_mngr_1pts" << i << ": " << (it)->result.nearest_points[0] << std::endl;
            // std::cout << "other_mngr_2pts" << i << ": " << (it)->result.nearest_points[1] << std::endl;
            std::cout << "first vector" << i << ": " << (it)->distVctWF.second << std::endl;
        }
    }
    visMark->visualMarkerBroadcaster(datum, "/world");

    ////////////////////////////////////////////////////////////
}

void AdmittanceController::stopping(const ros::Time &time)
{
    ROS_INFO("Stopping Controller");
}

void AdmittanceController::print()
{
    ROS_INFO("Printing");
    for (unsigned int i = 0; i < 6; i++)
    {
        std::cout << "Cart_cmd[" << i << "]: " << cmd_vel_ee(i) << std::endl;
    }
    for (unsigned int i = 0; i < 7; i++)
    {
        std::cout << "Torque[" << i << "]: " << torque(i) << std::endl;
    }
}

} // namespace pa10_impedance_control

PLUGINLIB_EXPORT_CLASS(pa10_impedance_control::AdmittanceController, controller_interface::ControllerBase)
