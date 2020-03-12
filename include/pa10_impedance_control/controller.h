#pragma once
#include <ros_control_toolbox/status.h>
#include <ros_control_toolbox/utils.h>

namespace rct_dev
{
class CrtlBase
{
public:
    CrtlBase()
    {
        std::cout << " Controller base initialized" << std::endl;
    };
    ~CrtlBase()
    {
        std::cout << " Controller base dtor" << std::endl;
    };

    virtual void update(const rct::Status &state,
                        const rct::Status &ref,
                        Eigen::MatrixXd &cmd) = 0;

    virtual void start() = 0;
    virtual void stop() = 0;
    // virtual void acc2vel() = 0;
    // virtual void acc2pos() = 0;
};

class ImpedanceCrtl : public CrtlBase
{
public:
    ImpedanceCrtl(Eigen::Matrix<double, 6, 6> _M, Eigen::Matrix<double, 6, 6> _K, Eigen::Matrix<double, 6, 6> _D)
        : M{_M}, K{_K}, D{_D}
    {
        Ktilde = K;
        std::cout << "Impedance controller initialized" << std::endl;
    };

    void update(const rct::Status &state, const rct::Status &ref, Eigen::MatrixXd &cmd)
    {

        // // perhaps should add a cmd member in rct::Status (instead of Eigen::Matrix<double, 6, 1> cmd)
        // // in this way ppc-like controller could return a cmd_status object
        
        // cartesian_imp_ctrl(state, ref, cmd);
        // joint_space_imp_ctrl(state, ref, cmd);
        cartesianEE_imp_ctrl(state, ref, cmd);
    };

    void start()
    {
        std::cout << "Start" << std::endl;
    };

    void stop()
    {
        std::cout << "Stop" << std::endl;
    };

    void cartesianEE_imp_ctrl(const rct::Status &state, const rct::Status &ref, Eigen::MatrixXd &cmd)
    {
        Ktilde.bottomRightCorner(3,3) = 2*rct::E(state.quat).transpose()*K.bottomRightCorner(3,3);

        e = ref.frame.pos - state.frame.pos;
        e.segment<3>(0) = state.R.transpose()*e.segment<3>(0);
        e.segment<3>(3) = state.R.transpose()*e.segment<3>(3);
        de = ref.frame.vel - state.frame.vel;
        de.segment<3>(0) = state.R.transpose()*de.segment<3>(0);
        de.segment<3>(3) = state.R.transpose()*de.segment<3>(3);

        std::cout<<"ERROR"<<std::endl;
        std::cout<<e<<std::endl;
        cmd = Ktilde * e + D*de;
    }

    void joint_space_imp_ctrl(const rct::Status &state, const rct::Status &ref, Eigen::MatrixXd &cmd)
    {
        Eigen::Matrix<double, 7, 1> econf;
        econf = ref.q_conf - state.q_conf;

        Eigen::Matrix<double, 7, 7> Kconf;
        Kconf.setIdentity();
        Kconf = 10 * Kconf;
        std::cout<<econf<<std::endl;
        cmd = (Kconf * econf);
    }

    void cartesian_imp_ctrl(const rct::Status &state, const rct::Status &ref, Eigen::MatrixXd &cmd)
    {
        e = ref.frame.pos - state.frame.pos;
        std::cout<<e<<std::endl;
        de = ref.frame.vel - state.frame.vel;
        cmd = K * e;
        // cmd = (K * e + D * de);
    }

private:
    Eigen::Matrix<double, 6, 6> M;
    Eigen::Matrix<double, 6, 6> K;
    Eigen::Matrix<double, 6, 6> Ktilde;
    Eigen::Matrix<double, 6, 6> D;
    Eigen::Matrix<double, 6, 1> e;
    Eigen::Matrix<double, 6, 1> de;
};

} // namespace rct_dev