#include <ros/ros.h>
#include <pantograph_hardware/pantograph_hw.h>


PantographHW::PantographHW()
{
    ros::V_string joint_names = boost::assign::list_of
        ("theta1")("theta4");

    ecat_master_ = std::make_unique<EthercatMaster>(0);
    for (unsigned int i=0; i<2; i++)
    {
        hardware_interface::JointStateHandle joint_state_handle(joint_names[i], 
                                                                &joint_[i].position, 
                                                                &joint_[i].velocity, 
                                                                &joint_[i].effort);
        joint_state_interface_.registerHandle(joint_state_handle);
        hardware_interface::JointHandle joint_handle(joint_state_handle, 
                                                     &joint_[i].command);
        velocity_joint_interface_.registerHandle(joint_handle);
        ecat_master_->set_op_mode(i, MODE_PV);
    }
    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);
    ecat_master_->run();
}


PantographHW::~PantographHW()
{
    ecat_master_->stop();
}


void PantographHW::read()
{
    for (unsigned int i=0; i<2; i++)
    {
        ecat_master_->get_actual_position(i, raw_joint_[i].position);
        ecat_master_->get_actual_velocity(i, raw_joint_[i].velocity);
        ecat_master_->get_actual_torque(i, raw_joint_[i].effort);
        joint_[i].position = raw_joint_[i].position /= encoder_ppr_;
        joint_[i].velocity = raw_joint_[i].velocity /= encoder_ppr_;
        joint_[i].effort = raw_joint_[i].effort *= torque_constant_;
    }
}


void PantographHW::write()
{
    for (unsigned int i=0; i<2; i++)
    {
        raw_joint_[i].command = static_cast<int32_t>(joint_[i].command * encoder_ppr_);
        ecat_master_->set_target_velocity(i, raw_joint_[i].command);
    }
}


void PantographHW::reset()
{
    for (unsigned int i=0; i<2; i++)
    {
        ecat_master_->set_target_velocity(i, 0);
        ecat_master_->clear_faults(i);
    }
}


void PantographHW::enable()
{
    for (unsigned int i=0; i<2; i++)
    {
        ecat_master_->enable_drive(i);
    }
}



void PantographHW::disable()
{
    for (unsigned int i=0; i<2; i++)
    {
        ecat_master_->disable_drive(i);
    }
}
