#include <ros/ros.h>
#include <pantograph_hardware/pantograph_hw.h>


PantographHW::PantographHW(ros::NodeHandle &nh) :
    encoder_ppr_(131072.0)
    , gear_ratio_(10.0)
    , torque_constant_(1.0)
{
    pulses_to_rad_ = encoder_ppr_*gear_ratio_/(2.0 * M_PI);
    
    ros::NodeHandle hwnh(nh, "pantograph_hardware");
    const unsigned int ecat_master_id(hwnh.param("ethercat_master_id", 0)); 
    ROS_INFO("[pantograph_hardware] EtherCAT master id: %u", ecat_master_id);
    const unsigned int ecat_cycle_period(hwnh.param("ethercat_cycle_period_ns", 5000000)); 
    ROS_INFO("[pantograph_hardware] EtherCAT cycle period: %u ns", ecat_cycle_period);

    ros::V_string joint_names = boost::assign::list_of
        ("theta1")("theta4");

    ecat_master_ = std::make_unique<EthercatMaster>(ecat_master_id, ecat_cycle_period);
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
    std::this_thread::sleep_for(1500ms);
    init(nh, hwnh);
}


PantographHW::~PantographHW()
{
    ecat_master_->stop();
}


bool PantographHW::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
{
    
    const double max_accel(robot_hw_nh.param("joint_max_accel", 50)); /* rad/s^2 */
    ROS_INFO("[pantograph_hardware] Max joint acceleration = %.1f rad/s^2", max_accel);
    const double max_decel(robot_hw_nh.param("joint_max_decel", 50)); /* rad/s^2 */
    ROS_INFO("[pantograph_hardware] Max joint deceleration = %.1f rad/s^2", max_decel);
    const bool do_home(robot_hw_nh.param("set_home_on_startup", true)); 
    ROS_INFO("[pantograph_hardware] Set home on startup: %s", do_home ? "true" : "false");
    const bool do_enable(robot_hw_nh.param("enable_on_startup", false)); 
    ROS_INFO("[pantograph_hardware] Enable motors on startup: %s", do_home ? "true" : "false");

    if (do_home) { home(); }

    for (unsigned int i=0; i<2; i++)
    {
        ecat_master_->set_profile_accel(i, static_cast<uint32_t>(max_accel*pulses_to_rad_));
        ecat_master_->set_profile_decel(i, static_cast<uint32_t>(max_decel*pulses_to_rad_));
    }

    if (do_enable) { enable(); }

    return true;
}

void PantographHW::read()
{
    for (unsigned int i=0; i<2; i++)
    {
        ecat_master_->get_actual_position(i, raw_joint_[i].position);
        ecat_master_->get_actual_velocity(i, raw_joint_[i].velocity);
        ecat_master_->get_actual_torque(i, raw_joint_[i].effort);
        joint_[i].position = (raw_joint_[i].position - home_pulses_[i]) / pulses_to_rad_;
        joint_[i].velocity = raw_joint_[i].velocity / pulses_to_rad_;
        joint_[i].effort = raw_joint_[i].effort * torque_constant_;
    }
}


void PantographHW::write()
{
    for (unsigned int i=0; i<2; i++)
    {
        raw_joint_[i].command = static_cast<int32_t>(joint_[i].command * pulses_to_rad_);
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


void PantographHW::home()
{
    disable();

    double old_home_pulses[2];
    std::copy(std::begin(home_pulses_), std::end(home_pulses_), std::begin(old_home_pulses));

    int counter = 0;
    while (counter < 5)
    {
        ROS_INFO("[pantograph_hardware] Waiting for home...");
        for (unsigned int i=0; i<2; i++)
        {
            read();
            home_pulses_[i] = raw_joint_[i].position;
        }
        if ((home_pulses_[0] != old_home_pulses[0]) && (home_pulses_[1] != old_home_pulses[1])) { 
            
            ROS_INFO("[pantograph_hardware] Current joint positions are set to zero.");
            break; 
        }
        std::this_thread::sleep_for(1s);
        counter++;
    }
    if (counter >= 5) 
    {
        ROS_WARN("[pantograph_hardware] Unable to set home position.");
    }
    
}
