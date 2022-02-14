#ifndef PANTOGRAPH_HARDWARE_INTERFACE_H
#define PANTOGRAPH_HARDWARE_INTERFACE_H

#include <boost/assign.hpp>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <urdf/model.h>

#include <pantograph_hardware/sanmotion.h>

using namespace sanmotion_ethercat;
using namespace std::literals::chrono_literals;

class PantographHW : public hardware_interface::RobotHW 
{
    public:
        PantographHW();
        ~PantographHW();
        void read();
        void write();
        void reset();
        void enable();
        void disable();

    protected:

        ros::NodeHandle nh_;

        // Interfaces
        hardware_interface::JointStateInterface      joint_state_interface_;
        hardware_interface::VelocityJointInterface   velocity_joint_interface_;

        // These are mutated on the controls thread only.
        struct Joint
        {
            double position;
            double velocity;
            double effort;
            double command;

            Joint() : position(0), velocity(0), effort(0), command(0)
            {
            }
        }
        joint_[2];

        struct RawJoint
        {
            int32_t position;
            int32_t velocity;
            int16_t effort;
            int32_t command;

            RawJoint() : position(0), velocity(0), effort(0), command(0)
            {
            }
        }
        raw_joint_[2];

        double encoder_ppr_ = 1.0, torque_constant_ = 1.0;

    private:
        std::unique_ptr<EthercatMaster> ecat_master_;
        

}; // class

#endif