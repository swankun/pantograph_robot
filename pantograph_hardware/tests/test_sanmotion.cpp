#include <string>
#include <iostream>
#include <thread>
#include <pantograph_hardware/sanmotion.h>

using namespace std::literals::chrono_literals;


int main(int argc, char* argv[])
{
    sanmotion_ethercat::EthercatMaster master(0);

    if (master.is_ready()) {   
        master.run();
    } else {
        fprintf(stderr, "Failed to start EtherCAT Master.\n");
        return -1;
    }
    
    std::this_thread::sleep_for(1500ms);
    master.set_op_mode(0, sanmotion_ethercat::MODE_PV);
    master.set_op_mode(1, sanmotion_ethercat::MODE_PV);
    master.enable_drive(0);
    master.enable_drive(1);
    std::this_thread::sleep_for(1500ms);
    master.set_target_velocity(0, 1000);
    master.set_target_velocity(1, 1000);
    // master.stop();
    std::this_thread::sleep_for(1500ms);
    // master.run();
    master.set_target_velocity(0, 10);
    master.set_target_velocity(1, 10);
    std::this_thread::sleep_for(3000ms);

    return 0;
}
