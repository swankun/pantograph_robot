#ifndef SANMOTION_ETHERCAT_SANMOTION_H
#define SANMOTION_ETHERCAT_SANMOTION_H


#define PERIOD_NS   (5000000)
#define MAX_SAFE_STACK (8 * 1024) /* The maximum stack size which is guranteed safe to access without faulting */
#define NSEC_PER_SEC (1000000000)
#define FREQUENCY (NSEC_PER_SEC / PERIOD_NS)
#define SANMOTION_ID 0x000001b9, 0x00000006

#include <errno.h>
#include <mutex>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h> /* mlockall() */
#include <sched.h> /* sched_setscheduler() */
#include <time.h>
#include <thread>
#include <vector>

#include <ecrt.h>

namespace sanmotion_ethercat
{

using namespace std::literals::chrono_literals;

typedef enum
{
    MODE_NOOP = 0,
    MODE_PP   = 1,
    MODE_PV   = 3,
    MODE_PT   = 4,
    MODE_HM   = 6,
    MODE_IP   = 7,
    MODE_CSP  = 8,
    MODE_CSV  = 9,
    MODE_CST  = 10,
} op_mode_t;

static const char* to_string(op_mode_t mode)
{
    switch (mode)
    {
        case MODE_NOOP:
            return "Mode not assigned";
            break;
        case MODE_PP:
            return "Profile position";
            break;
        case MODE_PV:
            return "Profile velocity";
            break;
        case MODE_PT:
            return "Profile torque";
            break;
        case MODE_HM:
            return "Homing";
            break;
        case MODE_IP:
            return "Interpolated position";
            break;
        case MODE_CSP:
            return "Cyclic synchronous position";
            break;
        case MODE_CSV:
            return "Cyclic synchronous velocity";
            break;
        case MODE_CST:
            return "Cyclic synchronous torque";
            break;
        default:
            return "Invalid operation mode";
            break;
    }
}

typedef enum
{
    NOT_READY_TO_SWITCH_ON_A = 0b0000000,
    NOT_READY_TO_SWITCH_ON_B = 0b0100000,
    SWITCH_ON_DISABLED_A     = 0b1000000,
    SWITCH_ON_DISABLED_B     = 0b1100000,
    READY_TO_SWITCH_ON       = 0b0100001,
    SWITCH_ON                = 0b0100011,
    OPERATION_ENABLED        = 0b0100111,
    QUICK_STOP_ACTIVE        = 0b0000111,
    FAULT_REACTION_ACTIVE_A  = 0b0001111,
    FAULT_REACTION_ACTIVE_B  = 0b0101111,
    FAULT_A                  = 0b0001000,
    FAULT_B                  = 0b0101000,
} fsa_state_t;


static fsa_state_t fsa_from(uint16_t status_word)
{
    status_word &= (1UL << 7) - 1; // get last 7 bits
    status_word &= ~(1UL << 4); // clear bit4
    return static_cast<fsa_state_t>(status_word);
}


static const char* to_string(fsa_state_t state)
{
    switch (state)
    {
        case NOT_READY_TO_SWITCH_ON_A:
            return "NOT_READY_TO_SWITCH_ON";
            break;
        case NOT_READY_TO_SWITCH_ON_B:
            return "NOT_READY_TO_SWITCH_ON";
            break;
        case SWITCH_ON_DISABLED_A:
            return "SWITCH_ON_DISABLED";
            break;
        case SWITCH_ON_DISABLED_B:
            return "SWITCH_ON_DISABLED";
            break;
        case READY_TO_SWITCH_ON:
            return "READY_TO_SWITCH_ON";
            break;
        case SWITCH_ON:
            return "SWITCH_ON";
            break;
        case OPERATION_ENABLED:
            return "OPERATION_ENABLED";
            break;
        case QUICK_STOP_ACTIVE:
            return "QUICK_STOP_ACTIVE";
            break;
        case FAULT_REACTION_ACTIVE_A:
            return "FAULT_REACTION_ACTIVE";
            break;
        case FAULT_REACTION_ACTIVE_B:
            return "FAULT_REACTION_ACTIVE";
            break;
        case FAULT_A:
            return "FAULT";
            break;
        case FAULT_B:
            return "FAULT";
            break;
        default:
            return "Invalid Status Word";
            break;
    }
}


class EthercatMaster
{

public:
    EthercatMaster(const size_t master_index, const unsigned int period_ns=PERIOD_NS);
    ~EthercatMaster();
    
    bool is_ready();
    void run();
    void stop();
    bool is_running();

    // Setters
    void set_op_mode(const unsigned int slave_id, const op_mode_t mode);
    void set_target_position(const unsigned int slave_id, const int32_t target);
    void set_profile_velocity(const unsigned int slave_id, const uint32_t target);
    void set_profile_accel(const unsigned int slave_id, const uint32_t target);
    void set_profile_decel(const unsigned int slave_id, const uint32_t target);
    void set_target_velocity(const unsigned int slave_id, const int32_t target);
    void set_target_torque(const unsigned int slave_id, const int16_t target);
    void enable_drive(const unsigned int slave_id);
    void disable_drive(const unsigned int slave_id);
    void clear_faults(const unsigned int slave_id);

    // Getters
    void get_actual_position(const unsigned int slave_id, int32_t &value);
    void get_actual_velocity(const unsigned int slave_id, int32_t &value);
    void get_actual_torque(const unsigned int slave_id, int16_t &value);
    void get_op_mode_display(const unsigned int slave_id, op_mode_t &value);
    void get_fsa_state(const unsigned int slave_id, fsa_state_t &value);

private:
    void configure_slaves();
    void register_pdo_entry();
    void check_domain_state();
    void check_master_state();
    void check_slave_config_states();
    void stack_prefault();
    void read_data();
    void write_data();
    void displayall();
    void cyclic_task();

    void set_control_word_shutdown(const unsigned int slave_id);
    void set_control_word_switchon_enable(const unsigned int slave_id);
    void set_control_word_clear_fault(const unsigned int slave_id);

    bool running_ = false;

    size_t retcode_ = 0;

    ec_master_t *master_ptr_ = NULL;
    ec_domain_t *domain_ptr_ = NULL;
    uint8_t *domain_pd_ = NULL;

    unsigned int period_ns_, frequency_;
    unsigned int task_counter_ = 0;
    std::thread cyclic_worker_;

    ec_slave_config_t *slave_config_[2];
    struct
    {
        unsigned int control_word;
        unsigned int op_mode;
        unsigned int target_position;
        unsigned int profile_velocity;
        unsigned int profile_accel;
        unsigned int profile_decel;
        unsigned int target_velocity;
        unsigned int target_torque;
        unsigned int status_word;
        unsigned int actual_position;
        unsigned int actual_velocity;
        unsigned int actual_torque;
        unsigned int op_mode_display;
    } slave_offset_[2];

    struct
    {
        uint16_t    control_word = ~(~0u << 2) << 1; /* 0001_1111 */
        op_mode_t   op_mode = MODE_NOOP;
        int32_t     target_position = 0;
        uint32_t    profile_velocity = 0xFFFFFFFF;
        uint32_t    profile_accel = 0xFFFFFFFF;
        uint32_t    profile_decel = 0xFFFFFFFF;
        int32_t     target_velocity = 0;
        int16_t     target_torque = 0;
        uint16_t    status_word = 0;
        int32_t     actual_position;
        int32_t     actual_velocity = 0;
        int16_t     actual_torque = 0;
        op_mode_t   op_mode_display;
        fsa_state_t fsa_state;
        std::mutex  mutex;
    } slave_data_[2];



}; // class EthercatMaster

} // namespace sanmotion_ethercat

#endif 
