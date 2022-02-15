#include <pantograph_hardware/sanmotion.h>

namespace sanmotion_ethercat
{

EthercatMaster::EthercatMaster(const size_t master_id, const unsigned int period_ns)
{
    period_ns_ = period_ns;
    frequency_ = NSEC_PER_SEC / period_ns_;
    
    /* Master configurations */
    master_ptr_ = ecrt_request_master(master_id);
    if (!master_ptr_) { 
        retcode_++;
    }
    /* Domain configurations */
    domain_ptr_ = ecrt_master_create_domain(master_ptr_);
    if (!domain_ptr_) {
        retcode_++;
    }

    /* Slave configurations */
    configure_slaves();
    /* Register PDO on domain */
    register_pdo_entry();

    // Activate master
    if (ecrt_master_activate(master_ptr_)) {
        fprintf(stderr, "Failed to activate EtherCAT Master.\n");
    }
    if (!(domain_pd_ = ecrt_domain_data(domain_ptr_))) {
        fprintf(stderr, "Failed to obtain domain process data.\n");
    }
    ec_master_info_t master_info;
    if(ecrt_master(master_ptr_, &master_info))
    {
        fprintf(stderr, "Failed to obtain master info after activating.\n");
    }

    // /* Set priority */
    // struct sched_param param = {};
    // param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    // printf("Attempting to set priority %i...\n", param.sched_priority);
    // if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
    //     perror("sched_setscheduler failed");
    // }

    /* Lock memory */
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        fprintf(stderr, "Warning: Failed to lock memory: %s\n",
                strerror(errno));
    }
    stack_prefault();
}


EthercatMaster::~EthercatMaster()
{
    stop();
    munlockall();
    ecrt_release_master(master_ptr_);
    printf("EtherCAT master is released.\n");
}


void EthercatMaster::stop()
{
    for (int slave_id=0; slave_id<1; slave_id++) {
        set_target_position(slave_id, slave_data_[slave_id].actual_position);
        set_target_velocity(slave_id, 0);
        set_target_torque(slave_id, 0);
        disable_drive(slave_id);
    }
    running_ = false;
    std::this_thread::sleep_for(10ms);
    if (cyclic_worker_.joinable()) { cyclic_worker_.join(); }
}



bool EthercatMaster::is_ready()
{
    return retcode_ == 0;
}


void EthercatMaster::run()
{
    // printf("\nStarting EtherCAT task.\n\n");
    // printf("** Domain state **\n"); check_domain_state();
    // printf("** Master state **\n"); check_master_state();
    // printf("** Slave states **\n"); check_slave_config_states();
    // printf("\n");

    // Background worker thread
    cyclic_worker_ = std::thread( [this]() {
        struct timespec wakeup_time;
        clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
        wakeup_time.tv_sec += 1; /* start in future */
        wakeup_time.tv_nsec = 0;
        task_counter_ = 0;

        int ret = 0;
        running_ = true;
        while(running_) {
            ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, NULL);
            if (ret) {
                fprintf(stderr, "clock_nanosleep(): %s\n", strerror(ret));
                break;
            }

            cyclic_task();

            wakeup_time.tv_nsec += period_ns_;
            while (wakeup_time.tv_nsec >= NSEC_PER_SEC) {
                wakeup_time.tv_nsec -= NSEC_PER_SEC;
                wakeup_time.tv_sec++;
            }
        } 
        // printf("EtherCAT task ended.\n");
    } );
    std::this_thread::sleep_for(10ms);
}


void EthercatMaster::configure_slaves()
{
    /* Slave info - output from `ethercat cstruct` */
    ec_pdo_entry_info_t sanmotion_pdo_entries[22] = {
        {0x6040, 0x00, 16}, /* Rx: Control word */
        {0x6060, 0x00, 8},  /* Rx: Mode of operation */
        {0x607a, 0x00, 32}, /* Rx: Target position */
        {0x6081, 0x00, 32}, /* Rx: Profile velocity */
        {0x6083, 0x00, 32}, /* Rx: Profile acceleration */
        {0x6084, 0x00, 32}, /* Rx: Profile declaration */
        {0x60ff, 0x00, 32}, /* Rx: Target velocity */
        {0x6071, 0x00, 16}, /* Rx: Target torque */
        {0x60b8, 0x00, 16}, /* Rx: Touch probe function */
        {0x60fe, 0x01, 32}, /* Rx: Physical outputs */
        {0x6041, 0x00, 16}, /* Tx: Status word */
        {0x2100, 0x00, 16}, /* Tx: Status Word 1 */
        {0x6064, 0x00, 32}, /* Tx: Position actual value */
        {0x606c, 0x00, 32}, /* Tx: Velocity actual value */
        {0x6077, 0x00, 16}, /* Tx: Torque actual value */
        {0x60f4, 0x00, 32}, /* Tx: Following error actual value */
        {0x60b9, 0x00, 16}, /* Tx: Touch probe status */
        {0x60ba, 0x00, 32}, /* Tx: Touch probe position 1 positive value */
        {0x60bb, 0x00, 32}, /* Tx: Touch probe position 1 negative value */
        {0x60fd, 0x00, 32}, /* Tx: Digital inputs */
        {0x1001, 0x00, 8},  /* Tx: Error register */
        {0x6061, 0x00, 8},  /* Tx: Mode of operation display */
    };
    ec_pdo_info_t sanmotion_pdos[2] = {
        {0x1700, 10, sanmotion_pdo_entries + 0}, /* RxPDO mapping */
        {0x1b00, 12, sanmotion_pdo_entries + 10}, /* TxPDO mapping */
    };
    ec_sync_info_t sanmotion_syncs[5] = {
        {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
        {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
        {2, EC_DIR_OUTPUT, 1, sanmotion_pdos + 0, EC_WD_DISABLE},
        {3, EC_DIR_INPUT, 1, sanmotion_pdos + 1, EC_WD_DISABLE},
        {0xff}
    };
    for (unsigned int i=0; i<2; i++)
    {
        slave_config_[i] = ecrt_master_slave_config(master_ptr_, 0, i, SANMOTION_ID);
        if (!slave_config_[i]) {
            fprintf(stderr, "Failed to get slave %u configuration.\n", i);
            retcode_++;
        }
        if (ecrt_slave_config_pdos(slave_config_[i], EC_END, sanmotion_syncs)) {
            fprintf(stderr, "Failed to configure PDOs for slave %u.\n", i);
            retcode_++;
        }
    }

}


void EthercatMaster::register_pdo_entry()
{
    std::vector<ec_pdo_entry_reg_t> domain_regs;
    for (uint16_t i=0; i<2; i++) 
    {
        domain_regs.push_back({0, i, SANMOTION_ID, 0x6040, 0x00, &slave_offset_[i].control_word});
        domain_regs.push_back({0, i, SANMOTION_ID, 0x6060, 0x00, &slave_offset_[i].op_mode});
        domain_regs.push_back({0, i, SANMOTION_ID, 0x607a, 0x00, &slave_offset_[i].target_position});
        domain_regs.push_back({0, i, SANMOTION_ID, 0x6081, 0x00, &slave_offset_[i].profile_velocity});
        domain_regs.push_back({0, i, SANMOTION_ID, 0x6083, 0x00, &slave_offset_[i].profile_accel});
        domain_regs.push_back({0, i, SANMOTION_ID, 0x6084, 0x00, &slave_offset_[i].profile_decel});
        domain_regs.push_back({0, i, SANMOTION_ID, 0x60ff, 0x00, &slave_offset_[i].target_velocity});
        domain_regs.push_back({0, i, SANMOTION_ID, 0x6071, 0x00, &slave_offset_[i].target_torque});
        domain_regs.push_back({0, i, SANMOTION_ID, 0x6041, 0x00, &slave_offset_[i].status_word});
        domain_regs.push_back({0, i, SANMOTION_ID, 0x6064, 0x00, &slave_offset_[i].actual_position});
        domain_regs.push_back({0, i, SANMOTION_ID, 0x606c, 0x00, &slave_offset_[i].actual_velocity});
        domain_regs.push_back({0, i, SANMOTION_ID, 0x6077, 0x00, &slave_offset_[i].actual_torque});
        domain_regs.push_back({0, i, SANMOTION_ID, 0x6061, 0x00, &slave_offset_[i].op_mode_display});
    }
    domain_regs.push_back({});

    /* Register desired PDOs on the domain */
    if (ecrt_domain_reg_pdo_entry_list(domain_ptr_, domain_regs.data())) {
        fprintf(stderr, "PDO entry registration failed!\n");
        retcode_++;
    }
}


void EthercatMaster::check_domain_state()
{
    ec_domain_state_t ds;
    ecrt_domain_state(domain_ptr_, &ds);
    printf("Domain WC %u.\n", ds.working_counter);
    printf("Domain state %u.\n", ds.wc_state);
}


void EthercatMaster::check_master_state()
{
    ec_master_state_t ms;
    ecrt_master_state(master_ptr_, &ms);
    printf("%u slave(s).\n", ms.slaves_responding);
    printf("AL states: 0x%02X.\n", ms.al_states);
    printf("Link is %s.\n", ms.link_up ? "up" : "down");
}


void EthercatMaster::check_slave_config_states() 
{
    ec_slave_config_state_t s;
    for (unsigned int i=0; i<2; i++)
    {
        ecrt_slave_config_state(slave_config_[i], &s);
        printf("Slave %u: state 0x%02X.\n", i, s.al_state);
        printf("Slave %u: %s.\n", i, s.online ? "online" : "offline");
        printf("Slave %u: %soperational.\n", i, s.operational ? "" : "not ");
    }
}


void EthercatMaster::read_data()
{
    for (uint16_t i=0; i<2; i++) 
    {
        slave_data_[i].status_word = EC_READ_U16(domain_pd_ + slave_offset_[i].status_word);
        slave_data_[i].fsa_state = fsa_from(slave_data_[i].status_word);
        slave_data_[i].actual_position = EC_READ_S32(domain_pd_ + slave_offset_[i].actual_position);
        slave_data_[i].actual_velocity = EC_READ_S32(domain_pd_ + slave_offset_[i].actual_velocity);
        slave_data_[i].actual_torque = EC_READ_S16(domain_pd_ + slave_offset_[i].actual_torque);
        slave_data_[i].op_mode_display = static_cast<op_mode_t>(EC_READ_S8(domain_pd_ + slave_offset_[i].op_mode_display));
    }
}


void EthercatMaster::displayall()
{
    printf("*** PDOS ***\n");
    printf("control_word:      %26s%#06x %26s%#06x\n", "", slave_data_[0].control_word, "",
                                             slave_data_[1].control_word);
    printf("op_mode:           %32s %32s\n", to_string(slave_data_[0].op_mode), 
                                             to_string(slave_data_[1].op_mode));
    printf("target_position:   %32d %32d\n", slave_data_[0].target_position, 
                                             slave_data_[1].target_position);
    printf("profile_velocity:  %32u %32u\n", slave_data_[0].profile_velocity, 
                                             slave_data_[1].profile_velocity);
    printf("profile_accel:     %32u %32u\n", slave_data_[0].profile_accel, 
                                             slave_data_[1].profile_accel);
    printf("profile_decel:     %32u %32u\n", slave_data_[0].profile_decel, 
                                             slave_data_[1].profile_decel);
    printf("target_velocity:   %32d %32d\n", slave_data_[0].target_velocity, 
                                             slave_data_[1].target_velocity);
    printf("target_torque:     %32d %32d\n", slave_data_[0].target_torque, 
                                             slave_data_[1].target_torque);
    printf("status_word:       %32u %32u\n", slave_data_[0].status_word, 
                                             slave_data_[1].status_word);
    printf("actual_position:   %32d %32d\n", slave_data_[0].actual_position, 
                                             slave_data_[1].actual_position);
    printf("actual_velocity:   %32d %32d\n", slave_data_[0].actual_velocity, 
                                             slave_data_[1].actual_velocity);
    printf("actual_torque:     %32d %32d\n", slave_data_[0].actual_torque, 
                                             slave_data_[1].actual_torque);
    printf("op_mode_display:   %32s %32s\n", to_string(slave_data_[0].op_mode_display),
                                             to_string(slave_data_[1].op_mode_display));
    printf("fsa_state:         %32s %32s\n", to_string(slave_data_[0].fsa_state),
                                             to_string(slave_data_[1].fsa_state));
    printf("\n");
}


void EthercatMaster::write_data()
{
    for (uint16_t i=0; i<2; i++) 
    {
        std::lock_guard<std::mutex> lock(slave_data_[i].mutex);
        EC_WRITE_U16(domain_pd_ + slave_offset_[i].control_word, slave_data_[i].control_word);
        EC_WRITE_U16(domain_pd_ + slave_offset_[i].op_mode, slave_data_[i].op_mode);
        EC_WRITE_S32(domain_pd_ + slave_offset_[i].target_position, slave_data_[i].target_position);
        EC_WRITE_U32(domain_pd_ + slave_offset_[i].profile_velocity, slave_data_[i].profile_velocity);
        EC_WRITE_U32(domain_pd_ + slave_offset_[i].profile_accel, slave_data_[i].profile_accel);
        EC_WRITE_U32(domain_pd_ + slave_offset_[i].profile_decel, slave_data_[i].profile_decel);
        EC_WRITE_S32(domain_pd_ + slave_offset_[i].target_velocity, slave_data_[i].target_velocity);
        EC_WRITE_S16(domain_pd_ + slave_offset_[i].target_torque, slave_data_[i].target_torque);
    }
}


void EthercatMaster::cyclic_task() 
{
    ecrt_master_receive(master_ptr_);
    ecrt_domain_process(domain_ptr_);
    
    if (task_counter_) {
        task_counter_--;
    } else { 
        // this block occur at every (n=frequency_) period (1 Hz)
        task_counter_ = frequency_;
        // printf("** Domain state **\n"); check_domain_state();
        // printf("** Master state **\n"); check_master_state();
        // printf("** Slave states **\n"); check_slave_config_states();
        // printf("\n");
        // displayall();
    }
    
    read_data();
    write_data();

    ecrt_domain_queue(domain_ptr_);
    ecrt_master_send(master_ptr_);
}


void EthercatMaster::stack_prefault() 
{
    unsigned char dummy[MAX_SAFE_STACK];
    memset(dummy, 0, MAX_SAFE_STACK);
}


void EthercatMaster::set_op_mode(const unsigned int slave_id, const op_mode_t mode)
{
    if (mode == slave_data_[slave_id].op_mode) { return ; }
    const std::lock_guard<std::mutex> lock(slave_data_[slave_id].mutex);
    slave_data_[slave_id].op_mode = mode;
}


void EthercatMaster::set_target_position(const unsigned int slave_id, const int32_t target)
{
    if (target == slave_data_[slave_id].target_position) { return ; }
    const std::lock_guard<std::mutex> lock(slave_data_[slave_id].mutex);
    slave_data_[slave_id].target_position = target;
}


void EthercatMaster::set_profile_velocity(const unsigned int slave_id, const uint32_t target)
{
    if (target == slave_data_[slave_id].profile_velocity) { return ; }
    const std::lock_guard<std::mutex> lock(slave_data_[slave_id].mutex);
    slave_data_[slave_id].profile_velocity = target;
}


void EthercatMaster::set_profile_accel(const unsigned int slave_id, const uint32_t target)
{
    if (target == slave_data_[slave_id].profile_accel) { return ; }
    const std::lock_guard<std::mutex> lock(slave_data_[slave_id].mutex);
    slave_data_[slave_id].profile_accel = target;
}


void EthercatMaster::set_profile_decel(const unsigned int slave_id, const uint32_t target)
{
    if (target == slave_data_[slave_id].profile_decel) { return ; }
    const std::lock_guard<std::mutex> lock(slave_data_[slave_id].mutex);
    slave_data_[slave_id].profile_decel = target;
}


void EthercatMaster::set_target_velocity(const unsigned int slave_id, const int32_t target)
{
    if (target == slave_data_[slave_id].target_velocity) { return ; }
    const std::lock_guard<std::mutex> lock(slave_data_[slave_id].mutex);
    slave_data_[slave_id].target_velocity = target;
}


void EthercatMaster::set_target_torque(const unsigned int slave_id, const int16_t target)
{
    if (target == slave_data_[slave_id].target_velocity) { return ; }
    const std::lock_guard<std::mutex> lock(slave_data_[slave_id].mutex);
    slave_data_[slave_id].target_torque = target;
}


void EthercatMaster::enable_drive(const unsigned int slave_id)
{
    if (slave_data_[slave_id].fsa_state == OPERATION_ENABLED) {
        return ;
    }
    set_target_position(slave_id, slave_data_[slave_id].actual_position);
    set_target_velocity(slave_id, 0);
    set_target_torque(slave_id, 0);
    if (slave_data_[slave_id].fsa_state == SWITCH_ON_DISABLED_A || 
        slave_data_[slave_id].fsa_state == SWITCH_ON_DISABLED_B) {
        /* Need to go to READY_TO_SWITCH_ON then OPERATION_ENABLE */
        set_control_word_shutdown(slave_id);
        std::this_thread::sleep_for(std::chrono::nanoseconds(period_ns_*10));
    }
    set_control_word_switchon_enable(slave_id);
    std::this_thread::sleep_for(std::chrono::nanoseconds(period_ns_*10));
}


void EthercatMaster::disable_drive(const unsigned int slave_id)
{
    set_control_word_shutdown(slave_id);
    std::this_thread::sleep_for(std::chrono::nanoseconds(period_ns_*10));
}


void EthercatMaster::clear_faults(const unsigned int slave_id)
{
    if (slave_data_[slave_id].fsa_state == FAULT_A || 
        slave_data_[slave_id].fsa_state == FAULT_B) {
        set_control_word_clear_fault(slave_id);
        std::this_thread::sleep_for(std::chrono::nanoseconds(period_ns_*10));
        set_control_word_shutdown(slave_id);
        std::this_thread::sleep_for(std::chrono::nanoseconds(period_ns_*10));
    } else {
        disable_drive(slave_id);
    }
}


void EthercatMaster::get_actual_position(const unsigned int slave_id, int32_t &value)
{
    value = slave_data_[slave_id].actual_position;
}


void EthercatMaster::get_actual_velocity(const unsigned int slave_id, int32_t &value)
{
    value = slave_data_[slave_id].actual_velocity;
}


void EthercatMaster::get_actual_torque(const unsigned int slave_id, int16_t &value)
{
    value = slave_data_[slave_id].actual_torque;
}


void EthercatMaster::get_op_mode_display(const unsigned int slave_id, op_mode_t &value)
{
    value = slave_data_[slave_id].op_mode_display;
}


void EthercatMaster::get_fsa_state(const unsigned int slave_id, fsa_state_t &value)
{
    value = slave_data_[slave_id].fsa_state;
}


void EthercatMaster::set_control_word_shutdown(const unsigned int slave_id)
{
    const std::lock_guard<std::mutex> lock(slave_data_[slave_id].mutex);
    slave_data_[slave_id].control_word = ~(~0 << 2) << 1; // 0001_1111
}


void EthercatMaster::set_control_word_switchon_enable(const unsigned int slave_id)
{
    const std::lock_guard<std::mutex> lock(slave_data_[slave_id].mutex);
    slave_data_[slave_id].control_word = ~(~0u << 4); // 0000_1111
}


void EthercatMaster::set_control_word_clear_fault(const unsigned int slave_id)
{
    const std::lock_guard<std::mutex> lock(slave_data_[slave_id].mutex);
    slave_data_[slave_id].control_word = (1 << 7); // 1000_0000
}




} // namespace sanmotion_ethercat
