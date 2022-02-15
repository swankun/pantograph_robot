#include <boost/thread.hpp>
#include <thread>

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <std_srvs/Trigger.h>

#include <pantograph_hardware/pantograph_hw.h>

using namespace std::literals::chrono_literals;

typedef std::chrono::steady_clock time_source;
typedef std_srvs::TriggerRequest  ResetReq;
typedef std_srvs::TriggerResponse ResetRes;


bool resetCb(const ResetReq &req, ResetRes &res, PantographHW* robot)
{
    robot->reset();
    res.success = true;
    return true;
}
bool disableCb(const ResetReq &req, ResetRes &res, PantographHW* robot)
{
    robot->disable();
    res.success = true;
    return true;
}
bool enableCb(const ResetReq &req, ResetRes &res, PantographHW* robot)
{
    robot->enable();
    res.success = true;
    return true;
}
bool homeCb(const ResetReq &req, ResetRes &res, PantographHW* robot)
{
    robot->home();
    res.success = true;
    return true;
}


int main(int argc, char* argv[])
{
    // Initialize ROS node.
    ros::init(argc, argv, "pantograph_hardware");

    // Background thread for the controls .
    ros::NodeHandle nh("~");
    ros::NodeHandle hwnh(nh, "motor_drivers");
    PantographHW robot(nh);
    controller_manager::ControllerManager cm(&robot, nh);
    ros::AsyncSpinner spinner(3);

    // ROS params
    const unsigned int control_loop_rate(nh.param("control_loop_rate_hz", 100)); 
    ROS_INFO("[pantograph_hardware] Control loop rate: %u Hz", control_loop_rate);

    // ROS pub/sub/services for main thread
    ros::ServiceServer service_reset = hwnh.advertiseService<ResetReq,ResetRes>(
        "clear_faults", boost::bind(resetCb, _1, _2, &robot));
    ros::ServiceServer service_disable = hwnh.advertiseService<ResetReq,ResetRes>(
        "disable", boost::bind(disableCb, _1, _2, &robot));
    ros::ServiceServer service_enable = hwnh.advertiseService<ResetReq,ResetRes>(
        "enable", boost::bind(enableCb, _1, _2, &robot));
    ros::ServiceServer service_home = hwnh.advertiseService<ResetReq,ResetRes>(
        "set_home", boost::bind(homeCb, _1, _2, &robot));
    
    spinner.start();
    time_source::time_point last_time = time_source::now();
    ros::Rate rate(control_loop_rate);
    while (ros::ok())
    {
        // Calculate monotonic time elapsed
        time_source::time_point this_time = time_source::now();
        std::chrono::duration<double> elapsed_duration = this_time - last_time;
        ros::Duration elapsed(elapsed_duration.count());
        last_time = this_time;

        robot.read();
        cm.update(ros::Time::now(), elapsed);
        robot.write();
        rate.sleep();
    }
    ros::waitForShutdown();

  return 0;
}
