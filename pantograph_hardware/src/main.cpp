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
    ros::init(argc, argv, "pantograph_base_node");

    // Background thread for the controls .
    ros::NodeHandle nh;
    ros::NodeHandle cmnh(nh, "pantograph_controller");
    PantographHW robot(nh);
    controller_manager::ControllerManager cm(&robot, cmnh);
    ros::AsyncSpinner spinner(3);

    // ROS pub/sub/services for main thread
    ros::ServiceServer service_reset = nh.advertiseService<ResetReq,ResetRes>(
        "clear_motor_faults", boost::bind(resetCb, _1, _2, &robot));
    ros::ServiceServer service_disable = nh.advertiseService<ResetReq,ResetRes>(
        "disable_motors", boost::bind(disableCb, _1, _2, &robot));
    ros::ServiceServer service_enable = nh.advertiseService<ResetReq,ResetRes>(
        "enable_motors", boost::bind(enableCb, _1, _2, &robot));
    ros::ServiceServer service_home = nh.advertiseService<ResetReq,ResetRes>(
        "set_home", boost::bind(homeCb, _1, _2, &robot));
    
    spinner.start();

    // std::thread(std::bind(controlThread, ros::Rate(50), &robot, &cm));
    time_source::time_point last_time = time_source::now();
    ros::Rate rate(50);
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
