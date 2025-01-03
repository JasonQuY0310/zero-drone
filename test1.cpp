#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>

ros::ServiceClient arming_client;
ros::ServiceClient takeoff_client;
ros::ServiceClient set_mode_client;

// 当前状态
mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

bool set_mode(const std::string& mode)
{
    mavros_msgs::SetMode srv;
    srv.request.custom_mode = mode;

    if (set_mode_client.call(srv))
    {
        if (srv.response.mode_sent)
        {
            ROS_INFO("Successfully set mode to %s.", mode.c_str());
            return true;
        }
        else
        {
            ROS_ERROR("Failed to set mode to %s.", mode.c_str());
            return false;
        }
    }
    else
    {
        ROS_ERROR("Failed to call service /mavros/set_mode.");
        return false;
    }
}

bool takeoff(double altitude)
{
    mavros_msgs::CommandTOL srv;
    srv.request.altitude = altitude;
    if (takeoff_client.call(srv))
    {
        ROS_INFO("Takeoff initiated to %.2f meters", altitude);
        return true;
    }
    else
    {
        ROS_ERROR("Failed to call service /mavros/cmd/takeoff.");
        return false;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "px4_takeoff_cpp_node");
    ros::NodeHandle nh;

    // 服务客户端初始化
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    // 订阅飞机状态
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

    // 等待飞机状态变为ARMED
    ROS_INFO("Waiting for UAV to be armed...");
    while (ros::ok() && !current_state.armed)
    {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    ROS_INFO("Vehicle armed!");

    // 发出解锁命令
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    if (arming_client.call(arm_cmd) && arm_cmd.response.success)
    {
        ROS_INFO("Vehicle armed");
    }
    else
    {
        ROS_ERROR("Failed to arm vehicle");
        return -1;
    }

    // 等待起飞前的准备
    ros::Duration(1).sleep();

    // 执行起飞到2米
    if (takeoff(2.0))
    {
        // 等待起飞后稳定
        ros::Duration(10).sleep();  // 起飞后保持稳定10秒
    }

    // 切换到Hold模式
    if (set_mode("HOLD"))
    {
        ROS_INFO("Vehicle in HOLD mode.");
    }
    else
    {
        ROS_ERROR("Failed to set vehicle to HOLD mode.");
    }

    ros::spin();
    return 0;
}
