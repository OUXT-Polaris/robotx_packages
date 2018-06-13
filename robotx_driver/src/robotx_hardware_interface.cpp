//headers in this package
#include <robotx_hardware_interface.h>
#include <robotx_msgs/UsvDrive.h>
#include <robotx_msgs/Heartbeat.h>

robotx_hardware_interface::robotx_hardware_interface() : params_(robotx_hardware_interface::parameters())
{
    heartbeat_pub_ = nh_.advertise<robotx_msgs::Heartbeat>("/heartbeat",1);
    if(params_.target == ALL || params_.target == SIMULATION)
    {
        usv_drive_cmd_pub_ = nh_.advertise<robotx_msgs::UsvDrive>("/cmd_drive", 1);
        left_thrust_joint_pub_ = nh_.advertise<std_msgs::Float64>("/left_thruster_position_controller/command",1);
        right_thrust_joint_pub_ = nh_.advertise<std_msgs::Float64>("/right_thruster_position_controller/command",1);
        last_motor_cmd_msg_.data.resize(4);
        last_motor_cmd_msg_.data[0] = 0;
        last_motor_cmd_msg_.data[1] = 0;
        last_motor_cmd_msg_.data[2] = 0;
        last_motor_cmd_msg_.data[3] = 0;
    }
    if(params_.target == ALL || params_.target == HARDWARE)
    {
        left_motor_cmd_client_ptr_ = new tcp_client(io_service_,params_.left_motor_ip,params_.left_motor_port,params_.timeout);
        right_motor_cmd_client_ptr_ = new tcp_client(io_service_,params_.right_motor_ip,params_.right_motor_port,params_.timeout);
    }
    driving_mode_ = params_.init_mode;
    current_task_number_ = 0;
    fix_sub_ = nh_.subscribe("/fix", 1, &robotx_hardware_interface::fix_callback_, this);
    motor_command_sub_ = nh_.subscribe("/wam_v/motor_command", 1, &robotx_hardware_interface::motor_command_callback_, this);
    boost::thread send_command_thread(boost::bind(&robotx_hardware_interface::send_command_, this));
    boost::thread publish_heartbeat_thread(boost::bind(&robotx_hardware_interface::publish_heartbeat_, this));
    send_command_thread.join();
    publish_heartbeat_thread.join();
}

robotx_hardware_interface::~robotx_hardware_interface()
{

}

void robotx_hardware_interface::current_task_number_callback_(std_msgs::UInt8 msg)
{
    current_task_number_ = msg.data;
    return;
}

void robotx_hardware_interface::fix_callback_(sensor_msgs::NavSatFix msg)
{
    last_fix_msg_ = msg;
    return;
}

void robotx_hardware_interface::joy_callback_(sensor_msgs::Joy msg)
{
    last_joy_cmd_ = msg;
    return;
}

//msg = [left_thruster_cmd left_thruster_joint_angle right_thruster_cmd right_thruster_joint_angle]
void robotx_hardware_interface::motor_command_callback_(std_msgs::Float64MultiArray msg)
{
    if(msg.data.size() == 4)
    {
        last_motor_cmd_msg_ = msg;  
    }
    return;
}

void robotx_hardware_interface::send_command_()
{
    ros::Rate rate(params_.frequency);
    while (ros::ok())
    {
        if(params_.target == ALL || params_.target == SIMULATION)
        {
            robotx_msgs::UsvDrive usv_drive_msg;
            usv_drive_msg.left = last_motor_cmd_msg_.data[0];
            usv_drive_msg.right = last_motor_cmd_msg_.data[2];
            std_msgs::Float64 left_thrust_joint_cmd_;
            left_thrust_joint_cmd_.data = last_motor_cmd_msg_.data[1];
            std_msgs::Float64 right_thrust_joint_cmd_;
            right_thrust_joint_cmd_.data = last_motor_cmd_msg_.data[3];
            usv_drive_cmd_pub_.publish(usv_drive_msg);
            left_thrust_joint_pub_.publish(left_thrust_joint_cmd_);
            right_thrust_joint_pub_.publish(right_thrust_joint_cmd_);
        }
        if(params_.target == ALL || params_.target == HARDWARE)
        {
            left_motor_cmd_client_ptr_->send(last_motor_cmd_msg_.data[0]);
            right_motor_cmd_client_ptr_->send(last_motor_cmd_msg_.data[2]);
        }
        rate.sleep();
    }
}

void robotx_hardware_interface::publish_heartbeat_()
{
    ros::Rate rate(1);
    while (ros::ok())
    {
        robotx_msgs::Heartbeat heartbeat_msg;
        if(last_fix_msg_.latitude > 0)
            heartbeat_msg.north_or_south = heartbeat_msg.NORTH;
        else
            heartbeat_msg.north_or_south = heartbeat_msg.SOUTH;
        heartbeat_msg.latitude = std::fabs(last_fix_msg_.latitude);
        if(heartbeat_msg.longitude > 0)
            heartbeat_msg.east_or_west = heartbeat_msg.EAST;
        else
            heartbeat_msg.east_or_west = heartbeat_msg.WEST;
        heartbeat_msg.longitude = std::fabs(last_fix_msg_.longitude);
        heartbeat_msg.team_id = params_.team_id;
        if(driving_mode_ == REMOTE_OPERATED)
            heartbeat_msg.vehicle_mode = heartbeat_msg.REMOTE_OPERATED;
        if(driving_mode_ == AUTONOMOUS)
            heartbeat_msg.vehicle_mode = heartbeat_msg.AUTONOMOUS;
        if(driving_mode_ == EMERGENCY)
            heartbeat_msg.vehicle_mode = heartbeat_msg.EMERGENCY;
        heartbeat_msg.current_task_number = current_task_number_;
        heartbeat_pub_.publish(heartbeat_msg);
        rate.sleep();
    }
}