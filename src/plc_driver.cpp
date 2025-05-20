#include <algorithm>
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include "plc_driver/plc_driver.h"
#include <geometry_msgs/PoseStamped.h> // Updated header inclusion

PLCDriver::PLCDriver(ros::NodeHandle& nh) : nh_(nh), pnh_("~")
{
    // Get serial communication parameters from config file
    ros::NodeHandle pnh("~");
    pnh_.getParam("serial_port", serial_port_);
    pnh_.getParam("baud_rate", serial_baudrate_);
    pnh_.getParam("Stopbits", Stopbits_);
    pnh_.getParam("Databits", Databits_);
    pnh_.getParam("timeout", serial_timeout_);
    pnh_.getParam("parity", serial_parity_);

    // Initialize publishers and subscribers
    pub_bool_ = nh_.advertise<std_msgs::Bool>("plc_bool_out", 1000);
    pub_int_ = nh_.advertise<std_msgs::Int32>("plc_int_out", 1000);
    pub_float_ = nh_.advertise<std_msgs::Float32>("plc_float_out", 1000);
    pub_PoseStamped_ = nh_.advertise<geometry_msgs::PoseStamped>("plc_PoseStamped_out", 1000);

    sub_bool_ = nh_.subscribe("plc_bool_in", 1000, &PLCDriver::writeserialBool, this);
    sub_int_ = nh_.subscribe("plc_int_in", 1000, &PLCDriver::writeserialInt, this);
    sub_float_ = nh_.subscribe("plc_float_in", 1000, &PLCDriver::writeserialFloat, this);
    sub_PoseStamped_ = nh_.subscribe("plc_PoseStamped_in", 1000, &PLCDriver::writeserialVector3, this);

    // Connect to the serial port
    serialConnect();
}

PLCDriver::~PLCDriver()
{
    serialDisconnect();
}

void PLCDriver::serialConnect()
{
    try {
        serial_port_.erase(std::remove_if(serial_port_.begin(), serial_port_.end(), ::isspace), serial_port_.end());

        ROS_INFO_STREAM("Trying to open serial port: '" << serial_port_ << "'");

        if (serial_port_.empty()) {
            ROS_ERROR("Serial port is empty after loading from parameter!");
            return;
        }

        serial_.setPort(serial_port_);
        serial_.setBaudrate(serial_baudrate_);

        serial::Timeout timeout = serial::Timeout::simpleTimeout(static_cast<uint32_t>(serial_timeout_ * 1000));
        serial_.setTimeout(timeout);
        serial_.open();

        if (serial_.isOpen()) {
            ROS_INFO_STREAM("Successfully connected to " << serial_port_ << " at " << serial_baudrate_);
        } else {
            ROS_ERROR_STREAM("Failed to open port " << serial_port_);
        }

    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Exception while opening serial port: " << e.what());
    }
}

void PLCDriver::serialDisconnect()
{
    if (serial_.isOpen()) {
        serial_.close();
        ROS_INFO_STREAM("Serial port closed: " << serial_port_);
    }
}


void PLCDriver::readserial()
{
    if (serial_.isOpen()) {
        std::string data = serial_.readline();
        ROS_INFO_STREAM("Received data: " << data);
    } else {
        ROS_WARN("Serial port is not open.");
    }
}

void PLCDriver::writeserialBool(const std_msgs::Bool::ConstPtr& msg)
{
    if (serial_.isOpen()) {
        std::string data = msg->data ? "1\n" : "0\n";
        serial_.write(data);
    } else {
        ROS_WARN("Serial port is not open.");
    }
}

void PLCDriver::writeserialInt(const std_msgs::Int32::ConstPtr& msg)
{
    if (serial_.isOpen()) {
        serial_.write(std::to_string(msg->data) + "\n");
    } else {
        ROS_WARN("Serial port is not open.");
    }
}

void PLCDriver::writeserialFloat(const std_msgs::Float32::ConstPtr& msg)
{
    if (serial_.isOpen()) {
        serial_.write(std::to_string(msg->data) + "\n");
    } else {
        ROS_WARN("Serial port is not open.");
    }
}

void PLCDriver::writeserialVector3(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if (serial_.isOpen()) {
        std::stringstream ss;
        ss << msg->pose.position.x << "," << msg->pose.position.y << "," << msg->pose.position.z << "\n";
        serial_.write(ss.str());
    } else {
        ROS_WARN("Serial port is not open.");
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "plc_driver_node");
    ros::NodeHandle nh;
    PLCDriver plc_driver(nh);
   // std::string raw_message = ser.read(ser.available());
   // ROS_INFO_STREAM("RAW:" << raw_message);
    ros::spin();
    return 0;
}
