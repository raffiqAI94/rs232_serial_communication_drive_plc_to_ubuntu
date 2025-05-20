#ifndef PLC_DRIVER_H
#define PLC_DRIVER_H

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>


class PLCDriver
{
public:
    PLCDriver(ros::NodeHandle& nh);
    ~PLCDriver();

    void readserial();
    void writeserialBool(const std_msgs::Bool::ConstPtr& msg);
    void writeserialInt(const std_msgs::Int32::ConstPtr& msg);
    void writeserialFloat(const std_msgs::Float32::ConstPtr& msg);
    void writeserialVector3(const geometry_msgs::PoseStamped::ConstPtr& msg);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    serial::Serial serial_;
	//serial::Serial* serial_ctx_;

    ros::Publisher pub_bool_;
    ros::Publisher pub_int_;
    ros::Publisher pub_float_;
    ros::Publisher pub_PoseStamped_;

    ros::Subscriber sub_bool_;
    ros::Subscriber sub_int_;
    ros::Subscriber sub_float_;
    ros::Subscriber sub_PoseStamped_;

    std::string serial_port_;
    int serial_baudrate_;
    int serial_timeout_;
    int Stopbits_;
    int Databits_;
    std::string serial_parity_;

    void serialConnect();
    void serialDisconnect();
};

#endif // PLC_DRIVER_H
