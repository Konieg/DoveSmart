
#ifndef _ORIGINCAR_BASE_H_
#define _ORIGINCAR_BASE_H_

#include <memory>
#include <inttypes.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <csignal>
#include <thread>

#include <iostream>
#include <string.h>
#include <string>
#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <rcl/types.h>
#include <sys/stat.h>

#include <serial/serial.h>
#include <fcntl.h>
#include <stdbool.h>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_ros/transform_broadcaster.h>
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "origincar_msg/msg/data.hpp"
#include "origincar_msg/msg/sign.hpp"  // 匹配信号发送
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
using namespace std;


#define SEND_DATA_CHECK   1          //Send data check flag bits //发送数据校验标志位
#define READ_DATA_CHECK   0          //Receive data to check flag bits //接收数据校验标志位
#define FRAME_HEADER      0X7B       //Frame head //帧头
#define FRAME_TAIL        0X7D       //Frame tail //帧尾
#define RECEIVE_DATA_SIZE 24         //The length of the data sent by the lower computer //下位机发送过来的数据的长度
#define SEND_DATA_SIZE    11         //The length of data sent by ROS to the lower machine //ROS向下位机发送的数据的长度
#define PI 				  3.1415926f //PI //圆周率

#define GYROSCOPE_RATIO   0.00026644f

#define ACCEl_RATIO 	  1671.84f

extern sensor_msgs::msg::Imu Mpu6050;

const double odom_pose_covariance[36]   = {1e-3,    0,    0,   0,   0,    0,
										      0, 1e-3,    0,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0,  1e3 };

const double odom_pose_covariance2[36]  = {1e-9,    0,    0,   0,   0,    0,
										      0, 1e-3, 1e-9,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0, 1e-9 };

const double odom_twist_covariance[36]  = {1e-3,    0,    0,   0,   0,    0,
										      0, 1e-3,    0,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0,  1e3 };

const double odom_twist_covariance2[36] = {1e-9,    0,    0,   0,   0,    0,
										      0, 1e-3, 1e-9,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0, 1e-9} ;

typedef struct __Vel_Pos_Data_
{
	float X;
	float Y;
	float Z;

} Vel_Pos_Data;

typedef struct __MPU6050_DATA_
{
	short accele_x_data;
	short accele_y_data;
	short accele_z_data;
    short gyros_x_data;
	short gyros_y_data;
	short gyros_z_data;

} MPU6050_DATA;

typedef struct _SEND_DATA_
{
	uint8_t tx[SEND_DATA_SIZE];
	float X_speed;
	float Y_speed;
	float Z_speed;
	unsigned char Frame_Tail;
} SEND_DATA;

typedef struct _RECEIVE_DATA_
{
	uint8_t rx[RECEIVE_DATA_SIZE];
	uint8_t Flag_Stop;
	unsigned char Frame_Header;
	float X_speed;
	float Y_speed;
	float Z_speed;
	float Power_Voltage;
	unsigned char Frame_Tail;
} RECEIVE_DATA;

class origincar_base : public rclcpp::Node

{
public:
	origincar_base();
	~origincar_base();
	void Control();
	void Publish_Odom();

public : 
	serial::Serial Stm32_Serial;

private:
	void declare_parameters();
	void get_parameters();


	void Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux);
	void Akm_Cmd_Vel_Callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr akm_ctl);

	void Publish_ImuSensor();
	void Publish_Voltage();
	auto createQuaternionMsgFromYaw(double yaw);

	bool Get_Sensor_Data();
	unsigned char Check_Sum(unsigned char Count_Number,unsigned char mode);
	short IMU_Trans(uint8_t Data_High,uint8_t Data_Low);
	float Odom_Trans(uint8_t Data_High,uint8_t Data_Low);

	void Sign_Switch_Callback(const std_msgs::msg::Int32::SharedPtr sign_switch);

private:
	rclcpp::Time _Now, _Last_Time;
	float Sampling_Time;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr Cmd_Vel_Sub;
	rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr Akm_Cmd_Vel_Sub;

	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_publisher;
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher; 

	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr test_publisher;

	rclcpp::Publisher<origincar_msg::msg::Data>::SharedPtr robotpose_publisher;
	rclcpp::Publisher<origincar_msg::msg::Data>::SharedPtr robotvel_publisher;
	// rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr tf_pub_;

	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_bro;
	rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
	rclcpp::TimerBase::SharedPtr test_timer;

	rclcpp::TimerBase::SharedPtr odom_timer;
	rclcpp::TimerBase::SharedPtr imu_timer;
	rclcpp::TimerBase::SharedPtr voltage_timer;

	rclcpp::TimerBase::SharedPtr robotpose_timer;
	rclcpp::TimerBase::SharedPtr robotvel_timer;

	std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr Sign_Switch_Sub;

	string usart_port_name, robot_frame_id, gyro_frame_id, odom_frame_id, akm_cmd_vel, test;
	std::string cmd_vel;
	int serial_baud_rate;
	RECEIVE_DATA Receive_Data;
	SEND_DATA Send_Data;

	Vel_Pos_Data Robot_Pos;
	Vel_Pos_Data Robot_Vel;
	MPU6050_DATA Mpu6050_Data;
	float Power_voltage;
    size_t count_;
};


#endif //_ORIGINCAR_BASE_H_
