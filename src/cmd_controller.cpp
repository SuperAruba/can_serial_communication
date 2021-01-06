#include "ros/ros.h"
#include <serial/serial.h> //ROS已经内置了的串口包
#include <geometry_msgs/Twist.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

std::string interface;
serial::Serial ser; //声明串口对象
int s;
int count_check = 0;
#define SERLENGTH 14
#define CANLENGTH 8

uint8_t CRC8(uint8_t *p, uint8_t len)
{
    uint8_t bit0, cbit, i, j, byte, temp;
    temp = 0;
    for (j = 0; j < len; j++)
    {
        byte = p[j];
        for (i = 0; i < 8; i++)
        {
            cbit = temp & 0x01;
            bit0 = byte & 0x01;
            temp = temp >> 1;
            if (cbit ^ bit0)
            {
                temp ^= 0x8c;
            }
            byte >>= 1;
        }
    }
    return temp;
}

//回调函数(订阅了cmd/vel话题)
void speed_callback(const geometry_msgs::Twist &cmd_vel)
{
   if(interface == "serial")
   {
        unsigned char buffer[SERLENGTH];
        buffer[0] = 0x5a;
        buffer[1] = 0x0c;
        buffer[2] = 1;
        buffer[3] = 1;
        int16_t speed = cmd_vel.linear.x * 1000;
        ROS_INFO_STREAM("liners: " << speed);
        speed = htons(speed);
        memcpy(buffer + 4, &speed, 2);//buffer 5 6
        buffer[6] = 0x00;
        buffer[7] = 0x00;
        speed = cmd_vel.angular.z * 1000;
        speed = htons(speed);
        ROS_INFO_STREAM("angs: " << speed);
        memcpy(buffer + 8, &speed, 2);// buffer 8 9
        buffer[10] = 0x00;

        unsigned char data[11];
        memcpy(data, buffer, 11);
        buffer[11] = CRC8(data, 11);
        buffer[12] = 0x0D;
        buffer[13] = 0x0A;
        for (int i = 0; i < SERLENGTH; i++)
        {
            printf("%.2X ", buffer[i]);
        }
        if (ser.isOpen())
        {
          ROS_INFO("\r\n ");
          ser.write(buffer, SERLENGTH); //发送串口数据
        }
    }else if(interface == "can") 
    {
        struct can_frame frame;
        frame.can_id = 0x100;
        frame.can_dlc = 0x08; //数据长度
        int16_t speed = cmd_vel.linear.x * 1000;
        ROS_INFO_STREAM("liners: " << speed);
        speed = htons(speed);
        memcpy(frame.data, &speed, 2);//buffer 5 6
        frame.data[2] = 0x00;
        frame.data[3] = 0x00;
        speed = cmd_vel.angular.z * 1000;
        speed = htons(speed);
        ROS_INFO_STREAM("angs: " << speed);
        memcpy(frame.data + 4, &speed, 2);// buffer 8 9
        frame.data[6] = count_check;
        unsigned char data[7];
        memcpy(data, frame.data, 7);
        frame.data[7]  = CRC8(data, 7);
        write(s, &frame, sizeof(frame));
        if(count_check == 255)
        {
            count_check = 0;
        }else{
            count_check++ ;
        }

        for (int i = 0; i < 8; i++)
        {
            // ROS_INFO_STREAM("buffer" << i << " = " << (int)buffer[i]);
            printf("%.2X ", frame.data[i]);
        }
    } 
}
int main(int argc, char **argv)
{
    std::string port;
    int baudrate;

    //初始化节点
    ros::init(argc, argv, "cmd_controller");
    ROS_INFO("Init Node.");
    //声明节点句柄
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    //三个参数：interface（使用的接口 can/serial），baudrate（波特率），port(在系统查找的设备名称)
    nh_private.param<std::string>("interface", interface, "can");
    nh_private.param<int>("baudrate", baudrate, 500000);

    nh_private.param<std::string>("port", port, "can0");

    if(interface == "serial")
    {
       port = "/dev/" + port;
       baudrate = 115200;
       try
       {
          //设置串口属性，并打开串口
          ser.setPort(port);
          ser.setBaudrate(baudrate);
          serial::Timeout to = serial::Timeout::simpleTimeout(500);
          ser.setTimeout(to);
          ser.open();
       }
          catch (serial::IOException &e)
       {
       ROS_ERROR_STREAM("Unable to open port ");
          return -1;
       }
    }else
    {
      //can接口使用的socketcan
      const char* port_ = port.c_str();
	  struct sockaddr_can addr;
	  struct ifreq ifr;
	  s = socket(PF_CAN, SOCK_RAW, CAN_RAW);//创建 SocketCAN 套接字
      strcpy(ifr.ifr_name,port_);
	  ioctl(s, SIOCGIFINDEX, &ifr);//指定 can0 设备
	  addr.can_family = AF_CAN;
	  addr.can_ifindex = ifr.ifr_ifindex;
	  bind(s, (struct sockaddr *)&addr, sizeof(addr)); //将套接字与 can port 绑定
	  setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);//不接收报文，只负责发送
    }

    ROS_INFO_STREAM("port : " << port);
    ROS_INFO_STREAM("baudrate : " << baudrate);
    ROS_INFO_STREAM("interface : " << interface);

    ros::Subscriber sub = nh.subscribe("/cmd_vel", 1000, speed_callback);
    
    ros::spin();
    close(s);
    return 0;
}
