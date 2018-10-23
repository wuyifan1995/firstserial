#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Char.h"
#include "std_msgs/Float32.h"
#include <string> 
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <math.h>
#include "serial/serial.h"
/*****************************************************************************/
using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
/*****************************************************************************/
unsigned char data_terminal0=0x0d;	//“/r"字符
unsigned char data_terminal1=0x0a;	//“/n"字符
unsigned char speed_data[10]={0};	//要发给串口的数据
string rec_buffer;	//串口数据接收变量

union floatData	//union的作用为实现char数组和float之间的转换
{
	float d;
	unsigned char data[4];
}stm32;
/*****************************************************************************/

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{	
	string port("/dev/ttyUSB0");	//串口号
	unsigned long baud = 9600;	//串口波特率
	serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));	//配置串口
	if(msg->data == "a")
	{	unsigned char temp[1];
		temp[0] = 0x61;
		my_serial.write(temp,1);
		};

	if(msg->data == "b")
	{	unsigned char temp[1];
		temp[0] = 0x62;
		my_serial.write(temp,1);
		};	

}

int main(int argc, char *argv[])
{
	string port("/dev/ttyUSB0");//串口号
	unsigned long baud = 9600;//串口波特率
	serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));//配置串口
	ros::init(argc, argv, "stm32data");
	ros::NodeHandle n;
	ros::Publisher numb = n.advertise<std_msgs::Float32>("serial", 1000); //定义要发布/odom主题
	
	ros::Subscriber sub = n.subscribe("keys", 20, chatterCallback);
	std_msgs::Float32 stm32fl;

	ros::Rate loop_rate(10);//设置周期休眠时间
	while(ros::ok())
	{
		rec_buffer =my_serial.readline(25,"\n");	//获取串口发送来的数据 缓存的buffer size = 25
		const char *receive_data=rec_buffer.data();	//保存串口发送来的数据,这里用到了指向了数组的指针
		if(rec_buffer.length()==5)	//串口接收的数据长度正确就处理并发布里程计数据消息
		{
			for(int i=0;i<4;i++)//提取串口的数据，自动幻化成float类型你该
			{
				stm32.data[i]=receive_data[i];
			}

			stm32fl.data = stm32.d;
			numb.publish(stm32fl);

			ros::spinOnce();//周期执行
            loop_rate.sleep();//周期休眠
		}
		//程序周期性调用
		//ros::spinOnce();  //callback函数必须处理所有问题时，才可以用到
	}

	return 0;
}