# ROS串口通讯实验
## 实验目的
用串口通讯实现stm32与上位机ROS的通讯，进行数据交换。具体一点就是stm32下位机不断地发送1-10给上位机，然后接受上位机键盘的控制信息，键盘“a”相当于红灯的控制开关，键盘“b”相当于黄灯的控制开关。
## 技术要点
-  串口通讯是一帧一帧地传输，每一帧包含8位的数据char类型，用的ASCII码。所以传输的数据必须从下位机转化成ASCII码，传输到上位机 然后再从上位机上转化为原来的数据类型
- 数据的转化是在两边用了C语言中的数据Union体，这个会自动分配内存，比如float是4个字节，而串口只能传输1个字节char类型，通过Union定义一个float类型以及char的数组来存放float类型，这样就可以非常方便的自动转化。
~~~C++
union data  //数据共用体
{
float odoemtry_float; 
unsigned char odometry_char[4]; //4个char 装一个float
}stm32;    
~~~
- 上位机读取串口的信息使用了C++中的serial包，这个包的api有个readline 是以“\n” 结束符来读取，所以在下位机发送数据的时候也要在末尾加一个结束符“\n”
~~~C++
rec_buffer =my_serial.readline(25,"\n");    //获取串口发送来的数据 缓存的buffer size = 25
const char *receive_data=rec_buffer.data();    //保存串口发送来的数据,这里用到了指向了数组的指针
if(rec_buffer.length()==5)    //串口接收的数据长度正确就处理并发布里程计数据消息
{
for(int i=0;i<4;i++)//提取串口的数据，自动幻化成float类型你该
{
stm32.data[i]=receive_data[i];
}

stm32fl.data = stm32.d;
numb.publish(stm32fl);
}
~~~
- ROS有自己的数据体系 有的可以通用有的则不行，比如ROS有Float32 和Float64，所以必须用ROS声明一个缓存变量来存放下位机取到的数据。
## 代码
### 下位机
- 串口的中断函数
~~~C++
void USART1_IRQHandler(void)                    //串口1中断服务程序
{
u8 Res;
#if SYSTEM_SUPPORT_OS         //如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
OSIntEnter();    
#endif
if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
{
Res =USART_ReceiveData(USART1);    //读取接收到的数据
switch(Res)
{
case 'a':
LED0 = !LED0;
break;
case 'b':
LED1 = !LED1;
break;
}            
} 
~~~
- main.c
~~~C++
#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "usart.h"

/************************************************
stm32 与ROS上位机通讯初试，
stm32 不断发送float类型的1-10 给上位机
然后stm32接受上位机的数据
如果是 a 则led0 switch
如果是 b 则led1 switch
************************************************/
/***********************************************  输出  *****************************************************************/

char stm32_data[5]={0};   //发送给串口的数据相当于一个float类型

/***********************************************  变量  *****************************************************************/

union data  //数据共用体
{
float odoemtry_float;
unsigned char odometry_char[4];
}stm32;    

/****************************************************************************************************************/    
int main(void)
{    
int i=0;
int m=0;
int n=0;
delay_init();             //延时函数初始化      
NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);      //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
uart_init(9600);     //串口初始化为9600
LED_Init();               //初始化与LED连接的硬件接口 
delay_ms(500);        //让人看得到灭

LED0=0;                 //点亮LED0
LED1=0;                //点亮LED1


while(1)
{    
for(i=0; i<=10;i++)
{
stm32.odoemtry_float = i;
for(m=0;m<4;m++)
{
stm32_data[m]=stm32.odometry_char[m];
}
stm32_data[4]='\n';//添加结束符
for(n=0;n<5;n++)
{
USART_ClearFlag(USART1,USART_FLAG_TC);  //在发送第一个数据前加此句，解决第一个数据不能正常发送的问题                
USART_SendData(USART1,stm32_data[n]);//发送一个字节到串口    
while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);    //等待发送结束            
}
}
}
}
~~~
### 上位机
- stm32data_sub.cpp
~~~C++
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
unsigned char data_terminal0=0x0d;    //“/r"字符
unsigned char data_terminal1=0x0a;    //“/n"字符
unsigned char speed_data[10]={0};    //要发给串口的数据
string rec_buffer;    //串口数据接收变量

union floatData    //union的作用为实现char数组和float之间的转换
{
float d;
unsigned char data[4];
}stm32;
/*****************************************************************************/

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{    
string port("/dev/ttyUSB0");    //串口号
unsigned long baud = 9600;    //串口波特率
serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));    //配置串口
if(msg->data == "a")
{    unsigned char temp[1];
temp[0] = 0x61;
my_serial.write(temp,1);
};

if(msg->data == "b")
{    unsigned char temp[1];
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
rec_buffer =my_serial.readline(25,"\n");    //获取串口发送来的数据 缓存的buffer size = 25
const char *receive_data=rec_buffer.data();    //保存串口发送来的数据,这里用到了指向了数组的指针
if(rec_buffer.length()==5)    //串口接收的数据长度正确就处理并发布里程计数据消息
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
}
return 0;
}
~~~
- 键盘程序
~~~python
#!/usr/bin/env python
'''key_publisher ROS Node'''
# license removed for brevity
import sys, select, tty, termios
import rospy
from std_msgs.msg import String

if __name__ == '__main__':
key_pub = rospy.Publisher('keys', String, queue_size=1)
rospy.init_node("keyboard_driver")
rate = rospy.Rate(100)
old_attr = termios.tcgetattr(sys.stdin)
tty.setcbreak(sys.stdin.fileno())
print "publishig keystrokes. Press Ctrl c to exit ...."
while not rospy.is_shutdown():
if select.select([sys.stdin],[],[],0)[0] == [sys.stdin]:
key_pub.publish(sys.stdin.read(1))
rate.sleep()
termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
~~~
