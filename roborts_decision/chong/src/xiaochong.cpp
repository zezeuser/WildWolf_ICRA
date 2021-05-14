#include     <stdio.h>      /*标准输入输出定义*/
#include     <string.h>
#include     <iostream>
#include     <stdlib.h>     /*标准函数库定义*/
#include     <unistd.h>     /*Unix 标准函数定义*/
#include     <sys/types.h>
#include     <sys/stat.h>
#include     <fcntl.h>      /*文件控制定义*/
#include     <termios.h>    /*PPSIX 终端控制定义*/
#include     <errno.h>      /*错误号定义*/
#include      <ros/ros.h>
#include     <chong/Speech.h>
#include      <sstream>
using namespace std;

int openPort(const char * dev_name);
int configurePort(int fd);
int main(int argc, char **argv)
{

    ros::init(argc,argv,"chongge");
    ros::NodeHandle n;
    ros::Publisher pub=n.advertise<chong::Speech>("count",10);
    chong::Speech msg;
    int fd;

    char code[1];
    memset(code, '0', 2);
    fd = openPort("/dev/ttyACM1");
    fd = configurePort(fd);
//    memset(p,0,1);


//    while(1)
//    {
//        memset(buff,0,strlen(buff));
//        write(fd,rw,1);
//        cout<<buff<<endl;
//    }
    while(1)
    {
//        write(fd,"ok",2);
         read(fd,code,1);
        //printf("awsd%f=",code[0]-'48');
         cout<<code[0]<<endl;
         msg.speech_count=(int)(code[0]-'0');
         ROS_INFO("i=%d", msg.speech_count);
         pub.publish(msg);
         sleep(1);
    }
    ros::spin();
    return 0;
}
//打开串口
int openPort(const char * dev_name)
{
    int fd; // file description for the serial port
    fd = open(dev_name, O_RDWR | O_NOCTTY | O_NDELAY);
    if(fd == -1)
    { // if open is unsucessful
        printf("open_port: Unable to open /dev/ttyS0. \n");
    }
    else
    {
        fcntl(fd, F_SETFL, 0);
        printf("port is open.\n");
    }
    return(fd);
}

//设置串口
int configurePort(int fd)
{
    struct termios port_settings;               // structure to store the port settings
    cfsetispeed(&port_settings, B9600);       // set baud rates
    cfsetospeed(&port_settings, B9600);                                               // set no parity, stop bits, data bits
    port_settings.c_cflag &= ~PARENB;           //no校验
    port_settings.c_cflag &= ~CSTOPB;           //停止位  1
    port_settings.c_cflag &= ~CSIZE;            //屏蔽其他标志位
    port_settings.c_cflag |= CS8;               //数据位  8
    port_settings.c_cc[VTIME] = 10;           //接收速度 x0.1
    port_settings.c_cc[VMIN] = 0;


    tcsetattr(fd, TCSANOW, &port_settings);     // apply the settings to the port
    cout<<"configurePort"<<endl;
    return(fd);
}

