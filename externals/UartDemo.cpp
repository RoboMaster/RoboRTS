#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <assert.h>
#include <termios.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <sys/types.h>
#include <errno.h>
#include <cmath>
#include <chrono>
#include <iostream>

# define BARE_RUN
# ifndef BARE_RUN
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
# endif
#define PI 3.14159265

static int ret;
static int fd;
float a[3],w[3],Angle[3],h[3];

struct CurrentPose
{
    float x;
    float y;
    float theta; // In rads
};
bool init = false;
float theta = 0;

CurrentPose currentpose = {0,0,0};

#define BAUD 9600
// open device based on device name

int uart_open(int fd,const char *pathname)
{
    fd = open(pathname, O_RDWR|O_NOCTTY); 
    if (-1 == fd)
    { 
        perror("Can't Open Serial Port"); 
		return(-1); 
	} 
    else
		printf("open %s success!\n",pathname);
    /*测试是否为终端设备*/ 
    if(isatty(STDIN_FILENO)==0) 
		printf("standard input is not a terminal device\n"); 
    else 
		printf("isatty success!\n"); 
    return fd; 
}

int uart_set(int fd,int nSpeed, int nBits, char nEvent, int nStop)
{
        struct termios newtio,oldtio; 
        /*保存测试现有串口参数设置，在这里如果串口号等出错，会有相关的出错信息*/ 
        if  ( tcgetattr( fd,&oldtio)  !=  0) {  
        perror("SetupSerial 1");
        printf("tcgetattr( fd,&oldtio) -> %d\n",tcgetattr( fd,&oldtio)); 
        return -1; 
        } 
        bzero( &newtio, sizeof( newtio ) ); 
        /*步骤一，设置字符大小*/ 
        newtio.c_cflag  |=  CLOCAL | CREAD;  
        newtio.c_cflag &= ~CSIZE;  
    /*设置停止位*/ 
        switch( nBits ) 
        { 

        case 7: 

        newtio.c_cflag |= CS7; 

        break; 

        case 8: 

        newtio.c_cflag |= CS8; 

        break; 

        } 

    /*设置奇偶校验位*/ 

        switch( nEvent ) 

        { 

        case 'o':

        case 'O': //奇数 

        newtio.c_cflag |= PARENB; 

        newtio.c_cflag |= PARODD; 

        newtio.c_iflag |= (INPCK | ISTRIP); 

        break; 

        case 'e':

        case 'E': //偶数 

        newtio.c_iflag |= (INPCK | ISTRIP); 

        newtio.c_cflag |= PARENB; 

        newtio.c_cflag &= ~PARODD; 

        break;

        case 'n':

        case 'N':  //无奇偶校验位 

        newtio.c_cflag &= ~PARENB; 

        break;

        default:

        break;

        } 

        /*设置波特率*/ 

    switch( nSpeed ) 

        { 

        case 2400: 

        cfsetispeed(&newtio, B2400); 

        cfsetospeed(&newtio, B2400); 

        break; 

        case 4800: 

        cfsetispeed(&newtio, B4800); 

        cfsetospeed(&newtio, B4800); 

        break; 

        case 9600: 

        cfsetispeed(&newtio, B9600); 

        cfsetospeed(&newtio, B9600); 

        break; 

        case 115200: 

        cfsetispeed(&newtio, B115200); 

        cfsetospeed(&newtio, B115200); 

        break; 

        case 460800: 

        cfsetispeed(&newtio, B460800); 

        cfsetospeed(&newtio, B460800); 

        break; 

        default: 

        cfsetispeed(&newtio, B9600); 

        cfsetospeed(&newtio, B9600); 

        break; 

        } 

    /*设置停止位*/ 

        if( nStop == 1 ) 

        newtio.c_cflag &=  ~CSTOPB; 

        else if ( nStop == 2 ) 

        newtio.c_cflag |=  CSTOPB; 

    /*设置等待时间和最小接收字符*/ 

        newtio.c_cc[VTIME]  = 0; 

        newtio.c_cc[VMIN] = 0; 

    /*处理未接收字符*/ 

        tcflush(fd,TCIFLUSH); 

    /*激活新配置*/ 

    if((tcsetattr(fd,TCSANOW,&newtio))!=0) 

        { 

        perror("com set error"); 

        return -1; 

        } 

        printf("set done!\n"); 

        return 0; 
}

int uart_close(int fd)
{
    assert(fd);
    close(fd);

    /*可以在这里做些清理工作*/

    return 0;
}

int send_data(int  fd, char *send_buffer,int length)
{
	length=write(fd,send_buffer,length*sizeof(unsigned char));
	return length;

}

int recv_data(int fd, char* recv_buffer,int length)
{
	length=read(fd,recv_buffer,length);
	return length;
}
# ifndef BARE_RUN
void pose_recv_callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    // How to unpack data
    currentpose.x = msg->x;
    currentpose.y = msg->y;
}
#endif
float angle_from_h(float h[3])
{
    return atan2(h[1],h[0]);
}

float ring(float i) // Calc in Rads
{
    i += PI/2;
    float num = fmod(PI+fmod(i,PI),  PI);
    return num - PI/2;
}

// Assume we need to transfer to python version here contains data we need to transfer
// 一次一个字节的存储解释数据
void ParseData(char chr)
{
		static char chrBuf[100];
		static unsigned char chrCnt=0;
		signed short sData[4];
		unsigned char i;
		time_t now;
		chrBuf[chrCnt++]=chr;
		if (chrCnt<11) return;
		
		if ((chrBuf[0]!=0x55)||((chrBuf[1]&0x50)!=0x50)) {printf("Error:%x %x\r\n",chrBuf[0],chrBuf[1]);memcpy(&chrBuf[0],&chrBuf[1],10);chrCnt--;return;}
		memcpy(&sData[0],&chrBuf[2],8);
		switch(chrBuf[1])
		{
				case 0x51:
					for (i=0;i<3;i++) a[i] = (float)sData[i]/32768.0*16.0;
					time(&now);
					printf("\r\nT:%s a:%6.3f %6.3f %6.3f ",asctime(localtime(&now)),a[0],a[1],a[2]);
					
					break;
				case 0x52:
					for (i=0;i<3;i++) 
                        w[i] = (float)sData[i]/32768.0*2000.0;
					printf("w:%7.3f %7.3f %7.3f ",w[0],w[1],w[2]);
					
					break;
				case 0x53:
					for (i=0;i<3;i++) 
                        Angle[i] = (float)sData[i]/32768.0*180.0;
					printf("A:%7.3f %7.3f %7.3f ",Angle[0],Angle[1],Angle[2]);
					break;
				case 0x54: // Here is desired data magnetic field data
					for (i=0;i<3;i++) 
                        h[i] = (float)sData[i]/32768.0*1.0;
					printf("h:%4.0f %4.0f %4.0f ",h[1],h[1],h[2]); // xyz??
					// Add a process function
                    theta = angle_from_h(h);

					break;
		}		
		chrCnt=0;
		
}
#ifndef BARE_RUN
void TaskLoop(float theta,ros::Publisher pose_pub)
{
    static bool oninit = true;
    static float yaw_offset = 0;
    static int init_counter = 0;
    static geometry_msgs::Pose2D forwardmsg;
    static std::chrono::high_resolution_clock::time_point last_time = std::chrono::high_resolution_clock::now();
    if(oninit)
    {
        init_counter ++;
        if(init_counter>1)
            yaw_offset += theta/init_counter + yaw_offset/(init_counter-1);
        else if(init_counter==1)
            yaw_offset = theta;
        if(init_counter>2000) // Supposingly 2s
        {
            oninit = false;
            init = true;
        }
    }
    else
    {
        theta  = ring(theta - yaw_offset);
        currentpose.theta = theta;
        forwardmsg.x = currentpose.x;
        forwardmsg.y = currentpose.y;
        forwardmsg.theta = currentpose.theta;
        // Publish message
        if(std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - last_time).count() > 0.025) // May be 40fps
        {
            last_time = std::chrono::high_resolution_clock::now();
            pose_pub.publish(forwardmsg);
        }
    }
}
#else
void TaskLoop(float theta)
{
    static bool oninit = true;
    static float yaw_offset = 0;
    static int init_counter = 0;
    static std::chrono::high_resolution_clock::time_point last_time = std::chrono::high_resolution_clock::now();
    if(oninit)
    {
        init_counter ++;
        if(init_counter>1)
            yaw_offset += theta/init_counter + yaw_offset/(init_counter-1);
        else if(init_counter==1)
            yaw_offset = theta;
        if(init_counter>2000) // Supposingly 2s
        {
            oninit = false;
            init = true;
        }
    }
    else
    {
        theta  = ring(theta - yaw_offset);
        currentpose.theta = theta;
         if(std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - last_time).count() > 0.1) // May be 40fps
        {
            last_time = std::chrono::high_resolution_clock::now();
            std::cout<<"Theta Angle is: "<<theta<<std::endl;
        }
    }
}


#endif
int main(int argc,char **argv)
{
    #ifndef BARE_RUN
    ros::init(argc, argv, "external_localization_node");
    ros::NodeHandle   n;
    ros::Publisher    pose_pub = n.advertise<geometry_msgs::Pose>("uwb",1000);
    ros::Subscriber  pose_sub = n.subscribe<geometry_msgs::Pose2D> ("robot_pose",1000,pose_recv_callback);
    #endif
    char r_buf[1024];
    bzero(r_buf,1024);

    fd = uart_open(fd,"/dev/ttyUSB0");/*串口号/dev/ttySn,USB口号/dev/ttyUSBn*/
    if(fd == -1)
    {
        fprintf(stderr,"uart_open error\n");
        exit(EXIT_FAILURE);
    }

    if(uart_set(fd,BAUD,8,'N',1) == -1)
    {
        fprintf(stderr,"uart set failed!\n");
        exit(EXIT_FAILURE);
    }

	FILE *fp;
	fp = fopen("Record.txt","w");
    while(1)
    {
        ret = recv_data(fd,r_buf,44);
        if(ret == -1)
        {
            fprintf(stderr,"uart read failed!\n");
            exit(EXIT_FAILURE);
        }
		for (int i=0;i<ret;i++) {fprintf(fp,"%2X ",r_buf[i]);ParseData(r_buf[i]);}
        // Add Combined Publish methods
        usleep(1000); // Each us
        #ifndef BARE_RUN
        TaskLoop(theta,pose_pub);
        ros::spinOnce();
        #else
        TaskLoop(theta);
        #endif

    }

    ret = uart_close(fd);
    if(ret == -1)
    {
        fprintf(stderr,"uart_close error\n");
        exit(EXIT_FAILURE);
    }

    exit(EXIT_SUCCESS);
}
