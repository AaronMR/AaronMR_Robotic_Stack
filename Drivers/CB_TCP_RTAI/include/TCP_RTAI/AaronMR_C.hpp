#include <ros/ros.h>


#include <sys/socket.h>
#include <netinet/in.h>
#include "parser.h"
#include "struct_Pose.hpp"

#include "structType_C.hpp"

class AaronMR_C
{
public:
    AaronMR_C(char * aux);
    ~AaronMR_C();

    int create_Socket();
    int connect_Socket();
    int send_Data();
    int recv_Data();

    structType* structToSend;
    structType* structToRecv;
    DataLayout configuration[5];
    char buffer[1024];

    char name_[20];
    char Node2RTAI_[20];
    char RTAI2Node_[20];

    char* nameProcess();

    int SetupCode(char* buffer);

    int SendRecv();

private:

    int host_port;
    char* host_name;
    char* Subscriber_name;
    char* Publisher_name;
    struct sockaddr_in my_addr;

    int hsock;

    int buffer_len;


/*
    Struct_1 a;
    Struct_2 b;
    Struct_3 c;
    Twist twist;
    Joy joy;
    Odometry odometry;
*/
    ros::NodeHandle n;
    ros::Publisher vel_pub;
    ros::Subscriber joy_sub;

    ros::Publisher joy_pub;

    void cmdVelCallback(const joy::Joy::ConstPtr& joy);

};
