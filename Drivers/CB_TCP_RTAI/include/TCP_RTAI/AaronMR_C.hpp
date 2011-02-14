#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Velocity.h>
#include <joy/Joy.h>
#include "parameters.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include "parser.h"
#include "struct_Pose.hpp"

//-----------------------------------Types---------------------------------------------
class structType {
public:

    virtual int serialize(char* maki) = 0;
    virtual int Unserialize(char* maki) = 0;
    virtual void* set_Publisher(char* name) = 0;
    virtual void* set_Subscriber(char* name) = 0;

    virtual bool canSend() = 0;
    virtual bool canRecv() = 0;

    virtual int spinOnce() = 0;

    void storeData();
    //virtual void serialize(std::ostream& os) const = 0;
};

class struct_Joy : public structType {
public:
    struct_Joy();

    int serialize(char* maki);
    int Unserialize(char* maki);
    void storeData(Joy *joy);
    void* set_Publisher(char* name);
    void* set_Subscriber(char* name);

    ros::NodeHandle n;
    ros::Publisher joy_pub;
    ros::Subscriber joy_sub;

    Joy auxJoy1;

    joy::Joy joy_msg;

    int sizeof_Joy;

    void cmdVelCallback(const joy::Joy::ConstPtr& joy);

    bool haveSubscriber;
    bool havePublisher;

    pthread_mutex_t mutex;

    bool canRecv_t;
    bool canSend_t;

    bool canSend();
    bool canRecv();

    int spinOnce();

};

class struct_Twist : public structType {
public:
    struct_Twist();

    int serialize(char* maki);
    int Unserialize(char* maki);
    void storeData(Joy *joy);
    void* set_Publisher(char* name);
    void* set_Subscriber(char* name);

    ros::NodeHandle n;
    ros::Publisher twist_pub;
    ros::Subscriber twist_sub;

    Joy auxJoy1;

    joy::Joy joy_msg;
    geometry_msgs::Twist twist_;

    int sizeof_Joy;

    void cmdVelCallback(const geometry_msgs::Twist& joy);

    bool haveSubscriber;
    bool havePublisher;

    pthread_mutex_t mutex;

    bool canRecv_t;
    bool canSend_t;

    bool canSend();
    bool canRecv();

    int spinOnce();

};


class struct_Pose : public structType {
public:
    struct_Pose();

    int serialize(char* maki);
    int Unserialize(char* maki);
    void storeData(Joy *joy);
    void* set_Publisher(char* name);
    void* set_Subscriber(char* name);

    ros::NodeHandle n;
    ros::Publisher Pose_pub;
    ros::Subscriber Pose_sub;

    Joy auxJoy1;
    Pose auxPose1;

    joy::Joy joy_msg;
    geometry_msgs::Twist twist_;

    int sizeof_Joy;

    void cmdCallback(const geometry_msgs::Twist& joy);

    bool haveSubscriber;
    bool havePublisher;

    pthread_mutex_t mutex;

    bool canRecv_t;
    bool canSend_t;

    bool canSend();
    bool canRecv();

    int spinOnce();

};

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
