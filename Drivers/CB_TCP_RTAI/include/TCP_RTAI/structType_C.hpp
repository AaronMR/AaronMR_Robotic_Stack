#include <ros/ros.h>
//#include "parameters.h"
#include "comStruct.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Velocity.h>
#include <joy/Joy.h>

#include <geometry_msgs/Point.h>
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

class struct_Template : public structType {
public:
    struct_Template();

    int serialize(char* data2s);
    int Unserialize(char* data2us);
    void storeData(Joy *joy);
    void* set_Publisher(char* name);
    void* set_Subscriber(char* name);

    ros::NodeHandle n;
    ros::Publisher template_pub;
    ros::Subscriber template_sub;



    // struct to send and receive
    struct_Template_t data2send;
    struct_Template_t data2recv;

    geometry_msgs::Point point_msg;



    void cmdCallback(const geometry_msgs::Point &data_);

    bool haveSubscriber;
    bool havePublisher;

    pthread_mutex_t mutex;

    bool canRecv_t;
    bool canSend_t;

    bool canSend();
    bool canRecv();

    int spinOnce();

};
