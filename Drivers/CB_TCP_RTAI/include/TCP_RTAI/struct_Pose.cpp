//#include "AaronMR_C.hpp"
#include "pack2.hpp"
#include "structType_C.hpp"
//############################################### struct_Pose ##############################

struct_Pose::struct_Pose()
{


    haveSubscriber = false;
    havePublisher = false;

    ros::spinOnce();
    cout << "raro raro" << endl;
    sizeof_Joy = sizeof(Joy);

    auxPose1.position.x = 0.0;
    auxPose1.position.y = 0.0;
    auxPose1.position.z = 0.0;

    auxPose1.orientation.x = 0.0;
    auxPose1.orientation.y = 0.0;
    auxPose1.orientation.z = 0.0;
    auxPose1.orientation.w = 0.0;

}

int struct_Pose::spinOnce()
{
    ros::spinOnce();
}

bool struct_Pose::canSend()
{
    int rc = 0;
    int aux = 0;

    rc = pthread_mutex_lock(&mutex);
    aux = canSend_t;
    rc = pthread_mutex_unlock(&mutex);

    return aux;
}

bool struct_Pose::canRecv()
{
    int rc = 0;
    int aux = 0;

    rc = pthread_mutex_lock(&mutex);
    aux = canRecv_t;
    rc = pthread_mutex_unlock(&mutex);

    return aux;
}

void struct_Pose::storeData(Joy *joy)
{
    return;
}


void struct_Pose::cmdCallback(const geometry_msgs::Twist &msg)
{

    cout << "estoy en el callback del struct_Joy_R" << endl;

    cout << "Estoy aki dentro" << endl;


}

void* struct_Pose::set_Subscriber(char* name)
{
    Pose_sub = n.subscribe(name, 10, &struct_Pose::cmdCallback, this);
    haveSubscriber = true;
    return NULL;
}

void* struct_Pose::set_Publisher(char* name)
{
    Pose_pub  = n.advertise<geometry_msgs::Twist>(name, 1);

    havePublisher = true;
    return NULL;
}



int struct_Pose::serialize(char* buf3)
{
    ros::spinOnce();
//---------------------------------------------------------------------------------------
	unsigned char buf[1024];
//	unsigned char buf2[1024]="makiboludo";
	unsigned char magic;
//	int monkeycount;
//	long altitude;
//	double absurdityfactor;
//	char *s = "Maki";
//	char s2[96];
	unsigned int packetsize, ps2;
//    double maki[9];

    //Joy auxJoy2;

    Twist twist;

    Pose pose;

    twist.angular.x = 1.1;
    twist.angular.y = 2.2;
    twist.angular.z = 3.3;

    twist.linear.x = 4.4;
    twist.linear.y = 5.5;
    twist.linear.z = 6.6;

    pose.position.x = 7.0;
    pose.position.y = 6.0;
    pose.position.z = 5.0;

    pose.orientation.x = 4.0;
    pose.orientation.y = 3.0;
    pose.orientation.z = 2.0;
    pose.orientation.w = 1.0;




    /*
    auxJoy2.axes[0] = auxJoy1.axes[0];
    auxJoy2.axes[1] = auxJoy1.axes[1];
    auxJoy2.axes[2] = auxJoy1.axes[2];
    auxJoy2.axes[3] = auxJoy1.axes[3];

    auxJoy2.buttons[0] = auxJoy1.buttons[0];
    auxJoy2.buttons[1] = auxJoy1.buttons[1];
    auxJoy2.buttons[2] = auxJoy1.buttons[2];
    auxJoy2.buttons[3] = auxJoy1.buttons[3];
    */



    packetsize = pack(buf, "CHddddddd",    'A',
                                            0,
                                            pose.position.x,
                                            pose.position.y,
                                            pose.position.z,
                                            pose.orientation.x,
                                            pose.orientation.y,
                                            pose.orientation.z,
                                            pose.orientation.w);

	packi16(buf+1, packetsize); // store packet size in packet for kicks


    memcpy((unsigned char*)buf3, buf, packetsize);


	unpack((unsigned char*)buf3, "CHddddddd",  &magic,
                                            &ps2,
                                            &pose.position.x,
                                            &pose.position.y,
                                            &pose.position.z,
                                            &pose.orientation.x,
                                            &pose.orientation.y,
                                            &pose.orientation.z,
                                            &pose.orientation.w);


	printf("send: '%c' %hhu %f %f %f %f %f %f %f\n",   magic,
                                                    ps2,
                                                    pose.position.x,
                                                    pose.position.y,
                                                    pose.position.z,
                                                    pose.orientation.x,
                                                    pose.orientation.y,
                                                    pose.orientation.z,
                                                    pose.orientation.w);

}

int struct_Pose::Unserialize(char* buf3)
{

//---------------------------------------------------------------------------------------
	unsigned char buf[1024];
//	unsigned char buf2[1024]="makiboludo";
	unsigned char magic;
//	int monkeycount;
//	long altitude;
//	double absurdityfactor;
//	char *s = "Maki";
//	char s2[96];
//	unsigned int packetsize,
    unsigned int ps2;
//    double maki[9];

    /*
    memcpy(buf, buf3, 1024);

    Twist twist2;

	unpack((unsigned char*)buf, "CHdddddd",  &magic,
                                            &ps2,
                                            &twist2.angular.x,
                                            &twist2.angular.y,
                                            &twist2.angular.z,
                                            &twist2.linear.x,
                                            &twist2.linear.y,
                                            &twist2.linear.z);

	printf("recv: '%c' %hhu %f %f %f %f %f %f\n",   magic,
                                                    ps2,
                                                    twist2.angular.x,
                                                    twist2.angular.y,
                                                    twist2.angular.z,
                                                    twist2.linear.x,
                                                    twist2.linear.y,
                                                    twist2.linear.z);
    */

    memcpy(buf, buf3, 1024);

    Pose pose;

    unpack((unsigned char*)buf, "CHddddddd",  &magic,
                                            &ps2,
                                            &pose.position.x,
                                            &pose.position.y,
                                            &pose.position.z,
                                            &pose.orientation.x,
                                            &pose.orientation.y,
                                            &pose.orientation.z,
                                            &pose.orientation.w);


	printf("send: '%c' %hhu %f %f %f %f %f %f %f\n",   magic,
                                                        ps2,
                                                        pose.position.x,
                                                        pose.position.y,
                                                        pose.position.z,
                                                        pose.orientation.x,
                                                        pose.orientation.y,
                                                        pose.orientation.z,
                                                        pose.orientation.w);

    /*


    Joy auxJoy2;
	unpack(buf, "CHffffhhhh",  &magic,
                                            &ps2,
                                            &auxJoy2.axes[0],
                                            &auxJoy2.axes[1],
                                            &auxJoy2.axes[2],
                                            &auxJoy2.axes[3],
                                            &auxJoy2.buttons[0],
                                            &auxJoy2.buttons[1],
                                            &auxJoy2.buttons[2],
                                            &auxJoy2.buttons[3]);

	printf("'%c' %hhu %f %f %f %f %d %d %d %d\n",   magic,
                                                    ps2,
                                                    auxJoy2.axes[0],
                                                    auxJoy2.axes[1],
                                                    auxJoy2.axes[2],
                                                    auxJoy2.axes[3],
                                                    auxJoy2.buttons[0],
                                                    auxJoy2.buttons[1],
                                                    auxJoy2.buttons[2],
                                                    auxJoy2.buttons[3]);
*/
    if(havePublisher)
    {
        /*
        joy_msg.buttons.resize(4);
        joy_msg.axes.resize(4);

        joy_msg.axes[0] = auxJoy2.axes[0];
        joy_msg.axes[1] = auxJoy2.axes[1];
        joy_msg.axes[2] = auxJoy2.axes[2];
        joy_msg.axes[3] = auxJoy2.axes[3];

        joy_msg.buttons[0] = auxJoy2.buttons[0];
        joy_msg.buttons[1] = auxJoy2.buttons[1];
        joy_msg.buttons[2] = auxJoy2.buttons[2];
        joy_msg.buttons[3] = auxJoy2.buttons[3];
        */

        /*
        twist_.angular.x = twist2.angular.x;
        twist_.angular.y = twist2.angular.y;
        twist_.angular.z = twist2.angular.z;
        twist_.linear.x = twist2.linear.x;
        twist_.linear.y = twist2.linear.y;
        twist_.linear.z = twist2.linear.z;

        Pose_pub.publish(joy_msg);
        */
    }
    return  NULL;
}

//###########################################################################################



