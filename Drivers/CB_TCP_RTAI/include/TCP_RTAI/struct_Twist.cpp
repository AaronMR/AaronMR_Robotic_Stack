#include "AaronMR_C.hpp"
#include "pack2.hpp"

//############################################### struct_Twist ##############################

struct_Twist::struct_Twist()
{


    haveSubscriber = false;
    havePublisher = false;

    ros::spinOnce();
    cout << "raro raro" << endl;
    sizeof_Joy = sizeof(Joy);

    mutex = PTHREAD_MUTEX_INITIALIZER;
    canRecv_t = true;
    canSend_t = true;
}


int struct_Twist::spinOnce()
{
    ros::spinOnce();
}

bool struct_Twist::canSend()
{
    int rc = 0;
    int aux = 0;

    rc = pthread_mutex_lock(&mutex);
    aux = canSend_t;
    rc = pthread_mutex_unlock(&mutex);

    return aux;
}

bool struct_Twist::canRecv()
{
    int rc = 0;
    int aux = 0;

    rc = pthread_mutex_lock(&mutex);
    aux = canRecv_t;
    rc = pthread_mutex_unlock(&mutex);

    return aux;
}

void struct_Twist::storeData(Joy *joy)
{
    return;
}


void struct_Twist::cmdVelCallback(const geometry_msgs::Twist &msg)
{

    cout << "estoy en el callback del struct_Joy_R" << endl;

    cout << "Estoy aki dentro" << endl;


}

void* struct_Twist::set_Subscriber(char* name)
{
    twist_sub = n.subscribe(name, 10, &struct_Twist::cmdVelCallback, this);
    haveSubscriber = true;
    return NULL;
}

void* struct_Twist::set_Publisher(char* name)
{
    twist_pub  = n.advertise<geometry_msgs::Twist>(name, 1);

    havePublisher = true;
    return NULL;
}



int struct_Twist::serialize(char* buf3)
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

    Joy auxJoy2;

    Twist twist;

    twist.angular.x = 1.1;
    twist.angular.y = 2.2;
    twist.angular.z = 3.3;

    twist.linear.x = 4.4;
    twist.linear.y = 5.5;
    twist.linear.z = 6.6;





    auxJoy2.axes[0] = auxJoy1.axes[0];
    auxJoy2.axes[1] = auxJoy1.axes[1];
    auxJoy2.axes[2] = auxJoy1.axes[2];
    auxJoy2.axes[3] = auxJoy1.axes[3];

    auxJoy2.buttons[0] = auxJoy1.buttons[0];
    auxJoy2.buttons[1] = auxJoy1.buttons[1];
    auxJoy2.buttons[2] = auxJoy1.buttons[2];
    auxJoy2.buttons[3] = auxJoy1.buttons[3];





	packetsize = pack(buf, "CHdddddd",    'A',
                                            0,
                                            twist.angular.x,
                                            twist.angular.y,
                                            twist.angular.z,
                                            twist.linear.x,
                                            twist.linear.y,
                                            twist.linear.z);

	packi16(buf+1, packetsize); // store packet size in packet for kicks


    memcpy((unsigned char*)buf3, buf, packetsize);


	unpack((unsigned char*)buf3, "CHdddddd",  &magic,
                                            &ps2,
                                            &twist.angular.x,
                                            &twist.angular.y,
                                            &twist.angular.z,
                                            &twist.linear.x,
                                            &twist.linear.y,
                                            &twist.linear.z);

	printf("send: '%c' %hhu %f %f %f %f %f %f\n",   magic,
                                                    ps2,
                                                    twist.angular.x,
                                                    twist.angular.y,
                                                    twist.angular.z,
                                                    twist.linear.x,
                                                    twist.linear.y,
                                                    twist.linear.z);

}

int struct_Twist::Unserialize(char* buf3)
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

        twist_.angular.x = twist2.angular.x;
        twist_.angular.y = twist2.angular.y;
        twist_.angular.z = twist2.angular.z;
        twist_.linear.x = twist2.linear.x;
        twist_.linear.y = twist2.linear.y;
        twist_.linear.z = twist2.linear.z;

        twist_pub.publish(joy_msg);
    }
    return  NULL;
}

