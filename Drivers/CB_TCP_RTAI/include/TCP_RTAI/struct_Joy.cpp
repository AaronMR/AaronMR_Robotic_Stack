//#include "AaronMR_C.hpp"
#include "pack2.hpp"
#include "structType_C.hpp"

//-----------------------------------------------struct struct_Joy--------------------------------------


struct_Joy::struct_Joy()
{


    haveSubscriber = false;
    havePublisher = false;

    ros::spinOnce();
    cout << "raro raro" << endl;
    sizeof_Joy = sizeof(Joy);


    auxJoy1.axes[0]=0.0;
    auxJoy1.axes[1]=0.0;
    auxJoy1.axes[2]=0.0;
    auxJoy1.axes[3]=0.0;

    auxJoy1.buttons[0]=0;
    auxJoy1.buttons[1]=0;
    auxJoy1.buttons[2]=0;
    auxJoy1.buttons[3]=0;

    mutex = PTHREAD_MUTEX_INITIALIZER;
    canRecv_t = true;
    canSend_t = true;
}


void struct_Joy::storeData(Joy *joy)
{
    return;
}

void struct_Joy::cmdVelCallback(const joy::Joy::ConstPtr& joy)
{
    //cout << "estoy en el callback del struct_Joy_R" << endl;

    auxJoy1.axes[0]=joy->axes[0];
    auxJoy1.axes[1]=joy->axes[1];
    auxJoy1.axes[2]=joy->axes[2];
    auxJoy1.axes[3]=joy->axes[3];

    auxJoy1.buttons[0]=joy->buttons[0];
    auxJoy1.buttons[1]=joy->buttons[1];
    auxJoy1.buttons[2]=joy->buttons[2];
    auxJoy1.buttons[3]=joy->buttons[3];

    pthread_mutex_lock(&mutex);

    //canRecv_t = true; //rand() % 2 + -1;
    canSend_t = true; //rand() % 2 + -1;

    pthread_mutex_unlock(&mutex);
    //cout << sizeof(Joy) << endl;
/*
    vel.angular = joy->axes[0];
    vel.linear = joy->axes[1];
*/
    //cout << "Estoy aki dentro" << endl;


}

void* struct_Joy::set_Subscriber(char* name)
{
    joy_sub = n.subscribe<joy::Joy>(name, 10, &struct_Joy::cmdVelCallback, this);
    haveSubscriber = true;
    return NULL;
}

void* struct_Joy::set_Publisher(char* name)
{
    joy_pub  = n.advertise<joy::Joy>(name, 1);

    havePublisher = true;
    return NULL;
}



bool struct_Joy::canSend()
{
    int rc = 0;
    int aux = 0;

    rc = pthread_mutex_lock(&mutex);
    aux = canSend_t;
    canSend_t = true;
    rc = pthread_mutex_unlock(&mutex);

    return aux;
}

bool struct_Joy::canRecv()
{
    int rc = 0;
    int aux = 0;

    rc = pthread_mutex_lock(&mutex);
    aux = canRecv_t;
    canRecv_t = true;
    rc = pthread_mutex_unlock(&mutex);

    return aux;
}

int struct_Joy::spinOnce()
{
    ros::spinOnce();
}

int struct_Joy::serialize(char* buf3)
{


/*
    int rc = 0;
    int nohacernada = 0;
    rc = pthread_mutex_lock(&mutex);


    if(canRecv)
    {
        cout << "canRecv = " << canRecv << endl;
        nohacernada = -1;
    }

    if(canSend)
    {
        cout << "canSend = " << canSend << endl;
        nohacernada = -1;
    }

    canRecv = true;
    canSend = true;
    rc = pthread_mutex_unlock(&mutex);

*/



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

    auxJoy2.axes[0] = auxJoy1.axes[0];
    auxJoy2.axes[1] = auxJoy1.axes[1];
    auxJoy2.axes[2] = auxJoy1.axes[2];
    auxJoy2.axes[3] = auxJoy1.axes[3];

    auxJoy2.buttons[0] = auxJoy1.buttons[0];
    auxJoy2.buttons[1] = auxJoy1.buttons[1];
    auxJoy2.buttons[2] = auxJoy1.buttons[2];
    auxJoy2.buttons[3] = auxJoy1.buttons[3];





	packetsize = pack(buf, "CHffffhhhh",    'A',
                                            0,
                                            auxJoy2.axes[0],
                                            auxJoy2.axes[1],
                                            auxJoy2.axes[2],
                                            auxJoy2.axes[3],
                                            auxJoy2.buttons[0],
                                            auxJoy2.buttons[1],
                                            auxJoy2.buttons[2],
                                            auxJoy2.buttons[3]);

	packi16(buf+1, packetsize); // store packet size in packet for kicks


    memcpy((unsigned char*)buf3, buf, packetsize);


	unpack((unsigned char*)buf3, "CHffffhhhh",  &magic,
                                            &ps2,
                                            &auxJoy2.axes[0],
                                            &auxJoy2.axes[1],
                                            &auxJoy2.axes[2],
                                            &auxJoy2.axes[3],
                                            &auxJoy2.buttons[0],
                                            &auxJoy2.buttons[1],
                                            &auxJoy2.buttons[2],
                                            &auxJoy2.buttons[3]);


	printf("send: '%c' %hhu %f %f %f %f %d %d %d %d\n",   magic,
                                                    ps2,
                                                    auxJoy2.axes[0],
                                                    auxJoy2.axes[1],
                                                    auxJoy2.axes[2],
                                                    auxJoy2.axes[3],
                                                    auxJoy2.buttons[0],
                                                    auxJoy2.buttons[1],
                                                    auxJoy2.buttons[2],
                                                    auxJoy2.buttons[3]);

    /*
    cout << "Send to RTAI" << magic << endl;
    cout << " Code = \t" << magic << endl;
    cout << " size_msg = \t" << ps2 << endl;
    cout << " axes[0] = \t" << auxJoy2.axes[0] << endl;
    cout << " axes[1] = \t" << auxJoy2.axes[1] << endl;
    cout << " axes[2] = \t" << auxJoy2.axes[2] << endl;
    cout << " axes[3] = \t" << auxJoy2.axes[3] << endl;
    cout << " buttons[0] = \t" << auxJoy2.buttons[0] << endl;
    cout << " buttons[1] = \t" << auxJoy2.buttons[1] << endl;
    cout << " buttons[2] = \t" << auxJoy2.buttons[2] << endl;
    cout << " buttons[3] = \t" << auxJoy2.buttons[3] << endl;
    */




    return 0;
}

int struct_Joy::Unserialize(char* buf3)
{

    /*
    int rc = 0;
    int nohacernada = 0;
    rc = pthread_mutex_lock(&mutex);


    if(canRecv_t)
    {
        cout << "canRecv = " << canRecv_t << endl;
        nohacernada = -1;
    }

    if(canSend_t)
    {
        cout << "canSend = " << canSend_t << endl;
        nohacernada = -1;
    }

    canRecv_t = false;
    canSend_t = false;
    rc = pthread_mutex_unlock(&mutex);
    */



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


	printf("recv: '%c' %hhu %f %f %f %f %d %d %d %d\n",   magic,
                                                    ps2,
                                                    auxJoy2.axes[0],
                                                    auxJoy2.axes[1],
                                                    auxJoy2.axes[2],
                                                    auxJoy2.axes[3],
                                                    auxJoy2.buttons[0],
                                                    auxJoy2.buttons[1],
                                                    auxJoy2.buttons[2],
                                                    auxJoy2.buttons[3]);

/*
    cout << "Recv from RTAI" << magic << endl;
    cout << " Code = \t" << magic << endl;
    cout << " size_msg = \t" << ps2 << endl;
    cout << " axes[0] = \t" << auxJoy2.axes[0] << endl;
    cout << " axes[1] = \t" << auxJoy2.axes[1] << endl;
    cout << " axes[2] = \t" << auxJoy2.axes[2] << endl;
    cout << " axes[3] = \t" << auxJoy2.axes[3] << endl;
    cout << " buttons[0] = \t" << auxJoy2.buttons[0] << endl;
    cout << " buttons[1] = \t" << auxJoy2.buttons[1] << endl;
    cout << " buttons[2] = \t" << auxJoy2.buttons[2] << endl;
    cout << " buttons[3] = \t" << auxJoy2.buttons[3] << endl;
*/
    if(havePublisher)
    {
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

        joy_pub.publish(joy_msg);
    }
    return 0;
}
