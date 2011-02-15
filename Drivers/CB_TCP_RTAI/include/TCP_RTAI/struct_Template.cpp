
#include "pack2.hpp"
#include "structType_C.hpp"

//-----------------------------------------------struct Template--------------------------------------


struct_Template::struct_Template()
{


    haveSubscriber = false;
    havePublisher = false;

    ros::spinOnce();
    cout << "raro raro" << endl;



    mutex = PTHREAD_MUTEX_INITIALIZER;
    canRecv_t = true;
    canSend_t = true;
}


void struct_Template::storeData(Joy *joy)
{
    return;
}

void struct_Template::cmdCallback(const geometry_msgs::Point &data_)
{

    pthread_mutex_lock(&mutex);

    canSend_t = true;

    pthread_mutex_unlock(&mutex);


}

void* struct_Template::set_Subscriber(char* name)
{
    template_sub = n.subscribe(name, 10, &struct_Template::cmdCallback, this);
    haveSubscriber = true;
    return NULL;
}

void* struct_Template::set_Publisher(char* name)
{
    template_pub  = n.advertise<geometry_msgs::Point>(name, 1);

    havePublisher = true;
    return NULL;
}



bool struct_Template::canSend()
{
    int rc = 0;
    int aux = 0;

    rc = pthread_mutex_lock(&mutex);
    aux = canSend_t;
    canSend_t = true;
    rc = pthread_mutex_unlock(&mutex);

    return aux;
}

bool struct_Template::canRecv()
{
    int rc = 0;
    int aux = 0;

    rc = pthread_mutex_lock(&mutex);
    aux = canRecv_t;
    canRecv_t = true;
    rc = pthread_mutex_unlock(&mutex);

    return aux;
}

int struct_Template::spinOnce()
{
    ros::spinOnce();
}

int struct_Template::serialize(char* data2s)
{

	unsigned char buf[1024];
	unsigned char magic;  // type of message
	unsigned int packetsize;
	unsigned int ps2;

    struct_Template_t aux;

    aux.val1 = 0.0;
    aux.val2 = 0.0;
    aux.val3 = 0.0;




    /**
    c = signed char             -> 8-bit
    C = unsigned char           -> 8-bit unsigned

    h = int                     -> 16-bit
    H = unsigned int            -> 16-bit unsigned

    l = long int                -> 32-bit
    L = unsigned long int       -> 32-bit unsigned

    q = long long int           -> 64-bit
    Q = unsigned long long int  -> 64-bit unsigned

    d = double                  -> float-32

    g = long double             -> float-64

    s = char*                   -> string

    f = double                  -> float-16

    **/


    /** CHhhh is :
        C = char to type of message
        H = for the leng of the message
        hhh = to send 3 int
    */

	packetsize = pack(buf, "CHddd",    'A',
                                        0,
                                        aux.val1,
                                        aux.val2,
                                        aux.val3);

	packi16(buf+1, packetsize); // store packet size in packet for kicks

    memcpy((unsigned char*)data2s, buf, packetsize);

#ifdef DEBUG
	unpack((unsigned char*)data2s, "CHddd", &magic,
                                            &ps2,
                                            &aux.val1,
                                            &aux.val2,
                                            &aux.val3);


	printf("send: '%c' %hhu %f %f %f\n",    magic,
                                            ps2,
                                            aux.val1,
                                            aux.val2,
                                            aux.val3);
#endif

    return 0;
}

int struct_Template::Unserialize(char* data2us)
{

	unsigned char buf[1024];
	unsigned char magic;
    unsigned int ps2;

    struct_Template_t aux;

    memcpy(buf, data2us, 1024);

    aux.val1 = 0;
    aux.val2 = 0;
    aux.val3 = 0;

    Joy auxJoy2;
	unpack(buf, "CHhhh",    &magic,
                            &ps2,
                            &aux.val1,
                            &aux.val2,
                            &aux.val3);


	printf("recv: '%c' %hhu %d %d %d\n", magic,
                                            ps2,
                                            aux.val1,
                                            aux.val2,
                                            aux.val3);


    if(havePublisher)
    {
        point_msg.x = aux.val1;
        point_msg.y = aux.val2;
        point_msg.z = aux.val3;

        template_pub.publish(point_msg);
    }
    return 0;
}
