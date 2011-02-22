#include "pack2.hpp"
#include "structType_C.hpp"

struct_posWheels::struct_posWheels()
{

    haveSubscriber  = false;
    havePublisher   = false;
    canRecv_t       = true;
    canSend_t       = true;
    mutex           = PTHREAD_MUTEX_INITIALIZER;


    polygon_msg.set_points_size(4);

    auxSerialize.pos_W[0].x = 0.0;
    auxSerialize.pos_W[0].y = 0.0;
    auxSerialize.pos_W[0].z = 0.0;

    auxSerialize.pos_W[1].x = 0.0;
    auxSerialize.pos_W[1].y = 0.0;
    auxSerialize.pos_W[1].z = 0.0;

    auxSerialize.pos_W[2].x = 0.0;
    auxSerialize.pos_W[2].y = 0.0;
    auxSerialize.pos_W[2].z = 0.0;

    auxSerialize.pos_W[3].x = 0.0;
    auxSerialize.pos_W[3].y = 0.0;
    auxSerialize.pos_W[3].z = 0.0;

    auxUnSerialize.pos_W[0].x = 0.0;
    auxUnSerialize.pos_W[0].y = 0.0;
    auxUnSerialize.pos_W[0].z = 0.0;
    auxUnSerialize.pos_W[1].x = 0.0;
    auxUnSerialize.pos_W[1].y = 0.0;
    auxUnSerialize.pos_W[1].z = 0.0;
    auxUnSerialize.pos_W[2].x = 0.0;
    auxUnSerialize.pos_W[2].y = 0.0;
    auxUnSerialize.pos_W[2].z = 0.0;
    auxUnSerialize.pos_W[3].x = 0.0;
    auxUnSerialize.pos_W[3].y = 0.0;
    auxUnSerialize.pos_W[3].z = 0.0;
}

void struct_posWheels::cmdCallback(const geometry_msgs::Polygon &data_)
{

    auxSerialize.pos_W[0].x = data_.points[0].x;
    auxSerialize.pos_W[0].y = data_.points[0].y;
    auxSerialize.pos_W[0].z = data_.points[0].z;

    auxSerialize.pos_W[1].x = data_.points[0].x;
    auxSerialize.pos_W[1].y = data_.points[0].y;
    auxSerialize.pos_W[1].z = data_.points[0].z;

    auxSerialize.pos_W[2].x = data_.points[0].x;
    auxSerialize.pos_W[2].y = data_.points[0].y;
    auxSerialize.pos_W[2].z = data_.points[0].z;

    auxSerialize.pos_W[3].x = data_.points[0].x;
    auxSerialize.pos_W[3].y = data_.points[0].y;
    auxSerialize.pos_W[3].z = data_.points[0].z;


    pthread_mutex_lock(&mutex);

    canSend_t = true;

    pthread_mutex_unlock(&mutex);


}

void* struct_posWheels::set_Subscriber(char* name)
{

    posWheels_sub = n.subscribe(name, 10, &struct_posWheels::cmdCallback, this);
    haveSubscriber = true;
    return NULL;

}

void* struct_posWheels::set_Publisher(char* name)
{

    posWheels_pub  = n.advertise<geometry_msgs::Polygon>(name, 1);

    havePublisher = true;
    return NULL;

}



bool struct_posWheels::canSend()
{
    /*
    int rc = 0;
    int aux = 0;

    rc = pthread_mutex_lock(&mutex);
    aux = canSend_t;
    canSend_t = true;
    rc = pthread_mutex_unlock(&mutex);

    return aux;
    */
}

bool struct_posWheels::canRecv()
{
    /*
    int rc = 0;
    int aux = 0;

    rc = pthread_mutex_lock(&mutex);
    aux = canRecv_t;
    canRecv_t = true;
    rc = pthread_mutex_unlock(&mutex);

    return aux;
    */
}

int struct_posWheels::spinOnce()
{
    /*
    ros::spinOnce();
    */
}

int struct_posWheels::serialize(char* data2s)
{
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

    CHhhh is :
        C = char to type of message
        H = for the leng of the message
        hhh = to send 3 int
    */

	unsigned char buf[1024];
	unsigned char magic;
	unsigned int packetsize;
	unsigned int ps2;

	packetsize = pack(buf, "CHdddddddddddd",    'A',
                                                0,
                                                auxSerialize.pos_W[0].x,
                                                auxSerialize.pos_W[0].y,
                                                auxSerialize.pos_W[0].z,
                                                auxSerialize.pos_W[1].x,
                                                auxSerialize.pos_W[1].y,
                                                auxSerialize.pos_W[1].z,
                                                auxSerialize.pos_W[2].x,
                                                auxSerialize.pos_W[2].y,
                                                auxSerialize.pos_W[2].z,
                                                auxSerialize.pos_W[3].x,
                                                auxSerialize.pos_W[3].y,
                                                auxSerialize.pos_W[3].z);

	packi16(buf+1, packetsize); // store packet size in packet for kicks

    memcpy((unsigned char*)data2s, buf, packetsize);


	unpack((unsigned char*)data2s, "CHdddddddddddd", &magic,
                                            &ps2,
                                            &auxSerialize.pos_W[0].x,
                                            &auxSerialize.pos_W[0].y,
                                            &auxSerialize.pos_W[0].z,
                                            &auxSerialize.pos_W[1].x,
                                            &auxSerialize.pos_W[1].y,
                                            &auxSerialize.pos_W[1].z,
                                            &auxSerialize.pos_W[2].x,
                                            &auxSerialize.pos_W[2].y,
                                            &auxSerialize.pos_W[2].z,
                                            &auxSerialize.pos_W[3].x,
                                            &auxSerialize.pos_W[3].y,
                                            &auxSerialize.pos_W[3].z);


	printf("posWheels - send: '%c' %hhu %f %f %f %f %f %f %f %f %f %f %f %f\n",    magic,
                                            ps2,
                                            auxSerialize.pos_W[0].x,
                                            auxSerialize.pos_W[0].y,
                                            auxSerialize.pos_W[0].z,
                                            auxSerialize.pos_W[1].x,
                                            auxSerialize.pos_W[1].y,
                                            auxSerialize.pos_W[1].z,
                                            auxSerialize.pos_W[2].x,
                                            auxSerialize.pos_W[2].y,
                                            auxSerialize.pos_W[2].z,
                                            auxSerialize.pos_W[3].x,
                                            auxSerialize.pos_W[3].y,
                                            auxSerialize.pos_W[3].z);


    return 0;
}

int struct_posWheels::Unserialize(char* data2us)
{

	unsigned char buf[1024];
	unsigned char magic;
    unsigned int ps2;

    memcpy(buf, data2us, 1024);

    unpack((unsigned char*)buf, "CHdddddddddddd",
                                &magic,
                                &ps2,
                                &auxUnSerialize.pos_W[0].x,
                                &auxUnSerialize.pos_W[0].y,
                                &auxUnSerialize.pos_W[0].z,
                                &auxUnSerialize.pos_W[1].x,
                                &auxUnSerialize.pos_W[1].y,
                                &auxUnSerialize.pos_W[1].z,
                                &auxUnSerialize.pos_W[2].x,
                                &auxUnSerialize.pos_W[2].y,
                                &auxUnSerialize.pos_W[2].z,
                                &auxUnSerialize.pos_W[3].x,
                                &auxUnSerialize.pos_W[3].y,
                                &auxUnSerialize.pos_W[3].z
                                );


	printf("posWheels - recv: '%c' %hhu %f %f %f %f %f %f %f %f %f %f %f %f\n",
                                            magic,
                                            ps2,
                                            auxUnSerialize.pos_W[0].x,
                                            auxUnSerialize.pos_W[0].y,
                                            auxUnSerialize.pos_W[0].z,
                                            auxUnSerialize.pos_W[1].x,
                                            auxUnSerialize.pos_W[1].y,
                                            auxUnSerialize.pos_W[1].z,
                                            auxUnSerialize.pos_W[2].x,
                                            auxUnSerialize.pos_W[2].y,
                                            auxUnSerialize.pos_W[2].z,
                                            auxUnSerialize.pos_W[3].x,
                                            auxUnSerialize.pos_W[3].y,
                                            auxUnSerialize.pos_W[3].z
                                            );


    if(havePublisher)
    {


        polygon_msg.points[0].x = auxUnSerialize.pos_W[0].x;
        polygon_msg.points[0].y = auxUnSerialize.pos_W[0].y;
        polygon_msg.points[0].z = auxUnSerialize.pos_W[0].z;

        polygon_msg.points[1].x = auxUnSerialize.pos_W[1].x;
        polygon_msg.points[1].y = auxUnSerialize.pos_W[1].y;
        polygon_msg.points[1].z = auxUnSerialize.pos_W[1].z;

        polygon_msg.points[2].x = auxUnSerialize.pos_W[2].x;
        polygon_msg.points[2].y = auxUnSerialize.pos_W[2].y;
        polygon_msg.points[2].z = auxUnSerialize.pos_W[2].z;

        polygon_msg.points[3].x = auxUnSerialize.pos_W[3].x;
        polygon_msg.points[3].y = auxUnSerialize.pos_W[3].y;
        polygon_msg.points[3].z = auxUnSerialize.pos_W[3].z;

        posWheels_pub.publish(polygon_msg);

    }

    return 0;
}
