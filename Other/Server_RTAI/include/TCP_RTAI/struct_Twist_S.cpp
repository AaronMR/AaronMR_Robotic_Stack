#include "AaronMR_S.hpp"
#include "pack2.hpp"
#include <rtai_shm.h>
//############################################### struct_Twist ##############################

struct_Twist::struct_Twist()
{
    haveSubscriber = false;
    havePublisher = false;
    cout << "raro raro" << endl;
    sizeof_Joy = sizeof(Joy);


    //dataIN = (Twist*)rtai_malloc (nam2num(SHMNAM_IN), sizeof(struct Twist)) ;
    //dataOUT = (Twist*)rtai_malloc (nam2num(SHMNAM_OUT), sizeof(struct Pose)) ;
    pause = 100000;
    t = 0;


}

void struct_Twist::iniSHM(int shm_in, int shm_out)
{
    if (shm_in == 1)
        dataIN = (Twist*)rtai_malloc (nam2num(SHMNAM_IN), sizeof(struct Twist)) ;

    if (shm_out == 1)
        dataOUT = (Twist*)rtai_malloc (nam2num(SHMNAM_OUT), sizeof(struct Pose)) ;
}


void struct_Twist::storeData(Joy *joy)
{
    return;
}

char *struct_Twist::serialize(char* buf3)
{
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


    Twist twist;

    twist.angular.x = dataOUT->angular.x;
    twist.angular.y = dataOUT->angular.y;
    twist.angular.z = dataOUT->angular.z;

    twist.linear.x = dataOUT->linear.x;
    twist.linear.y = dataOUT->linear.y;
    twist.linear.z = dataOUT->linear.z;



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

char *struct_Twist::Unserialize(char* buf3)
{
	unsigned char buf[1024];
	//unsigned char buf2[1024]="makiboludo";
	unsigned char magic;
	//int monkeycount;
	//long altitude;
	//double absurdityfactor;
	//char *s = "Maki";
	//char s2[96];
	//unsigned int packetsize, ps2;
	unsigned int ps2;
    //double maki[9];

    memcpy(buf, buf3, 1024);

    Twist twist2;

	unpack((unsigned char*)buf, "CHdddddd",  &magic,
                                            &ps2,
                                            &twist2.linear.x,
                                            &twist2.linear.y,
                                            &twist2.linear.z,
                                            &twist2.angular.x,
                                            &twist2.angular.y,
                                            &twist2.angular.z);

	printf("recv: '%c' %hhu %f %f %f %f %f %f\n",   magic,
                                                    ps2,
                                                    twist2.linear.x,
                                                    twist2.linear.y,
                                                    twist2.linear.z,
                                                    twist2.angular.x,
                                                    twist2.angular.y,
                                                    twist2.angular.z);

//###################################################################

    dataIN->angular.x = twist2.angular.x;
	dataIN->angular.y = twist2.angular.y;
    dataIN->angular.z = twist2.angular.z;
	dataIN->linear.x = twist2.linear.x;
    dataIN->linear.y = twist2.linear.y;
	dataIN->linear.z = twist2.linear.z;
    dataIN->newValue = true;



//###################################################################

    return NULL;
}
