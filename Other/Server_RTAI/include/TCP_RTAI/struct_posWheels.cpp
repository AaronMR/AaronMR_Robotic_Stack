#include "AaronMR_S.hpp"
#include "pack2.hpp"
#include <rtai_shm.h>
//############################################### struct_Twist ##############################

struct_posWheels::struct_posWheels()
{

}

void struct_posWheels::iniSHM(int shm_in, int shm_out, char* SHM_name)
{
    if (shm_in == 1)
    {
        dataIN = (posWheels_t*)rtai_malloc (nam2num(SHM_name), sizeof(struct posWheels_t)) ;
    }

    if (shm_out == 1)
    {
        dataOUT = (posWheels_t*)rtai_malloc (nam2num(SHM_name), sizeof(struct posWheels_t)) ;
    }
}

char *struct_posWheels::serialize(char* buf3)
{
	unsigned char buf[1024];
	unsigned char magic;
	unsigned int packetsize, ps2;

   /*
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
*/

}

char *struct_posWheels::Unserialize(char* buf3)
{
    /*
	unsigned char buf[1024];
	unsigned char magic;
	unsigned int ps2;

    memcpy(buf, buf3, 1024);



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
/*
    dataIN->angular.x = twist2.angular.x;
	dataIN->angular.y = twist2.angular.y;
    dataIN->angular.z = twist2.angular.z;
	dataIN->linear.x = twist2.linear.x;
    dataIN->linear.y = twist2.linear.y;
	dataIN->linear.z = twist2.linear.z;
    dataIN->newValue = true;
*/
    return NULL;
}
