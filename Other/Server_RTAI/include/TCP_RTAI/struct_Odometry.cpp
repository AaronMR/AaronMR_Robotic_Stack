
#include "AaronMR_S.hpp"
#include "pack2.hpp"
#include <rtai_shm.h>

struct_Odometry::struct_Odometry()
{
    //Initialitation auxSerialize variable
    auxSerialize.pose.pose.position.x = 0.0;
	auxSerialize.pose.pose.position.y = 0.0;
	auxSerialize.pose.pose.position.z = 0.0;

	auxSerialize.pose.pose.orientation.x = 0.0;
	auxSerialize.pose.pose.orientation.y = 0.0;
	auxSerialize.pose.pose.orientation.z = 0.0;
	auxSerialize.pose.pose.orientation.w = 0.0;

	auxSerialize.twist.twist.angular.x = 0.0;
	auxSerialize.twist.twist.angular.y = 0.0;
	auxSerialize.twist.twist.angular.z = 0.0;

	auxSerialize.twist.twist.linear.x = 0.0;
	auxSerialize.twist.twist.linear.y = 0.0;
	auxSerialize.twist.twist.linear.z = 0.0;

	//Initialitation auxUnSerialize variable
    auxUnSerialize.pose.pose.position.x = 0.0;
	auxUnSerialize.pose.pose.position.y = 0.0;
	auxUnSerialize.pose.pose.position.z = 0.0;

	auxUnSerialize.pose.pose.orientation.x = 0.0;
	auxUnSerialize.pose.pose.orientation.y = 0.0;
	auxUnSerialize.pose.pose.orientation.z = 0.0;
	auxUnSerialize.pose.pose.orientation.w = 0.0;

	auxUnSerialize.twist.twist.angular.x = 0.0;
	auxUnSerialize.twist.twist.angular.y = 0.0;
	auxUnSerialize.twist.twist.angular.z = 0.0;

	auxUnSerialize.twist.twist.linear.x = 0.0;
	auxUnSerialize.twist.twist.linear.y = 0.0;
	auxUnSerialize.twist.twist.linear.z = 0.0;
}

void struct_Odometry::iniSHM(int shm_in, int shm_out, char* SHM_name)
{
    if (shm_in == 1)
    {
        dataIN = (odometry_t*)rtai_malloc (nam2num(SHM_name), sizeof(struct odometry_t)) ;

        dataIN->pose.pose.position.x = 0.0;
        dataIN->pose.pose.position.y = 0.0;
        dataIN->pose.pose.position.z = 0.0;

        dataIN->pose.pose.orientation.x = 0.0;
        dataIN->pose.pose.orientation.y = 0.0;
        dataIN->pose.pose.orientation.z = 0.0;
        dataIN->pose.pose.orientation.w = 0.0;

        dataIN->twist.twist.angular.x = 0.0;
        dataIN->twist.twist.angular.y = 0.0;
        dataIN->twist.twist.angular.z = 0.0;

        dataIN->twist.twist.linear.x = 0.0;
        dataIN->twist.twist.linear.y = 0.0;
        dataIN->twist.twist.linear.z = 0.0;

    }

    if (shm_out == 1)
    {
        dataOUT = (odometry_t*)rtai_malloc (nam2num(SHM_name), sizeof(struct odometry_t)) ;

        dataOUT->pose.pose.position.x = 0.0;
        dataOUT->pose.pose.position.y = 0.0;
        dataOUT->pose.pose.position.z = 0.0;

        dataOUT->pose.pose.orientation.x = 0.0;
        dataOUT->pose.pose.orientation.y = 0.0;
        dataOUT->pose.pose.orientation.z = 0.0;
        dataOUT->pose.pose.orientation.w = 0.0;

        dataOUT->twist.twist.angular.x = 0.0;
        dataOUT->twist.twist.angular.y = 0.0;
        dataOUT->twist.twist.angular.z = 0.0;

        dataOUT->twist.twist.linear.x = 0.0;
        dataOUT->twist.twist.linear.y = 0.0;
        dataOUT->twist.twist.linear.z = 0.0;

    }
}

char *struct_Odometry::serialize(char* buf3)
{

	unsigned char buf[1024];
	unsigned char magic;
	unsigned int packetsize, ps2;

    auxSerialize.pose.pose.position.x = dataOUT->pose.pose.position.x;
	auxSerialize.pose.pose.position.y = dataOUT->pose.pose.position.y;
	auxSerialize.pose.pose.position.z = dataOUT->pose.pose.position.z;

	auxSerialize.pose.pose.orientation.x = dataOUT->pose.pose.orientation.x;
	auxSerialize.pose.pose.orientation.y = dataOUT->pose.pose.orientation.y;
	auxSerialize.pose.pose.orientation.z = dataOUT->pose.pose.orientation.z;
	auxSerialize.pose.pose.orientation.w = dataOUT->pose.pose.orientation.w;

	auxSerialize.twist.twist.angular.x = dataOUT->twist.twist.angular.x;
	auxSerialize.twist.twist.angular.y = dataOUT->twist.twist.angular.x;
	auxSerialize.twist.twist.angular.z = dataOUT->twist.twist.angular.x;

	auxSerialize.twist.twist.linear.x = dataOUT->twist.twist.linear.x;
	auxSerialize.twist.twist.linear.y = dataOUT->twist.twist.linear.y;
	auxSerialize.twist.twist.linear.z = dataOUT->twist.twist.linear.z;

    packetsize = pack(buf, "CHddddddddddddd", 'A',
                                        0,
                                        auxSerialize.pose.pose.position.x,
                                        auxSerialize.pose.pose.position.y,
                                        auxSerialize.pose.pose.position.z,
                                        auxSerialize.pose.pose.orientation.x,
                                        auxSerialize.pose.pose.orientation.y,
                                        auxSerialize.pose.pose.orientation.z,
                                        auxSerialize.pose.pose.orientation.w,
                                        auxSerialize.twist.twist.angular.x,
                                        auxSerialize.twist.twist.angular.y,
                                        auxSerialize.twist.twist.angular.z,
                                        auxSerialize.twist.twist.linear.x,
                                        auxSerialize.twist.twist.linear.y,
                                        auxSerialize.twist.twist.linear.z
                                        );

    packi16(buf+1, packetsize); // store packet size in packet for kicks

    memcpy((unsigned char*)buf3, buf, packetsize);

	unpack((unsigned char*)buf3, "CHddddddddddddd",
                                        &magic,
                                        &ps2,
                                        &auxSerialize.pose.pose.position.x,
                                        &auxSerialize.pose.pose.position.y,
                                        &auxSerialize.pose.pose.position.z,
                                        &auxSerialize.pose.pose.orientation.x,
                                        &auxSerialize.pose.pose.orientation.y,
                                        &auxSerialize.pose.pose.orientation.z,
                                        &auxSerialize.pose.pose.orientation.w,
                                        &auxSerialize.twist.twist.angular.x,
                                        &auxSerialize.twist.twist.angular.y,
                                        &auxSerialize.twist.twist.angular.z,
                                        &auxSerialize.twist.twist.linear.x,
                                        &auxSerialize.twist.twist.linear.y,
                                        &auxSerialize.twist.twist.linear.z
                                        );

	printf("posWheels - send: '%c' %hhu %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
                                        magic,
                                        ps2,
                                        auxSerialize.pose.pose.position.x,
                                        auxSerialize.pose.pose.position.y,
                                        auxSerialize.pose.pose.position.z,
                                        auxSerialize.pose.pose.orientation.x,
                                        auxSerialize.pose.pose.orientation.y,
                                        auxSerialize.pose.pose.orientation.z,
                                        auxSerialize.pose.pose.orientation.w,
                                        auxSerialize.twist.twist.angular.x,
                                        auxSerialize.twist.twist.angular.y,
                                        auxSerialize.twist.twist.angular.z,
                                        auxSerialize.twist.twist.linear.x,
                                        auxSerialize.twist.twist.linear.y,
                                        auxSerialize.twist.twist.linear.z
                                            );

}

char *struct_Odometry::Unserialize(char* buf3)
{

    unsigned char buf[1024];
	unsigned char magic;
	unsigned int ps2;

    memcpy(buf, buf3, 1024);

	unpack((unsigned char*)buf, "CHddddddddddddd",
                                        &magic,
                                        &ps2,
                                        &auxUnSerialize.pose.pose.position.x,
                                        &auxUnSerialize.pose.pose.position.y,
                                        &auxUnSerialize.pose.pose.position.z,
                                        &auxUnSerialize.pose.pose.orientation.x,
                                        &auxUnSerialize.pose.pose.orientation.y,
                                        &auxUnSerialize.pose.pose.orientation.z,
                                        &auxUnSerialize.pose.pose.orientation.w,
                                        &auxUnSerialize.twist.twist.angular.x,
                                        &auxUnSerialize.twist.twist.angular.y,
                                        &auxUnSerialize.twist.twist.angular.z,
                                        &auxUnSerialize.twist.twist.linear.x,
                                        &auxUnSerialize.twist.twist.linear.y,
                                        &auxUnSerialize.twist.twist.linear.z
                                            );

	printf("posWheels - recv: '%c' %hhu %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
                                        magic,
                                        ps2,
                                        auxUnSerialize.pose.pose.position.x,
                                        auxUnSerialize.pose.pose.position.y,
                                        auxUnSerialize.pose.pose.position.z,
                                        auxUnSerialize.pose.pose.orientation.x,
                                        auxUnSerialize.pose.pose.orientation.y,
                                        auxUnSerialize.pose.pose.orientation.z,
                                        auxUnSerialize.pose.pose.orientation.w,
                                        auxUnSerialize.twist.twist.angular.x,
                                        auxUnSerialize.twist.twist.angular.y,
                                        auxUnSerialize.twist.twist.angular.z,
                                        auxUnSerialize.twist.twist.linear.x,
                                        auxUnSerialize.twist.twist.linear.y,
                                        auxUnSerialize.twist.twist.linear.z
                                        );


    dataIN->pose.pose.position.x = auxUnSerialize.pose.pose.position.x;
    dataIN->pose.pose.position.y = auxUnSerialize.pose.pose.position.y;
    dataIN->pose.pose.position.z = auxUnSerialize.pose.pose.position.z;

    dataIN->pose.pose.orientation.x = auxUnSerialize.pose.pose.orientation.x;
    dataIN->pose.pose.orientation.y = auxUnSerialize.pose.pose.orientation.y;
    dataIN->pose.pose.orientation.z = auxUnSerialize.pose.pose.orientation.z;
    dataIN->pose.pose.orientation.w = auxUnSerialize.pose.pose.orientation.w;

    dataIN->twist.twist.angular.x = auxUnSerialize.twist.twist.angular.x;
    dataIN->twist.twist.angular.y = auxUnSerialize.twist.twist.angular.y;
    dataIN->twist.twist.angular.z = auxUnSerialize.twist.twist.angular.z;

    dataIN->twist.twist.linear.x = auxUnSerialize.twist.twist.linear.x;
    dataIN->twist.twist.linear.y = auxUnSerialize.twist.twist.linear.y;
    dataIN->twist.twist.linear.z = auxUnSerialize.twist.twist.linear.z;

    return NULL;

}
