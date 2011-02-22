
#include "AaronMR_S.hpp"
#include "pack2.hpp"

#include <stdio.h>
#include <math.h>
#include <rtai_shm.h>


struct_comStruc::struct_comStruc()
{
    sizeof_Joy = sizeof(Joy);

}


void struct_comStruc::iniSHM(int shm_in, int shm_out, char* SHM_name)
{
    if (shm_in == 1)
    {
        dataIN = (comStruc_IN*)rtai_malloc (nam2num(SHM_name), sizeof(struct comStruc_IN)) ;
        dataIN->x1 = 0.0;
        dataIN->y1 = 0.0;
        dataIN->x2 = 0.0;
        dataIN->y2 = 0.0;
    }

    if (shm_out == 1)
    {
        dataOUT = (comStruc_OUT*)rtai_malloc (nam2num(SHM_name), sizeof(struct comStruc_OUT)) ;
        dataOUT->position.x = 0.1;
        dataOUT->position.y = 0.1;
        dataOUT->position.z = 0.1;

        dataOUT->orientation.x = 0.1;
        dataOUT->orientation.y = 0.1;
        dataOUT->orientation.z = 0.1;
        dataOUT->orientation.w = 0.1;
    }
}

void struct_comStruc::storeData(Joy *joy)
{
    return;
}

char *struct_comStruc::serialize(char* buf3)
{
	unsigned char buf[1024];

	unsigned char magic;

	unsigned int packetsize, ps2;

    comStruc_OUT auxSerialize;

    auxSerialize.position.x = dataOUT->position.x;
    auxSerialize.position.y = dataOUT->position.y;
    auxSerialize.position.z = dataOUT->position.z;

    auxSerialize.orientation.x = dataOUT->orientation.x;
    auxSerialize.orientation.y = dataOUT->orientation.y;
    auxSerialize.orientation.z = dataOUT->orientation.z;
    auxSerialize.orientation.w = dataOUT->orientation.w;

	packetsize = pack(buf, "CHddddddd",    'A',
                                            0,
                                            auxSerialize.position.x,
                                            auxSerialize.position.y,
                                            auxSerialize.position.z,
                                            auxSerialize.orientation.x,
                                            auxSerialize.orientation.y,
                                            auxSerialize.orientation.z,
                                            auxSerialize.orientation.w
                                            );

	packi16(buf+1, packetsize); // store packet size in packet for kicks

    memcpy((unsigned char*)buf3, buf, packetsize);

	unpack((unsigned char*)buf3, "CHddddddd",  &magic,
                                            &ps2,
                                            &auxSerialize.position.x,
                                            &auxSerialize.position.y,
                                            &auxSerialize.position.z,
                                            &auxSerialize.orientation.x,
                                            &auxSerialize.orientation.y,
                                            &auxSerialize.orientation.z,
                                            &auxSerialize.orientation.w
                                            );

	printf("comStruc - send: '%c' %hhu %f %f %f %f %f %f %f \n",   magic,
                                            ps2,
                                            auxSerialize.position.x,
                                            auxSerialize.position.y,
                                            auxSerialize.position.z,
                                            auxSerialize.orientation.x,
                                            auxSerialize.orientation.y,
                                            auxSerialize.orientation.z,
                                            auxSerialize.orientation.w
                                            );

}

char *struct_comStruc::Unserialize(char* buf3)
{
/*
	unsigned char buf[1024];
//	unsigned char buf2[1024]="makiboludo";
	unsigned char magic;
//	int monkeycount;
//	long altitude;
//	double absurdityfactor;
//	char *s = "Maki";
//	char s2[96];
//	unsigned int packetsize, ps2;
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



//###################################################################


    dataIN->axes[0] = auxJoy2.axes[0];
	dataIN->axes[1]= auxJoy2.axes[1];
    dataIN->axes[2]= auxJoy2.axes[2];
	dataIN->axes[3]= auxJoy2.axes[3];
    dataIN->buttons[0]= auxJoy2.buttons[0];
	dataIN->buttons[1] = auxJoy2.buttons[1];
    dataIN->buttons[2] = auxJoy2.buttons[2];
	dataIN->buttons[3] = auxJoy2.buttons[3];

	dataIN->newValue = true;


//###################################################################
*/
}
