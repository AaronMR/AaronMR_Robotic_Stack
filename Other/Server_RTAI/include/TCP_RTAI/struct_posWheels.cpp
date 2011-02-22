#include "AaronMR_S.hpp"
#include "pack2.hpp"
#include <rtai_shm.h>

struct_posWheels::struct_posWheels()
{
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

void struct_posWheels::iniSHM(int shm_in, int shm_out, char* SHM_name)
{
    if (shm_in == 1)
    {
        dataIN = (posWheels_t*)rtai_malloc (nam2num(SHM_name), sizeof(struct posWheels_t)) ;

        dataIN->pos_W[0].x = 0.0;
        dataIN->pos_W[0].y = 0.0;
        dataIN->pos_W[0].z = 0.0;

        dataIN->pos_W[1].x = 0.0;
        dataIN->pos_W[1].y = 0.0;
        dataIN->pos_W[1].z = 0.0;

        dataIN->pos_W[2].x = 0.0;
        dataIN->pos_W[2].y = 0.0;
        dataIN->pos_W[2].z = 0.0;

        dataIN->pos_W[3].x = 0.0;
        dataIN->pos_W[3].y = 0.0;
        dataIN->pos_W[3].z = 0.0;
    }

    if (shm_out == 1)
    {
        dataOUT = (posWheels_t*)rtai_malloc (nam2num(SHM_name), sizeof(struct posWheels_t)) ;

        dataOUT->pos_W[0].x = 0.0;
        dataOUT->pos_W[0].y = 0.0;
        dataOUT->pos_W[0].z = 0.0;

        dataOUT->pos_W[1].x = 0.0;
        dataOUT->pos_W[1].y = 0.0;
        dataOUT->pos_W[1].z = 0.0;

        dataOUT->pos_W[2].x = 0.0;
        dataOUT->pos_W[2].y = 0.0;
        dataOUT->pos_W[2].z = 0.0;

        dataOUT->pos_W[3].x = 0.0;
        dataOUT->pos_W[3].y = 0.0;
        dataOUT->pos_W[3].z = 0.0;
    }
}

char *struct_posWheels::serialize(char* buf3)
{
	unsigned char buf[1024];
	unsigned char magic;
	unsigned int packetsize, ps2;

    auxSerialize.pos_W[0].x = dataOUT->pos_W[0].x;
    auxSerialize.pos_W[0].y = dataOUT->pos_W[0].y;
    auxSerialize.pos_W[0].z = dataOUT->pos_W[0].z;

    auxSerialize.pos_W[1].x = dataOUT->pos_W[1].x;
    auxSerialize.pos_W[1].y = dataOUT->pos_W[1].y;
    auxSerialize.pos_W[1].z = dataOUT->pos_W[1].z;

    auxSerialize.pos_W[2].x = dataOUT->pos_W[2].x;
    auxSerialize.pos_W[2].y = dataOUT->pos_W[2].y;
    auxSerialize.pos_W[2].z = dataOUT->pos_W[2].z;

    auxSerialize.pos_W[3].x = dataOUT->pos_W[3].x;
    auxSerialize.pos_W[3].y = dataOUT->pos_W[3].y;
    auxSerialize.pos_W[3].z = dataOUT->pos_W[3].z;

    packetsize = pack(buf, "CHdddddddddddd", 'A',
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
                                        auxSerialize.pos_W[3].z
                                        );

    packi16(buf+1, packetsize); // store packet size in packet for kicks

    memcpy((unsigned char*)buf3, buf, packetsize);

	unpack((unsigned char*)buf3, "CHdddddddddddd",
                                            &magic,
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
                                            &auxSerialize.pos_W[3].z
                                            );

	printf("posWheels - send: '%c' %hhu %f %f %f %f %f %f %f %f %f %f %f %f\n",
                                            magic,
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
                                            auxSerialize.pos_W[3].z
                                            );

}

char *struct_posWheels::Unserialize(char* buf3)
{

    unsigned char buf[1024];
	unsigned char magic;
	unsigned int ps2;

    memcpy(buf, buf3, 1024);

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

    dataIN->pos_W[0].x = auxUnSerialize.pos_W[0].x;
    dataIN->pos_W[0].y = auxUnSerialize.pos_W[0].y;
    dataIN->pos_W[0].z = auxUnSerialize.pos_W[0].z;

    dataIN->pos_W[1].x = auxUnSerialize.pos_W[1].x;
    dataIN->pos_W[1].y = auxUnSerialize.pos_W[1].y;
    dataIN->pos_W[1].z = auxUnSerialize.pos_W[1].z;

    dataIN->pos_W[2].x = auxUnSerialize.pos_W[2].x;
    dataIN->pos_W[2].y = auxUnSerialize.pos_W[2].y;
    dataIN->pos_W[2].z = auxUnSerialize.pos_W[2].z;

    dataIN->pos_W[3].x = auxUnSerialize.pos_W[3].x;
    dataIN->pos_W[3].y = auxUnSerialize.pos_W[3].y;
    dataIN->pos_W[3].z = auxUnSerialize.pos_W[3].z;

    return NULL;
}
