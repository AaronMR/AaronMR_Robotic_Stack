
#include "AaronMR_S.hpp"
#include "pack2.hpp"
#include <rtai_shm.h>
//############################################### struct_Pose ##############################

struct_Pose::struct_Pose()
{
    haveSubscriber = false;
    havePublisher = false;
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



void struct_Pose::iniSHM(int shm_in, int shm_out)
{
    if (shm_in == 1)
    {
        dataIN = (Pose*)rtai_malloc (nam2num(SHMNAM_IN), sizeof(struct Pose)) ;
        dataIN->position.x = 0.0;
        dataIN->position.y = 0.0;
        dataIN->position.z = 0.0;

        dataIN->orientation.x = 0.0;
        dataIN->orientation.y = 0.0;
        dataIN->orientation.z = 0.0;
        dataIN->orientation.w = 0.0;
        dataIN->newValue = 0;

    }


    if (shm_out == 1)
    {
        dataOUT = (Pose*)rtai_malloc (nam2num(SHMNAM_OUT), sizeof(struct Pose)) ;
        dataOUT->position.x = 0.0;
        dataOUT->position.y = 0.0;
        dataOUT->position.z = 0.0;

        dataOUT->orientation.x = 0.0;
        dataOUT->orientation.y = 0.0;
        dataOUT->orientation.z = 0.0;
        dataOUT->orientation.w = 0.0;
    }

}

void struct_Pose::storeData(Joy *joy)
{
    return;
}

char *struct_Pose::serialize(char* buf3)
{
	unsigned char buf[1024];
	unsigned char magic;
	unsigned int packetsize, ps2;

    Pose pose;

    pose.position.x = dataOUT->position.x;
    pose.position.y = dataOUT->position.y;
    pose.position.z = dataOUT->position.z;

    pose.orientation.x = dataOUT->orientation.x;
    pose.orientation.y = dataOUT->orientation.y;
    pose.orientation.z = dataOUT->orientation.z;
    pose.orientation.w = dataOUT->orientation.w;

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

char *struct_Pose::Unserialize(char* buf3)
{
	unsigned char buf[1024];
	unsigned char magic;
	unsigned int ps2;


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

    dataOUT->position.x = pose.position.x;
    dataOUT->position.x = pose.position.y;
    dataOUT->position.x = pose.position.z;

    dataOUT->orientation.x = pose.orientation.x;
    dataOUT->orientation.x = pose.orientation.y;
    dataOUT->orientation.x = pose.orientation.z;
    dataOUT->orientation.x = pose.orientation.w;

    return NULL;
}
