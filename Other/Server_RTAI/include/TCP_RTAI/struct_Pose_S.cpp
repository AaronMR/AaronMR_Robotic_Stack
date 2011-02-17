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

void struct_Pose::storeData(Joy *joy)
{
    return;
}

char *struct_Pose::serialize(char* buf3)
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

    Joy auxJoy2;
    Twist twist;

    twist.angular.x = 1.1;
    twist.angular.y = 2.2;
    twist.angular.z = 3.3;

    twist.linear.x = 4.4;
    twist.linear.y = 5.5;
    twist.linear.z = 6.6;

    Pose pose;

    pose.position.x = 1.0;
    pose.position.y = 2.0;
    pose.position.z = 3.0;

    pose.orientation.x = 4.0;
    pose.orientation.y = 5.0;
    pose.orientation.z = 6.0;
    pose.orientation.w = 7.0;



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
    /*
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
    */

    return NULL;
}
