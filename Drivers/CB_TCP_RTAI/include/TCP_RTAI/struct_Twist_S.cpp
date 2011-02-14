#include "AaronMR_S.hpp"
#include "pack2.hpp"
//############################################### struct_Twist ##############################

struct_Twist::struct_Twist()
{
    haveSubscriber = false;
    havePublisher = false;
    cout << "raro raro" << endl;
    sizeof_Joy = sizeof(Joy);
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

    Joy auxJoy2;
    Twist twist;

    twist.angular.x = 1.1;
    twist.angular.y = 2.2;
    twist.angular.z = 3.3;

    twist.linear.x = 4.4;
    twist.linear.y = 5.5;
    twist.linear.z = 6.6;

    auxJoy2.axes[0] = auxJoy1.axes[0];
    auxJoy2.axes[1] = auxJoy1.axes[1];
    auxJoy2.axes[2] = auxJoy1.axes[2];
    auxJoy2.axes[3] = auxJoy1.axes[3];

    auxJoy2.buttons[0] = auxJoy1.buttons[0];
    auxJoy2.buttons[1] = auxJoy1.buttons[1];
    auxJoy2.buttons[2] = auxJoy1.buttons[2];
    auxJoy2.buttons[3] = auxJoy1.buttons[3];

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

    return NULL;
}
