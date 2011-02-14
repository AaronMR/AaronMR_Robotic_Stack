#include "AaronMR_S.hpp"
#include "pack2.hpp"
//-----------------------------------------------struct struct_Joy--------------------------------------

struct_Joy::struct_Joy()
{
    haveSubscriber = false;
    havePublisher = false;
    cout << "raro raro" << endl;
    sizeof_Joy = sizeof(Joy);

    auxJoy1.axes[0] = 0.0;
    auxJoy1.axes[1] = 0.0;
    auxJoy1.axes[2] = 0.0;
    auxJoy1.axes[3] = 0.0;
    auxJoy1.buttons[0] = 0.0;
    auxJoy1.buttons[1] = 0.0;
    auxJoy1.buttons[2] = 0.0;
    auxJoy1.buttons[3] = 0.0;

}

void struct_Joy::storeData(Joy *joy)
{
    return;
}

char *struct_Joy::serialize(char* buf3)
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
    auxJoy2.axes[0] = auxJoy1.axes[0];
    auxJoy2.axes[1] = auxJoy1.axes[1];
    auxJoy2.axes[2] = auxJoy1.axes[2];
    auxJoy2.axes[3] = auxJoy1.axes[3];
    auxJoy2.buttons[0] = auxJoy1.buttons[0];
    auxJoy2.buttons[1] = auxJoy1.buttons[1];
    auxJoy2.buttons[2] = auxJoy1.buttons[2];
    auxJoy2.buttons[3] = auxJoy1.buttons[3];

	packetsize = pack(buf, "CHffffhhhh",    'A',
                                            0,
                                            auxJoy2.axes[0],
                                            auxJoy2.axes[1],
                                            auxJoy2.axes[2],
                                            auxJoy2.axes[3],
                                            auxJoy2.buttons[0],
                                            auxJoy2.buttons[1],
                                            auxJoy2.buttons[2],
                                            auxJoy2.buttons[3]);

	packi16(buf+1, packetsize); // store packet size in packet for kicks

    memcpy((unsigned char*)buf3, buf, packetsize);

	unpack((unsigned char*)buf3, "CHffffhhhh",  &magic,
                                            &ps2,
                                            &auxJoy2.axes[0],
                                            &auxJoy2.axes[1],
                                            &auxJoy2.axes[2],
                                            &auxJoy2.axes[3],
                                            &auxJoy2.buttons[0],
                                            &auxJoy2.buttons[1],
                                            &auxJoy2.buttons[2],
                                            &auxJoy2.buttons[3]);

	printf("send: '%c' %hhu %f %f %f %f %d %d %d %d\n",   magic,
                                                    ps2,
                                                    auxJoy2.axes[0],
                                                    auxJoy2.axes[1],
                                                    auxJoy2.axes[2],
                                                    auxJoy2.axes[3],
                                                    auxJoy2.buttons[0],
                                                    auxJoy2.buttons[1],
                                                    auxJoy2.buttons[2],
                                                    auxJoy2.buttons[3]);

}

char *struct_Joy::Unserialize(char* buf3)
{

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


}
