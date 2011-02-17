//#include "parameters.h"
#include "comStruc.h"


class transmision
{

public:
    transmision(int aux);
    ~transmision();

    int sendData(Struct_1 *a);
    int recvData(Struct_1 *a);

    int sendData(Struct_2 *b);
    int recvData(Struct_2 *b);

    int sendData(Struct_3 *c);
    int recvData(Struct_3 *c);

    int sendData(Joy *joy);
    int recvData(Joy *joy);

    int sendData(char* buffer);
    int recvData(char* buffer);

    int sendData(Twist *twist);
    int recvData(Twist *twist);

    int sendData(Odometry *odometry);
    int recvData(Odometry *odometry);

    int sock;
};
