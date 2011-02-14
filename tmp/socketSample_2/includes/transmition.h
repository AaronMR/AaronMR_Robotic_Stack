#include "parameters.h"

class transmision
{
public:
    transmision(int aux);
    ~transmision();
    int sendData(Struct_1 *a);
    int recvData(Struct_1 *a);

    int sendData(Struct_2 *b);
    int recvData(Struct_2 *b);

    int sock;
};
