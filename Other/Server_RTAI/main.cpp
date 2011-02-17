#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <netinet/in.h>
#include <resolv.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <pthread.h>

#include "transmition.h"
//#include "parameters.h"

#include "AaronMR_S.hpp"

DataLayout process[5];
int numProcess;

int main(int argv, char** argc){

    AaronMR_S test(argc[1]);

    while(1)
        test.waitEvent();

    return 0;


FINISH:
;
}

