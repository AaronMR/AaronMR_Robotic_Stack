#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <resolv.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>


#include <cstdlib>
#include "AaronMR_C.hpp"

using namespace std;

int exitClient(int hsock);

DataLayout process[5];

turtlesim::Velocity vel;

Struct_3 auxJoy;


//###########################################################################################







//-----------------------------------MAIN---------------------------
int main(int argv, char** argc)
{


    //main2();
   // ini ROS
    //ros::init(argv, argc, "RTAI_node");

//    char * file = "/home/aaronmr/ROS/AaronMR_Robotic_Stack/Drivers/CB_TCP_RTAI/include/TCP_RTAI/process_3.cfg";


    //AaronMR_C test(file);
    //char *nameprocess = "mierda1";
/*
    int aux = 0;
    while(1)
    {
        cout << "aux = "<< aux << endl;
        aux++;
        usleep(100000);
        system("clear");
    }
*/

    ros::init(argv, argc, argc[2]);

    AaronMR_C test(argc[1]);



    test.create_Socket();


    if(test.connect_Socket()== -1)
    {
        printf("The connection error\n");
        return -1;
    }

    while(1)
    {

        //test.SendRecv();


        usleep(10000);

        //cout << "Send and Recv"<< endl;
        if(test.send_Data() == -1)
        {
            printf("The connection error\n");
            return -1;
        }


        if(test.recv_Data() == -1)
        {
            printf("The connection error\n");
            return -1;
        }

        printf("\n");

        //system("clear");

    }



    return 0;

}


