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
#include "parameters.h"

#include "AaronMR_S.hpp"

//void* SocketHandler(void*);

DataLayout process[5];
int numProcess;

int main(int argv, char** argc){

    char * file = "/home/aaronmr/ROS/AaronMR_Robotic_Stack/Drivers/CB_TCP_RTAI/include/TCP_RTAI/ServerFile.cfg";
    AaronMR_S test(argc[1]);

    while(1)
        test.waitEvent();
        //test.acceptConnection();


    return 0;


FINISH:
;
}

/*
void* SocketHandler(void* lp){


    struct DataLayout *temp = (struct DataLayout *)lp;
    struct DataLayout processThread_2;
    processThread_2.active = temp->active;

    processThread_2.name = temp->name;
    processThread_2.IP_RTAI = temp->IP_RTAI;
    processThread_2.Node2RTAI = temp->Node2RTAI;
    processThread_2.PORT_RTAI = temp->PORT_RTAI;
    processThread_2.RTAI2Node = temp->RTAI2Node;
    processThread_2.SHM = temp->SHM;
    processThread_2.active = temp->active;
    processThread_2.csock = temp->csock;

    int csock = processThread_2.csock;
    int counter = 0;
    //int processFind = 0;


    transmision socketRTAI(csock);


    Struct_1 a;
    Struct_2 b;
    Struct_3 c;
    Twist twist;
    Odometry odometry;
    //Joy joy;

    a.indx_counter = counter;
    a.cos_value = rand();
    a.sin_value = rand();

    b.indx_counter = -69;
    b.cos_value = -69;
    b.sin_value = -69;

    c.axes[0]=0;
    c.axes[1]=0;
    c.axes[2]=0;
    c.axes[3]=0;

    c.buttons[0]=0;
    c.buttons[1]=0;
    c.buttons[2]=0;
    c.buttons[3]=0;

//    odometry.child_frame_id = "test";

    odometry.pose.covariance[0] = 0;
    odometry.pose.pose.orientation.w = 0;
    odometry.pose.pose.orientation.x = 0;
    odometry.pose.pose.orientation.y = 0;
    odometry.pose.pose.orientation.z = 0;

    odometry.pose.pose.position.x = 0;
    odometry.pose.pose.position.y = 0;
    odometry.pose.pose.position.z = 0;

    counter = counter + 1;

    cout << "name = " << processThread_2.name << endl
        << "IP_RTAI = " << processThread_2.IP_RTAI << endl
        << "Node2RTAI = " << processThread_2.Node2RTAI << endl
        << "PORT_RTAI = " << processThread_2.PORT_RTAI << endl
        << "RTAI2Node = " << processThread_2.RTAI2Node << endl
        << "SHM = " << processThread_2.SHM << endl
        << " = " << processThread_2.active << endl
        << " = " << processThread_2.csock << endl ;


    //DataLayout processThread;

	char buffer[1024];
	int buffer_len = 1024;
	int bytecount = 0;
    memset(buffer, 0, buffer_len);

    cout << "esperando identificacion del proceso" << endl;


    printf("Received bytes %d\nReceived string \"%s\"\n", bytecount, buffer);

    //----------------------------------------------------------------


    while(1)
    {

        cout << "--- Process = " << processThread_2.name << "---" << endl;



        int Node2RTAI = 0;
        if(processThread_2.Node2RTAI == "Struct_1")
        {
            Node2RTAI = 1;
        }else if(processThread_2.Node2RTAI == "Struct_2")
        {
            Node2RTAI = 2;
        }else if(processThread_2.Node2RTAI == "Struct_3")
        {
            Node2RTAI = 3;
        }else if(processThread_2.Node2RTAI == "Twist")
        {
            Node2RTAI = 4;
        }else if(processThread_2.Node2RTAI == "Odometry")
        {
            Node2RTAI = 5;
        }else if(processThread_2.Node2RTAI == "Joy")
        {
            Node2RTAI = 6;
        }

        cout <<"Node2RTAI = " << Node2RTAI << endl;
        char buffer2[1024];
        switch(Node2RTAI)
        {
            case 1:
                if(socketRTAI.recvData(&a) == -1){
                    fprintf(stderr, "Error receiving data %d\n", errno);
                    goto FINISH;
                }
                break;

            case 2:
                if(socketRTAI.recvData(&b) == -1){
                    fprintf(stderr, "Error receiving data %d\n", errno);
                    goto FINISH;
                }
                break;

            case 3:
                if(socketRTAI.recvData(&c) == -1){
                    fprintf(stderr, "Error receiving data %d\n", errno);
                    goto FINISH;
                }
                break;

            case 4:
                if(socketRTAI.recvData(&twist) == -1){
                    fprintf(stderr, "Error receiving data %d\n", errno);
                    goto FINISH;
                }
                break;

            case 5:
                if(socketRTAI.recvData(&odometry) == -1){
                    fprintf(stderr, "Error receiving data %d\n", errno);
                    goto FINISH;
                }
                break;

            case 6:
                if(socketRTAI.recvData(buffer2) == -1){
                    fprintf(stderr, "Error receiving data %d\n", errno);
                    goto FINISH;
                }
                break;


            default:
                cout << "Struct no valid...." << endl;
        }


        //-------------------------------------------------------------------------------

        a.indx_counter = a.indx_counter * (-1);
        a.cos_value = a.cos_value * (-1);
        a.sin_value = a.sin_value * (-1);


        b.indx_counter = b.indx_counter * (-1);
        b.cos_value = b.cos_value * (-1);
        b.sin_value = b.sin_value * (-1);

        c.axes[0]=0;
        c.axes[1]=0;
        c.axes[2]=0;
        c.axes[3]=0;

        c.buttons[0]=0;
        c.buttons[1]=0;
        c.buttons[2]=0;
        c.buttons[3]=0;

        int RTAI2Node = 0;
        if(processThread_2.RTAI2Node == "Struct_1")
        {
            RTAI2Node = 1;
        }else if(processThread_2.RTAI2Node == "Struct_2")
        {
            RTAI2Node = 2;
        }else if(processThread_2.RTAI2Node == "Struct_3")
        {
            RTAI2Node = 3;
        }else if(processThread_2.RTAI2Node == "Twist")
        {
            RTAI2Node = 4;
        }else if(processThread_2.Node2RTAI == "Odometry")
        {
            RTAI2Node = 5;
        }else if(processThread_2.Node2RTAI == "Joy")
        {
            RTAI2Node = 6;
        }

        switch(RTAI2Node)
        {
            case 1:
                if(socketRTAI.sendData(&a) == -1){
                    fprintf(stderr, "Error receiving data %d\n", errno);
                    goto FINISH;
                }
                break;

            case 2:
                if(socketRTAI.sendData(&b) == -1){
                    fprintf(stderr, "Error receiving data %d\n", errno);
                    goto FINISH;
                }
                break;

            case 3:
                if(socketRTAI.sendData(&c) == -1){
                    fprintf(stderr, "Error receiving data %d\n", errno);
                    goto FINISH;
                }
                break;

            case 4:
                if(socketRTAI.sendData(&twist) == -1){
                    fprintf(stderr, "Error receiving data %d\n", errno);
                    goto FINISH;
                }
                break;

            case 5:
                if(socketRTAI.sendData(&odometry) == -1){
                    fprintf(stderr, "Error receiving data %d\n", errno);
                    goto FINISH;
                }
                break;

            case 6:
                //if(socketRTAI.sendData(&joy) == -1){
                if(socketRTAI.sendData(buffer2) == -1){
                    fprintf(stderr, "Error receiving data %d\n", errno);
                    goto FINISH;
                }
                break;
            default:
                cout << "struct no valid" << endl;
        }

       usleep(500000);
    }


FINISH:

	cout << "Close thread and connecion tcp" << endl;
    return 0;
}
*/

/*
void* SocketHandler(void* lp){
    int *csock = (int*)lp;

    char buffer[1024];
    int buffer_len = 1024;
    int bytecount;

    memset(buffer, 0, buffer_len);

    while(1)
    {

        if((bytecount = recv(*csock, buffer, buffer_len, 0)) < 1){
            fprintf(stderr, "Error receiving data %d\n", errno);
            goto FINISH;
        }
        printf("Received bytes %d\nReceived string \"%s\"\n", bytecount, buffer);
        strcat(buffer, " SERVER ECHO");

        if((bytecount = send(*csock, buffer, strlen(buffer), 0)) < 1){
            fprintf(stderr, "Error sending data %d\n", errno);
            goto FINISH;
        }

        printf("Sent bytes %d\n", bytecount);
    }


FINISH:
    //free(&csock);

    return 0;


}
*/
