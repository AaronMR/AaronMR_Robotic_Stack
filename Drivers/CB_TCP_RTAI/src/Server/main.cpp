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
#include "parser.h"
//#include "parameters.h"


#include "transmition.h"



void* SocketHandler(void*);

DataLayout process[5];
int numProcess;

int main(int argv, char** argc){

    numProcess = 0;

    // load configuration file
	parseFile parser(argc[1]);
	parser.Parse(process);


    // numero de procesos configurados en el fichero
    numProcess = parser.getNumProcess();
    if(numProcess == 0)
    {
        cout << "No hay ningun proceso configurado" << endl;
    }else{
        cout << "Procesos configurados = " << parser.getNumProcess() <<endl;
    }

	int host_port= 1101;

	struct sockaddr_in my_addr;

	int hsock;
	int * p_int ;
	int err;

	socklen_t addr_size = 0;
	int csock;
	sockaddr_in sadr;
	pthread_t thread_id=0;


	hsock = socket(AF_INET, SOCK_STREAM, 0);
	if(hsock == -1){
		printf("Error initializing socket %d\n", errno);
		goto FINISH;
	}

	p_int = (int*)malloc(sizeof(int));
	*p_int = 1;

	if( (setsockopt(hsock, SOL_SOCKET, SO_REUSEADDR, (char*)p_int, sizeof(int)) == -1 )||
		(setsockopt(hsock, SOL_SOCKET, SO_KEEPALIVE, (char*)p_int, sizeof(int)) == -1 ) ){
		printf("Error setting options %d\n", errno);
		free(p_int);
		goto FINISH;
	}
	free(p_int);

	my_addr.sin_family = AF_INET ;
	my_addr.sin_port = htons(host_port);

	memset(&(my_addr.sin_zero), 0, 8);
	my_addr.sin_addr.s_addr = INADDR_ANY ;

	if( bind( hsock, (sockaddr*)&my_addr, sizeof(my_addr)) == -1 ){
		fprintf(stderr,"Error binding to socket, make sure nothing else is listening on this port %d\n",errno);
		goto FINISH;
	}
	if(listen( hsock, 10) == -1 ){
		fprintf(stderr, "Error listening %d\n",errno);
		goto FINISH;
	}

	//Now lets do the server stuff

	addr_size = sizeof(sockaddr_in);



	while(true){
/*
        printf("waiting for a connection\n");

        if((csock = accept( hsock, (sockaddr*)&sadr, &addr_size))!= -1){
            printf("---------------------\nReceived connection from %s\n",inet_ntoa(sadr.sin_addr));
            pthread_create(&thread_id,0,&SocketHandler, (void*)&csock );
            pthread_detach(thread_id);
        }
        else{
            fprintf(stderr, "Error accepting %d\n", errno);
        }
*/


		printf("waiting for a connection\n");
		//csock = (int*)malloc(sizeof(int));
		if((csock = accept( hsock, (sockaddr*)&sadr, &addr_size))!= -1){
			printf("---------------------\nReceived connection from %s\n",inet_ntoa(sadr.sin_addr));

			cout << "---------------------------" << endl;



            char buffer[1024];
            int buffer_len = 1024;
            int bytecount;
            memset(buffer, 0, buffer_len);
            DataLayout processThread;

            if((bytecount = recv(csock, buffer, buffer_len, 0))== -1){
                fprintf(stderr, "Error receiving data %d\n", errno);
                //goto FINISH;
            }

            printf("Received bytes %d\nReceived string \"%s\"\n", bytecount, buffer);

            for( int i = 0; i < numProcess ; i++)
            {
                if((buffer == process[i].name) && (process[i].active == 0))
                {

                    cout << "Configurando hilo" << endl;
                    processThread.name = process[i].name;
                    processThread.IP_RTAI = process[i].IP_RTAI;
                    processThread.Node2RTAI = process[i].Node2RTAI;
                    processThread.PORT_RTAI = process[i].PORT_RTAI;
                    processThread.RTAI2Node = process[i].RTAI2Node;
                    processThread.SHM = process[i].SHM;

                    processThread.csock = process[i].csock = csock;

                    //processFind = 1;
                    strcpy(buffer, "Creando proceso");
                    if((bytecount = send(csock, buffer, strlen(buffer), 0))== -1){
                        fprintf(stderr, "Error sending data %d\n", errno);
                        //goto FINISH;
                    }
                }
            }


            cout << "---------------------------" << endl;

			pthread_create(&thread_id,0,&SocketHandler, (void*)&processThread);
			pthread_detach(thread_id);






		}
		else{
			fprintf(stderr, "Error accepting %d\n", errno);
		}

	}

FINISH:
;
}


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
    int processFind = 0;


    transmision socketRTAI(csock);


    Struct_1 a;
    Struct_2 b;
    Struct_3 c;

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
	int bytecount;
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
        }


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
            default:
                cout << "Struct no valid...." << endl;
        }

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
            default:
                cout << "nada que mostrar" << endl;
        }

       usleep(1000000);
    }


FINISH:

	cout << "Close thread and connecion tcp" << endl;
    return 0;
}

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
