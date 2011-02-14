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

#include <iostream>

#include "parser.h"
//#include "parameters.h"

#include "transmition.h"

using namespace std;

int exitClient(int hsock);

DataLayout process[5];

int main(int argv, char** argc){

	int host_port= 0000;
//	char* host_name="140.78.133.232";//"127.0.0.1";
	char* host_name="111.111.111.111";

	struct sockaddr_in my_addr;


	parseFile parser(argc[1]);
	parser.Parse(process);


	host_port = atoi(process[0].PORT_RTAI.data());
	host_name = (char*)process[0].IP_RTAI.data();
    int counter = 0;

	char buffer[1024];
	int bytecount;
	int buffer_len=0;

	int hsock;
	int * p_int;
	int err;

	hsock = socket(AF_INET, SOCK_STREAM, 0);
	if(hsock == -1){
		printf("Error initializing socket %d\n",errno);
		//goto FINISH;
		exitClient(hsock);
	}

    transmision socketRTAI(hsock);

	p_int = (int*)malloc(sizeof(int));
	*p_int = 1;

	if( (setsockopt(hsock, SOL_SOCKET, SO_REUSEADDR, (char*)p_int, sizeof(int)) == -1 )||
		(setsockopt(hsock, SOL_SOCKET, SO_KEEPALIVE, (char*)p_int, sizeof(int)) == -1 ) ){
		printf("Error setting options %d\n",errno);
		free(p_int);
		//goto FINISH;
		exitClient(hsock);
	}
	free(p_int);

	my_addr.sin_family = AF_INET ;
	my_addr.sin_port = htons(host_port);

	memset(&(my_addr.sin_zero), 0, 8);
	my_addr.sin_addr.s_addr = inet_addr(host_name);

	if( connect( hsock, (struct sockaddr*)&my_addr, sizeof(my_addr)) == -1 ){
		if((err = errno) != EINPROGRESS){
			fprintf(stderr, "Error connecting socket %d\n", errno);
			//goto FINISH;
			exitClient(hsock);
		}
	}

	//Now lets do the client related stuff

	buffer_len = 1024;

	memset(buffer, '\0', buffer_len);

	memcpy(buffer, process[0].name.data(), process[0].name.length() );
	cout << "The name of the service is = " << buffer << endl;

	if( (bytecount=send(hsock, buffer, strlen(buffer),0))== -1){
		fprintf(stderr, "Error sending data %d\n", errno);
		//goto FINISH;
		exitClient(hsock);
	}

	memset(buffer, '\0', buffer_len);

	if((bytecount = recv(hsock, buffer, buffer_len, 0))== -1){
		fprintf(stderr, "Error receiving data %d\n", errno);
		//goto FINISH;
        exitClient(hsock);
	}
	printf("Recieved bytes %d\nReceived string \"%s\"\n", bytecount, buffer);

	if(strcmp(buffer, "close") == 0)
	{
		cout << "puta mierda"<< endl;
		//goto FINISH;
		exitClient(hsock);
	}


    Struct_1 a;
    Struct_2 b;

    a.indx_counter = counter;
    a.cos_value = rand();
    a.sin_value = rand();

    b.indx_counter = counter;;
    b.cos_value = rand();
    b.sin_value = rand();

    counter = counter + 1;
    //------------------------------------------------

	while(1)
    	{

            cout << "--- Process = " << process[0].name << "---" << endl;

            int Node2RTAI = 0;
            int RTAI2Node = 0;

            counter = counter +1;
            a.indx_counter = counter;
            a.cos_value = counter;
        	a.sin_value = counter;

            b.indx_counter = counter;
            b.cos_value = counter;
        	b.sin_value = counter;

        if(process[0].Node2RTAI == "Struct_1")
        {
            Node2RTAI = 1;
        }else if(process[0].Node2RTAI == "Struct_2")
        {
            Node2RTAI = 2;
        }


        switch(Node2RTAI)
        {
            case 1:
                if(socketRTAI.sendData(&a) == -1){
                    fprintf(stderr, "Error receiving data %d\n", errno);
                    //goto FINISH;
                        exitClient(hsock);
                }
                break;

            case 2:
                if(socketRTAI.sendData(&b) == -1){
                    fprintf(stderr, "Error receiving data %d\n", errno);
                    //goto FINISH;
                        exitClient(hsock);
                }
                break;

            default:
                cout << "nada que mostrar" << endl;
        }


                a.indx_counter = a.indx_counter * (-1);
                a.cos_value = a.cos_value * (-1);
                a.sin_value = a.sin_value * (-1);


            if(process[0].RTAI2Node == "Struct_1")
            {
                RTAI2Node = 1;
            }else if(process[0].RTAI2Node == "Struct_2")
            {
                RTAI2Node = 2;
            }

        switch(RTAI2Node)

        {
                case 1:
                    if(socketRTAI.recvData(&a) == -1){
                        fprintf(stderr, "Error receiving data %d\n", errno);
                        //goto FINISH;
                        exitClient(hsock);
                    }
                    break;

                case 2:
                    if(socketRTAI.recvData(&b) == -1){
                        fprintf(stderr, "Error receiving data %d\n", errno);
                        //goto FINISH;
                        exitClient(hsock);
                    }
                    break;

                default:
                    cout << "nada que mostrar" << endl;
            }


	usleep(100000);
	}
	close(hsock);

}

int exitClient(int hsock)
{
	close(hsock);
	return 0;
    //exit(1);
}
