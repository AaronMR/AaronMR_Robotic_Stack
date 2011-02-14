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

void* SocketHandler(void*);

int main(int argv, char** argc){

    int host_port= 1101;

    struct sockaddr_in my_addr;

    int hsock;
    int * p_int ;
    int err;

    socklen_t addr_size = 0;
    int* csock;
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
        printf("waiting for a connection\n");
        csock = (int*)malloc(sizeof(int));
        if((*csock = accept( hsock, (sockaddr*)&sadr, &addr_size))!= -1){
            printf("---------------------\nReceived connection from %s\n",inet_ntoa(sadr.sin_addr));
	    int bytecount = 0;

	char buffer[1024];
        int buffer_len = 1024;
	strcpy(buffer, "puta mierdaaaaaaaaaaa");
        if((bytecount = send(*csock, buffer, strlen(buffer), 0))== -1)
	{
          	fprintf(stderr, "Error sending data %d\n", errno);
          	//goto FINISH;
       	}
	printf("bytecount = %d \n", bytecount);

            pthread_create(&thread_id,0,&SocketHandler, (void*)csock );
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
    int *csock = (int*)lp;

    char buffer[1024];
    int buffer_len = 1024;
    int bytecount;

    memset(buffer, 0, buffer_len);
    if((bytecount = recv(*csock, buffer, buffer_len, 0))== -1){
        fprintf(stderr, "Error receiving data %d\n", errno);
        goto FINISH;
    }
    printf("Received bytes %d\nReceived string \"%s\"\n", bytecount, buffer);
    strcat(buffer, " SERVER ECHO");

    if((bytecount = send(*csock, buffer, strlen(buffer), 0))== -1){
        fprintf(stderr, "Error sending data %d\n", errno);
        goto FINISH;
    }
    
    printf("Sent bytes %d\n", bytecount);


FINISH:
    free(csock);
    return 0;
}
