#include <stdio.h>      /* for printf() and fprintf() */
#include <sys/socket.h> /* for socket(), bind(), and connect() */
#include <arpa/inet.h>  /* for sockaddr_in and inet_ntoa() */
#include <stdlib.h>     /* for atoi() and exit() */
#include <string.h>     /* for memset() */
#include <unistd.h>     /* for close() */
#include <stdio.h>  /* for perror() */
#include <stdlib.h> /* for exit() */

#include <stdio.h>      /* for printf() and fprintf() */
#include <sys/socket.h> /* for recv() and send() */
#include <unistd.h>     /* for close() */

#define RCVBUFSIZE 32   /* Size of receive buffer */

#define MAXPENDING 5    /* Maximum outstanding connection requests */


void DieWithError(char *errorMessage);  /* Error handling function */
void HandleTCPClient(int clntSocket);   /* TCP client handling function */

struct vectorDouble
{
    double x;
    double y;
    double z;
};

struct sendData
{
    int ID;
    char name[20];

    struct vectorDouble linear;
    struct vectorDouble angular;

};

void DieWithError(char *errorMessage);  /* Error handling function */

void HandleTCPClient(int clntSocket)
{
    struct sendData a;
    char echoBuffer[RCVBUFSIZE];        /* Buffer for echo string */
    int recvMsgSize;                    /* Size of received message */




    /* Receive message from client */
    if ((recvMsgSize = recv(clntSocket, &a, sizeof(a), 0)) < 0)
        DieWithError("recv() failed");

    printf("Primera lectura, recvMsgSize = %d \n", recvMsgSize);
    printf("%s \n", a.name);


    /* Receive message from client */
/*
    if ((recvMsgSize = recv(clntSocket, echoBuffer, RCVBUFSIZE, 0)) < 0)
        DieWithError("recv() failed");
*/
    
    /* Send received string and receive again until end of transmission */
    while (recvMsgSize > 0)      /* zero indicates end of transmission */
    {

	printf("------------------Received data------------------ \n");
     	printf("a.name = %s \n", a.name);

     	printf("a.angular.x = %lf \n", a.angular.x);
     	printf("a.angular.y = %lf \n", a.angular.y);
     	printf("a.angular.z = %lf \n", a.angular.z);

     	printf("a.linear.x = %lf \n", a.linear.x);
     	printf("a.linear.y = %lf \n", a.linear.y);
     	printf("a.linear.z = %lf \n", a.linear.z);


	a.ID = 10;
    	a.name[0]= 'P';
    	a.name[1]= 'E';
    	a.name[2]= 'P';
    	a.name[3]= 'I';
    	a.name[4]= 'T';

    	a.angular.x = 0;
    	a.angular.y = 0;
    	a.angular.z = 0;

    	a.linear.x = 0;
    	a.linear.y = 0;
    	a.linear.z = 0;


    	if (send(clntSocket, &a, sizeof(a), 0) != sizeof(a))
            DieWithError("send() sent a different number of bytes than expected");

//        printf("%s \n", echoBuffer);
        /* Echo message back to client */
/*
	echoBuffer[0] = '1';
	echoBuffer[1] = '2';
	echoBuffer[2] = '3';
	echoBuffer[3] = '4';
        if (send(clntSocket, echoBuffer, recvMsgSize, 0) != recvMsgSize)
            DieWithError("send() failed");
*/

        /* Receive message from client */
        if ((recvMsgSize = recv(clntSocket, &a, sizeof(a), 0)) < 0)
            DieWithError("recv() failed");


        /* See if there is more data to receive */
/*
        if ((recvMsgSize = recv(clntSocket, echoBuffer, RCVBUFSIZE, 0)) < 0)
            DieWithError("recv() failed");
*/
	printf("recvMsgSize = %d \n", recvMsgSize);
    }

    close(clntSocket);    /* Close client socket */
}

void DieWithError(char *errorMessage)
{
    perror(errorMessage);
    exit(1);
}


int main(int argc, char *argv[])
{
    int servSock;                    /* Socket descriptor for server */
    int clntSock;                    /* Socket descriptor for client */
    struct sockaddr_in echoServAddr; /* Local address */
    struct sockaddr_in echoClntAddr; /* Client address */
    unsigned short echoServPort;     /* Server port */
    unsigned int clntLen;            /* Length of client address data structure */

    if (argc != 2)     /* Test for correct number of arguments */
    {
        fprintf(stderr, "Usage:  %s <Server Port>\n", argv[0]);
        exit(1);
    }

    echoServPort = atoi(argv[1]);  /* First arg:  local port */

    /* Create socket for incoming connections */
    if ((servSock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
        DieWithError("socket() failed");
      
    /* Construct local address structure */
    memset(&echoServAddr, 0, sizeof(echoServAddr));   /* Zero out structure */
    echoServAddr.sin_family = AF_INET;                /* Internet address family */
    echoServAddr.sin_addr.s_addr = htonl(INADDR_ANY); /* Any incoming interface */
    echoServAddr.sin_port = htons(echoServPort);      /* Local port */

    /* Bind to the local address */
    if (bind(servSock, (struct sockaddr *) &echoServAddr, sizeof(echoServAddr)) < 0)
        DieWithError("bind() failed");

    /* Mark the socket so it will listen for incoming connections */
    if (listen(servSock, MAXPENDING) < 0)
        DieWithError("listen() failed");

    for (;;) /* Run forever */
    {
        /* Set the size of the in-out parameter */
        clntLen = sizeof(echoClntAddr);

        /* Wait for a client to connect */
        if ((clntSock = accept(servSock, (struct sockaddr *) &echoClntAddr, 
                               &clntLen)) < 0)
            DieWithError("accept() failed");

        /* clntSock is connected to a client! */

        printf("Handling client %s\n", inet_ntoa(echoClntAddr.sin_addr));

        HandleTCPClient(clntSock);
    }
    /* NOT REACHED */
}

