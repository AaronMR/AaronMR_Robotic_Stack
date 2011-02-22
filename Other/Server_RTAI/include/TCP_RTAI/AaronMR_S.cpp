#include "AaronMR_S.hpp"
#include <iostream>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string>
#include <errno.h>
#include "pack2.hpp"
#include "transmition.h"

#include <pthread.h>

// time to sleep threads and principal proces
int usleepServer = 100000;
int usleepThread = 100000;

int exitClient(int hsock);

AaronMR_S::AaronMR_S(char * aux)
{

    FD_ZERO(&master);    // clear the master and temp sets
    FD_ZERO(&read_fds);

    DataLayout auxFile[5];
    parseFile parser(aux);
    parser.Parse(auxFile);
    numProcess = parser.getNumProcess();

    pthread_mutexattr_t mta;
    pthread_mutexattr_init(&mta);



    for(int i=0;i<numProcess;i++)
    {
        configuration[i].active = auxFile[i].active;
        configuration[i].csock = auxFile[i].csock;
        configuration[i].IP_RTAI = auxFile[i].IP_RTAI.data();
        configuration[i].name = auxFile[i].name.data();
        configuration[i].Node2RTAI = auxFile[i].Node2RTAI.data();
        configuration[i].PORT_RTAI = auxFile[i].PORT_RTAI.data();
        configuration[i].Publisher = auxFile[i].Publisher.data();
        configuration[i].RTAI2Node = auxFile[i].RTAI2Node.data();
        configuration[i].SHM_IN = auxFile[i].SHM_IN.data();
        configuration[i].SHM_OUT = auxFile[i].SHM_OUT.data();
        configuration[i].Subscriber = auxFile[i].Subscriber.data();

        //configuration[i].mutex = PTHREAD_MUTEX_INITIALIZER;
        pthread_mutex_init(&configuration[i].mutex, &mta);
        configuration[i].canRecv = false;
        configuration[i].canSend = false;
    }
//    configuration[0].name = auxFile[0].name.data();
//    configuration[1].name = auxFile[1].name.data();
//    configuration[2].name = auxFile[2].name.data();


    numProcess = parser.getNumProcess();
    if(numProcess == 0)
    {
        cout << "No hay ningun proceso configurado" << endl;
    }else{
        cout << "Procesos configurados = " << parser.getNumProcess() <<endl;
    }

	int host_port= 1101;

	int * p_int ;
//	int err;

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

	FD_SET(hsock, &master);

	// keep track of the biggest file descriptor
    fdmax = hsock; // so far, it's this one

	//Now lets do the server stuff

	addr_size = sizeof(sockaddr_in);


FINISH:
;
    return;


};


int AaronMR_S::create_Socket()
{
    hsock = socket(AF_INET, SOCK_STREAM, 0);
    if(hsock == -1)
    {
        printf("Error initializing socket %d\n",errno);
        return -1;
    }
    return hsock;
}


string mierda("miardaaa");

void *get_in_addr(struct sockaddr *sa)
{
    if (sa->sa_family == AF_INET) {
        return &(((struct sockaddr_in*)sa)->sin_addr);
    }

    return &(((struct sockaddr_in6*)sa)->sin6_addr);
}

int AaronMR_S::waitEvent()
{

    // main loop
    cout << "dentro" << endl;

    //########################################################################
    // main loop
    listener = hsock;
    for(;;) {
        usleep(usleepServer);
        //cout << "Yeahhhh!!!!" << endl;
        read_fds = master; // copy it
        if (select(fdmax+1, &read_fds, NULL, NULL, NULL) == -1) {
            perror("select");
            exit(4);
        }

        // run through the existing connections looking for data to read
        int i=0;
        for(i = 0; i <= fdmax; i++) {
            if (FD_ISSET(i, &read_fds)) { // we got one!!
                if (i == listener) {
                    // handle new connections
                    addrlen = sizeof remoteaddr;
                    newfd = accept(hsock,
                        (struct sockaddr *)&remoteaddr,
                        &addrlen);

                    if (newfd == -1) {
                        perror("accept");
                    } else {
                        FD_SET(newfd, &master); // add to master set
                        if (newfd > fdmax) {    // keep track of the max
                            fdmax = newfd;
                        }
                        printf("selectserver: new connection from %s on "
                            "socket %d\n",
                            inet_ntop(remoteaddr.ss_family,
                                get_in_addr((struct sockaddr*)&remoteaddr),
                                remoteIP, INET6_ADDRSTRLEN),
                            newfd);

                        startThread(newfd);
                    }
                } else {
                    int numberThread = -1;
                    for(int w = 0; w < 5 ; w++)
                    {
                        if(configuration[w].csock == i)
                        {
                            numberThread = w;
                        }
                    }

                    if(numberThread != -1)
                    {
                        pthread_mutex_lock(&configuration[numberThread].mutex);

                        configuration[numberThread].canRecv = true; //rand() % 2 + -1;
                        //configuration[numberThread].canSend = true; //rand() % 2 + -1;

                        pthread_mutex_unlock(&configuration[numberThread].mutex);
                    }

                    /*
                    int nbytes = 0;
                    int j=0;
                    char buf[1024];    // buffer for client data
                    // handle data from a client
                    if ((nbytes = recv(i, buf, sizeof buf, 0)) <= 0) {
                        // got error or connection closed by client
                        if (nbytes == 0) {
                            // connection closed
                            printf("selectserver: socket %d hung up\n", i);
                        } else {
                            perror("recv");
                        }
                        close(i); // bye!
                        FD_CLR(i, &master); // remove from master set
                    } else {
                        // we got some data from a client
                        for(j = 0; j <= fdmax; j++) {
                            // send to everyone!
                            if (FD_ISSET(j, &master)) {
                                // except the listener and ourselves
                                if (j != listener && j != i) {
                                    if (send(j, buf, nbytes, 0) == -1) {
                                        perror("send");
                                    }
                                }
                            }
                        }
                    }
                    */
                } // END handle data from client
            } // END got new incoming connection
        } // END looping through file descriptors
    } // END for(;;)--and you thought it would never end!
    //########################################################################


    return 0;
}

//############################################################################################

int AaronMR_S::startThread(int csock)
{
    printf("---------------------\nReceived connection from %s\n",inet_ntoa(sadr.sin_addr));
    cout << "---------------------------" << endl;

    char buffer[1024];
    int buffer_len = 1024;
    int bytecount;
    memset(buffer, 0, buffer_len);
    DataLayout processThread;

    //#########################################################

    unsigned char magic;
    char *s = "Maki";
    unsigned int ps2;
    char s2[96];
    char s3[96];
    char s4[96];
    unsigned char buf[1024];

    char maki2[1024];

    //structToSend->send(hsock);
    //cout << "maki2 = " << strlen((const char*)maki2) << endl;

    if( (bytecount=recv(csock, (void*)maki2, 1024,0)) < 1)
    {
        fprintf(stderr, "recv_Data Error reciving data %d\n", errno);
        //goto FINISH;
        exitClient(hsock);
        return -1;
    }


    memcpy((unsigned char*)buf, maki2, 1024);

    unpack((unsigned char*)buf, "CHsss",    &magic,
                                            &ps2,
                                            &s2,
                                            &s3,
                                            &s4);

    printf("recv: '%c' %hhu %s %s %s\n",    magic,
                                            ps2,
                                            s2,
                                            s3,
                                            s4);

    //#########################################################

    int existProcess = 0;
    int activeProcess = 0;
    int process2Active = -1;
    for( int i = 0; i < numProcess ; i++)
    {
        if(configuration[i].name.compare(s2) == 0)
        {
            cout << "encontre el proceso a encender = " << s2 << endl;
            process2Active = i;
        }
    }

    //#########################################################################

    if(process2Active > -1) //(configuration[i].name.compare(s2) == 0) && (configuration[i].active == 0))
    {
        cout << "active =" << configuration[process2Active].active << endl;
        if(configuration[process2Active].active == 0)
        {
            configuration[process2Active].active = 1;
            existProcess = 1;
            activeProcess = 1;

            processThread.name = configuration[process2Active].name;
            processThread.IP_RTAI = configuration[process2Active].IP_RTAI;
            processThread.Node2RTAI = configuration[process2Active].Node2RTAI;
            processThread.PORT_RTAI = configuration[process2Active].PORT_RTAI;
            processThread.RTAI2Node = configuration[process2Active].RTAI2Node;
            processThread.SHM_IN = configuration[process2Active].SHM_IN;
            processThread.SHM_OUT = configuration[process2Active].SHM_OUT;

            processThread.csock = configuration[process2Active].csock = csock;


            //##########################################################
            unsigned int packetsize, ps2;

            char s2[96];
            char s3[96];
            char s4[96];
            char buffer[1024];
            packetsize = pack(buf, "CHs", 'A', 0, "Existe el proceso");

            packi16(buf+1, packetsize);


            memcpy((unsigned char*)buffer, buf, packetsize);


            if( (bytecount=send(csock, (void*)buffer, 1024,0)) < 1)
            {
                fprintf(stderr, "recv_Data Error reciving data %d\n", errno);
                //goto FINISH;
                exitClient(hsock);
                return -1;
            }

            unpack((unsigned char*)buffer, "CHs",   &magic,
                                                    &ps2,
                                                    &s2);

            printf("send: '%c' %hhu %s\n",  magic,
                                            ps2,
                                            s2);

            //pthread_create( &thread_id, 0, &AaronMR_S::SocketHandler, (void*)&processThread);
            pthread_create( &thread_id, 0, &AaronMR_S::SocketHandler, (void*)&configuration[process2Active]);
            pthread_detach(thread_id);



        }else{
            cout << "el proceso esta activo, lo sentimos" << endl;

            unsigned int packetsize, ps2;
            char *s = "Maki";
            char s2[96];
            char s3[96];
            char s4[96];
            char buffer[1024];
            packetsize = pack(buf, "CHs", 'E', 0, "Proceso Activo");

            packi16(buf+1, packetsize);


            memcpy((unsigned char*)buffer, buf, packetsize);


            if( (bytecount=send(csock, (void*)buffer, 1024,0)) < 1)
            {
                fprintf(stderr, "recv_Data Error reciving data %d\n", errno);
                //goto FINISH;
                exitClient(hsock);
                return -1;
            }

            unpack((unsigned char*)buffer, "CHs",   &magic,
                                                    &ps2,
                                                    &s2);

            printf("send: '%c' %hhu %s\n",  magic,
                                            ps2,
                                            s2);
        }

    }

    if(existProcess == 0)
    {
        cout << "no existe el proceso con nombre " << s2 << endl;
        unsigned int packetsize, ps2;
        char *s = "Maki";
        char s2[96];
        char s3[96];
        char s4[96];
        char buffer[1024];
        packetsize = pack(buf, "CHs", 'E', 0, "no existe");

        packi16(buf+1, packetsize);

        memcpy((unsigned char*)buffer, buf, packetsize);

        if( (bytecount=send(csock, (void*)buffer, 1024,0)) < 1)
        {
            fprintf(stderr, "recv_Data Error reciving data %d\n", errno);
            //goto FINISH;
            //exitClient(hsock);
            return -1;
        }

        unpack((unsigned char*)buffer, "CHs",   &magic,
                                                &ps2,
                                                &s2);

        printf("send: '%c' %hhu %s\n",  magic,
                                        ps2,
                                        s2);
    }

    if(activeProcess == 0)
    {
        cout << "el proceso ya esta activo " << s2 << endl;
        unsigned int packetsize, ps2;
        char *s = "Maki";
        char s2[96];
        char s3[96];
        char s4[96];
        char buffer[1024];
        packetsize = pack(buf, "CHs", 'E', 0, "proceso activo");

        packi16(buf+1, packetsize);

        memcpy((unsigned char*)buffer, buf, packetsize);


        if( (bytecount=send(csock, (void*)buffer, 1024,0)) < 1)
        {
            fprintf(stderr, "recv_Data Error reciving data %d\n", errno);
            //goto FINISH;
            //exitClient(hsock);
            return -1;
        }

        unpack((unsigned char*)buffer, "CHs",   &magic,
                                                &ps2,
                                                &s2);

        printf("send: '%c' %hhu %s\n",  magic,
                                        ps2,
                                        s2);
    }
}

//############################################################################################

int AaronMR_S::acceptConnection()
{


        int csock;
		printf("waiting for a connection\n");
		//csock = (int*)malloc(sizeof(int));



		/*
		// funcion para activar y desactivar el envio de datos....
		while(1)
        {
            pthread_mutex_lock(&configuration[process2Active].mutex);

            configuration[process2Active].canRecv = rand() % 2 + -1;
            configuration[process2Active].canSend = rand() % 2 + -1;

            pthread_mutex_unlock(&configuration[process2Active].mutex);

            usleep(1000);

        }
        */



		if((csock = accept( hsock, (sockaddr*)&sadr, &addr_size))!= -1){
			printf("---------------------\nReceived connection from %s\n",inet_ntoa(sadr.sin_addr));

			cout << "---------------------------" << endl;

            char buffer[1024];

            int buffer_len = 1024;
            int bytecount;
            memset(buffer, 0, buffer_len);
            DataLayout processThread;

            //if((bytecount = recv(csock, buffer, buffer_len, 0))== -1){
            //    fprintf(stderr, "Error receiving data %d\n", errno);
            //    //goto FINISH;
            //}

            //#########################################################

            unsigned char magic;
            char *s = "Maki";
            unsigned int ps2;
            char s2[96];
            char s3[96];
            char s4[96];
            unsigned char buf[1024];

            char maki2[1024];

            //structToSend->send(hsock);
            //cout << "maki2 = " << strlen((const char*)maki2) << endl;

            if( (bytecount=recv(csock, (void*)maki2, 1024,0)) < 1)
            {
                fprintf(stderr, "recv_Data Error reciving data %d\n", errno);
                //goto FINISH;
                exitClient(hsock);
                return -1;
            }


            memcpy((unsigned char*)buf, maki2, 1024);

            unpack((unsigned char*)buf, "CHsss",    &magic,
                                            &ps2,
                                            &s2,
                                            &s3,
                                            &s4);

            printf("recv: '%c' %hhu %s %s %s\n",   magic,
                                     ps2,
                                     s2,
                                     s3,
                                     s4);

            cout << configuration[0].name.data()<< s2 << endl;
            cout << configuration[1].name.data()<< s2 << endl;
            cout << configuration[2].name.data()<< s2 << endl;
            //#########################################################

            int existProcess = 0;
            int activeProcess = 0;
            int process2Active = -1;


            for( int i = 0; i < numProcess ; i++)
            {
                if(configuration[i].name.compare(s2) == 0)
                {
                    cout << "encontre el proceso a encender = " << s2 << endl;
                    process2Active = i;
                }
            }

            //#########################################################################

            if(process2Active > -1) //(configuration[i].name.compare(s2) == 0) && (configuration[i].active == 0))
            {
                cout << "active =" << configuration[process2Active].active << endl;
                if(configuration[process2Active].active == 0)
                {
                    configuration[process2Active].active = 1;
                    existProcess = 1;
                    activeProcess = 1;

                    cout << "Configurando hilo " << s2 << endl;
                    processThread.name = configuration[process2Active].name;
                    processThread.IP_RTAI = configuration[process2Active].IP_RTAI;
                    processThread.Node2RTAI = configuration[process2Active].Node2RTAI;
                    processThread.PORT_RTAI = configuration[process2Active].PORT_RTAI;
                    processThread.RTAI2Node = configuration[process2Active].RTAI2Node;
                    processThread.SHM_IN = configuration[process2Active].SHM_IN;
                    processThread.SHM_OUT = configuration[process2Active].SHM_OUT;

                    processThread.csock = configuration[process2Active].csock = csock;


                    //##########################################################
                    unsigned int packetsize, ps2;
                    char *s = "Maki";
                    char s2[96];
                    char s3[96];
                    char s4[96];
                    char buffer[1024];
                    packetsize = pack(buf, "CHs", 'A', 0, "Existe el proceso");

                    packi16(buf+1, packetsize);


                    memcpy((unsigned char*)buffer, buf, packetsize);


                    if( (bytecount=send(csock, (void*)buffer, 1024,0)) < 1)
                    {
                        fprintf(stderr, "recv_Data Error reciving data %d\n", errno);
                        //goto FINISH;
                        exitClient(hsock);
                        return -1;
                    }

                    unpack((unsigned char*)buffer, "CHs",    &magic,
                                                            &ps2,
                                                            &s2);

                    printf("send: '%c' %hhu %s\n",   magic,
                                                     ps2,
                                                     s2);
                //################################################################
                /*
                    //processFind = 1;
                    strcpy(buffer, "Creando proceso");
                    if((bytecount = send(csock, buffer, strlen(buffer), 0))== -1){
                        fprintf(stderr, "Error sending data %d\n", errno);
                        //goto FINISH;
                    }

                    cout << "---------------------------" << endl;

                */
                    // creacion del hilo
                    //pthread_create( &thread_id, 0, &AaronMR_S::SocketHandler, (void*)&processThread);
                    pthread_create( &thread_id, 0, &AaronMR_S::SocketHandler, (void*)&configuration[process2Active]);
                    pthread_detach(thread_id);



                }else{
                    cout << "el proceso esta activo, lo sentimos" << endl;

                    unsigned int packetsize, ps2;
                    char *s = "Maki";
                    char s2[96];
                    char s3[96];
                    char s4[96];
                    char buffer[1024];
                    packetsize = pack(buf, "CHs", 'E', 0, "Proceso Activo");

                    packi16(buf+1, packetsize);


                    memcpy((unsigned char*)buffer, buf, packetsize);


                    if( (bytecount=send(csock, (void*)buffer, 1024,0)) < 1)
                    {
                        fprintf(stderr, "recv_Data Error reciving data %d\n", errno);
                        //goto FINISH;
                        exitClient(hsock);
                        return -1;
                    }

                    unpack((unsigned char*)buffer, "CHs",    &magic,
                                                            &ps2,
                                                            &s2);

                    printf("send: '%c' %hhu %s\n",   magic,
                                                     ps2,
                                                     s2);

                }


            }


            //#########################################################################

            /*
            for( int i = 0; i < numProcess ; i++)
            {

                if((configuration[i].name.compare(s2) == 0) && (configuration[i].active == 0))
                {
                    existProcess = 1;
                    activeProcess = 1;

                    cout << "Configurando hilo " << s2 << endl;
                    processThread.name = configuration[i].name;
                    processThread.IP_RTAI = configuration[i].IP_RTAI;
                    processThread.Node2RTAI = configuration[i].Node2RTAI;
                    processThread.PORT_RTAI = configuration[i].PORT_RTAI;
                    processThread.RTAI2Node = configuration[i].RTAI2Node;
                    processThread.SHM = configuration[i].SHM;

                    processThread.csock = configuration[i].csock = csock;


                    //##########################################################
                    unsigned int packetsize, ps2;
                    char *s = "Maki";
                    char s2[96];
                    char s3[96];
                    char s4[96];
                    char buffer[1024];
                    packetsize = pack(buf, "CHs", 'A', 0, "Existe el proceso");

                    packi16(buf+1, packetsize);


                    memcpy((unsigned char*)buffer, buf, packetsize);


                    if( (bytecount=send(csock, (void*)buffer, 1024,0)) < 1)
                    {
                        fprintf(stderr, "recv_Data Error reciving data %d\n", errno);
                        //goto FINISH;
                        exitClient(hsock);
                        return -1;
                    }

                    unpack((unsigned char*)buffer, "CHs",    &magic,
                                                            &ps2,
                                                            &s2);

                    printf("send: '%c' %hhu %s\n",   magic,
                                                     ps2,
                                                     s2);
                //################################################################

                //   //processFind = 1;
                //    strcpy(buffer, "Creando proceso");
                //    if((bytecount = send(csock, buffer, strlen(buffer), 0))== -1){
                //        fprintf(stderr, "Error sending data %d\n", errno);
                //        //goto FINISH;
                //    }

                //    cout << "---------------------------" << endl;


                    // creacion del hilo
                    pthread_create( &thread_id, 0, &AaronMR_S::SocketHandler, (void*)&processThread);
                    pthread_detach(thread_id);



                }
            }
            */

            if(existProcess == 0)
            {
                cout << "no existe el proceso con nombre " << s2 << endl;
                unsigned int packetsize, ps2;
                char *s = "Maki";
                char s2[96];
                char s3[96];
                char s4[96];
                char buffer[1024];
                packetsize = pack(buf, "CHs", 'E', 0, "no existe");

                packi16(buf+1, packetsize);


                memcpy((unsigned char*)buffer, buf, packetsize);


                if( (bytecount=send(csock, (void*)buffer, 1024,0)) < 1)
                {
                    fprintf(stderr, "recv_Data Error reciving data %d\n", errno);
                    //goto FINISH;
                    //exitClient(hsock);
                    return -1;
                }

                unpack((unsigned char*)buffer, "CHs",    &magic,
                                                        &ps2,
                                                        &s2);

                printf("send: '%c' %hhu %s\n",   magic,
                                                 ps2,
                                                 s2);
            }

            if(activeProcess == 0)
            {
                cout << "el proceso ya esta activo " << s2 << endl;
                unsigned int packetsize, ps2;
                char *s = "Maki";
                char s2[96];
                char s3[96];
                char s4[96];
                char buffer[1024];
                packetsize = pack(buf, "CHs", 'E', 0, "proceso activo");

                packi16(buf+1, packetsize);


                memcpy((unsigned char*)buffer, buf, packetsize);


                if( (bytecount=send(csock, (void*)buffer, 1024,0)) < 1)
                {
                    fprintf(stderr, "recv_Data Error reciving data %d\n", errno);
                    //goto FINISH;
                    //exitClient(hsock);
                    return -1;
                }

                unpack((unsigned char*)buffer, "CHs",    &magic,
                                                        &ps2,
                                                        &s2);

                printf("send: '%c' %hhu %s\n",   magic,
                                                 ps2,
                                                 s2);
            }










		}
		else{
			fprintf(stderr, "Error accepting %d\n", errno);
			exitClient(hsock);
		}

	//}

}

int AaronMR_S::connect_Socket()
{
    int err;
    int bytecount = 0;
    if( connect( hsock, (struct sockaddr*)&my_addr, sizeof(my_addr)) == -1 )
    {
        if((err = errno) != EINPROGRESS)
        {
            fprintf(stderr, "Error connecting socket %d\n", errno);
            exitClient(hsock);
        }
    }

    buffer_len = 1024;
    memset(buffer, '\0', buffer_len);
    memcpy(buffer, configuration[0].name.data(), configuration[0].name.length() );
    cout << "The name of the service is = " << buffer << endl;

    if( (bytecount=send(hsock, buffer, strlen(buffer),0))== -1)
    {
        fprintf(stderr, "Error sending data %d\n", errno);

        exitClient(hsock);
    }

    memset(buffer, '\0', buffer_len);

    if((bytecount = recv(hsock, buffer, buffer_len, 0))== -1)
    {
        fprintf(stderr, "Error receiving data %d\n", errno);
        exitClient(hsock);
    }
    printf("Recieved bytes %d\nReceived string \"%s\"\n", bytecount, buffer);

    if(strcmp(buffer, "close") == 0)
    {
        cout << "se cerro"<< endl;
        exitClient(hsock);
    }
}


AaronMR_S::~AaronMR_S()
{
//    int a=0;
};



void* AaronMR_S::SocketHandler(void* lp){


    struct DataLayout *temp = (struct DataLayout *)lp;
    struct DataLayout processThread_2;
    processThread_2.active = temp->active;

    processThread_2.name = temp->name;
    processThread_2.IP_RTAI = temp->IP_RTAI;
    processThread_2.Node2RTAI = temp->Node2RTAI;
    processThread_2.PORT_RTAI = temp->PORT_RTAI;
    processThread_2.RTAI2Node = temp->RTAI2Node;
    processThread_2.SHM_IN = temp->SHM_IN;
    processThread_2.SHM_OUT = temp->SHM_OUT;
    processThread_2.active = temp->active;
    processThread_2.csock = temp->csock;

    temp->canRecv = true;
    temp->canSend = true;

    int csock = processThread_2.csock;
    int counter = 0;
//    int processFind = 0;


    transmision socketRTAI(csock);

    structType* structToSend;
    structType* structToRecv;
    //int Node2RTAI = 0;
    //int RTAI2Node = 0;


//    const char* Node2RTAI_ = processThread_2.Node2RTAI.data();
//    const char* RTAI2Node_ = processThread_2.RTAI2Node.data();

    // configuration send structure
    if(processThread_2.Node2RTAI.compare("Twist") == 0)
    {
        //Node2RTAI = 4;
        structToRecv = new struct_Twist;
        structToRecv->iniSHM(1, 0, (char*)processThread_2.SHM_IN.data());


    }else if(processThread_2.Node2RTAI.compare("Odometry") == 0)
    {
        //Node2RTAI = 5;
        structToRecv = new struct_Joy;
        structToRecv->iniSHM(1, 0, (char*)processThread_2.SHM_IN.data());

    }else if(processThread_2.Node2RTAI.compare("Joy") == 0)
    {
        //Node2RTAI = 6;
        structToRecv = new struct_Joy;
        structToRecv->iniSHM(1, 0, (char*)processThread_2.SHM_IN.data());

    }else if(processThread_2.Node2RTAI.compare("Pose") == 0)
    {
        //Node2RTAI = 6;
        structToRecv = new struct_Pose;
        structToRecv->iniSHM(1, 0, (char*)processThread_2.SHM_IN.data());

    }else if(processThread_2.Node2RTAI.compare("posWheels") == 0)
    {
        //Node2RTAI = 6;
        structToRecv = new struct_posWheels;
        structToRecv->iniSHM(1, 0, (char*)processThread_2.SHM_IN.data());

    }




    // configuration recv structure
    if(processThread_2.RTAI2Node.compare("Twist") == 0)
    {
        //RTAI2Node = 4;
        structToSend = new struct_Twist;
        structToSend->iniSHM(0,1, (char*)processThread_2.SHM_OUT.data());

    }else if(processThread_2.RTAI2Node.compare("Odometry") == 0)
    {
        //RTAI2Node = 5;
        structToSend = new struct_Joy;
        structToSend->iniSHM(0,1, (char*)processThread_2.SHM_OUT.data());

    }else if(processThread_2.RTAI2Node.compare("Joy") == 0)
    {
        //RTAI2Node = 6;
        structToSend = new struct_Joy;
        structToSend->iniSHM(0,1, (char*)processThread_2.SHM_OUT.data());

    }else if(processThread_2.RTAI2Node.compare("Pose") == 0)
    {
        //RTAI2Node = 6;
        structToSend = new struct_Pose;
        structToSend->iniSHM(0,1, (char*)processThread_2.SHM_OUT.data());
    }else if(processThread_2.RTAI2Node.compare("posWheels") == 0)
    {
        //RTAI2Node = 6;
        structToSend = new struct_posWheels;
        structToSend->iniSHM(0,1, (char*)processThread_2.SHM_OUT.data());
    }

    counter = counter + 1;

    cout << "name = " << processThread_2.name << endl
        << "IP_RTAI = " << processThread_2.IP_RTAI << endl
        << "Node2RTAI = " << processThread_2.Node2RTAI << endl
        << "PORT_RTAI = " << processThread_2.PORT_RTAI << endl
        << "RTAI2Node = " << processThread_2.RTAI2Node << endl
        << "SHM_IN = " << processThread_2.SHM_IN << endl
        << "SHM_OUT = " << processThread_2.SHM_OUT << endl
        << " = " << processThread_2.active << endl
        << " = " << processThread_2.csock << endl ;



	char buffer[1024];
	int buffer_len = 1024;
	int bytecount=0;
    memset(buffer, 0, buffer_len);

    cout << "esperando identificacion del proceso" << endl;


    printf("Received bytes %d\nReceived string \"%s\"\n", bytecount, buffer);

    //----------------------------------------------------------------


    while(1)
    {
        int   rc;
        char maki2[1024];

        //structToSend->send(hsock);
        //cout << "maki2 = " << strlen((const char*)maki2) << endl;
        int bytecount = 0;


        //temp->canRecv = rand() % 2 + -1;
        //temp->canSend = rand() % 2 + -1;

        rc = pthread_mutex_lock(&temp->mutex);

        if(temp->canRecv)
        {
            //cout << "canRecv es verdadero" << endl;
            if( (bytecount=recv(csock, (void*)maki2, 1024,0)) < 1)
            {
                fprintf(stderr, "Error sending data %d\n", errno);
                goto FINISH;
                //exitClient(csock);
            }
            //cout << bytecount << endl;
            structToRecv->Unserialize(maki2);
        }else
            cout << "canRecv es falso" << endl;


        // ----  poner datos en SHM  ----
        //
        //
        //
        // ----  coger datos de SHM  ----

        if(temp->canSend)
        {
            //cout << "canSend es verdadero" << endl;
            structToSend->serialize(maki2);

            if( (bytecount=send(csock, (void*)maki2, 1024,0)) < 1)
            {
                fprintf(stderr, "Error sending data %d\n", errno);
                goto FINISH;
                //exitClient(csock);
            }
            //cout << bytecount << endl;
        }else
            cout << "canSend es falso" << endl;

        temp->canRecv = false;
        temp->canSend = true;
        rc = pthread_mutex_unlock(&temp->mutex);

        printf("\n");
        usleep(usleepThread);

    }


FINISH:

	cout << "Close thread and connecion tcp" << endl;
    return 0;
}


int exitClient(int hsock)
{
    close(hsock);
    return 0;
}


