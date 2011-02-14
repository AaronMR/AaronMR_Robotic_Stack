#include "AaronMR_C.hpp"
#include <iostream>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string>

#include "pack2.cpp"

#include <fcntl.h>


#include <sys/poll.h>





//##############################################################################
/*----------------------------------------------------------------------
 Portable function to set a socket into nonblocking mode.
 Calling this on a socket causes all future read() and write() calls on
 that socket to do only as much as they can immediately, and return
 without waiting.
 If no data can be read or written, they return -1 and set errno
 to EAGAIN (or EWOULDBLOCK).
 Thanks to Bjorn Reese for this code.
----------------------------------------------------------------------*/
int setNonblocking(int fd)
{
    int flags;

    /* If they have O_NONBLOCK, use the Posix way to do it */
#if defined(O_NONBLOCK)
    /* Fixme: O_NONBLOCK is defined but broken on SunOS 4.1.x and AIX 3.2.5. */
    if (-1 == (flags = fcntl(fd, F_GETFL, 0)))
        flags = 0;
    return fcntl(fd, F_SETFL, flags | O_NONBLOCK);
#else
    /* Otherwise, use the old way of doing it */
    flags = 1;
    return ioctl(fd, FIOBIO, &flags);
#endif
}

//##############################################################################

int exitClient(int hsock);


/** \brief Constructor
 *
 * \param file url
 * \param
 * \return
 *
 */

AaronMR_C::AaronMR_C(char * aux)
{

    int counter = 0;
    int Node2RTAI = 0;
    int RTAI2Node = 0;

//    configuration[0] = new DataLayout;
//    configuration[1] = new DataLayout;
//    configuration[2] = new DataLayout;
//    configuration[3] = new DataLayout;
//    configuration[4] = new DataLayout;
// el error esta en esta parte
    DataLayout auxFile[1];

    parseFile parser(aux);
    parser.Parse(configuration);
//-----------------------------
/*
    configuration.active = auxFile[0].active;
    configuration.csock = auxFile[0].csock;
    configuration.IP_RTAI = auxFile[0].IP_RTAI;
    configuration.name = auxFile[0].name;
    configuration.Node2RTAI = auxFile[0].Node2RTAI;
    configuration.PORT_RTAI = auxFile[0].PORT_RTAI;
    configuration.Publisher = auxFile[0].Publisher;
    configuration.RTAI2Node = auxFile[0].RTAI2Node;
    configuration.SHM = auxFile[0].SHM;
    configuration.Subscriber = auxFile[0].Subscriber;
*/

/*
    configuration.active = 0;
    configuration.csock = 0;
    configuration.IP_RTAI = "127.0.0.1";
    configuration.IP_RTAI = "140.78.133.43";
    configuration.name = "Process_3";
    configuration.Node2RTAI = "Joy";
    configuration.PORT_RTAI = "1101";
    configuration.Publisher = "maki2";
    configuration.RTAI2Node = "Joy";
    configuration.SHM = "SHM_3";
    configuration.Subscriber = "joy";
*/

//    configuration.active =      parser.active;
//    configuration.csock =       parser.csock;

    //printf("%s\n",auxFile[0].name.c_str());
    //configuration.IP_RTAI =     parser.IP_RTAI;
    //configuration.name =        auxFile[0].name.c_str();
    //configuration.Node2RTAI =   parser.Node2RTAI;
    //configuration.PORT_RTAI =   parser.PORT_RTAI;
    //configuration.Publisher =   parser.Publisher;
    //configuration.RTAI2Node =   parser.RTAI2Node;
    //configuration.SHM =         parser.SHM;
    //configuration.Subscriber =  parser.Subscriber;






    //vel_pub  = n.advertise<turtlesim::Velocity>("prueba_publicacion", 1);


//--------------------------------------------------
    host_port = atoi(configuration[0].PORT_RTAI.data());
    host_name = (char*)configuration[0].IP_RTAI.data();
    Subscriber_name = (char*)configuration[0].Subscriber.data();
    Publisher_name = (char*)configuration[0].Publisher.data();


    my_addr.sin_family = AF_INET ;
    my_addr.sin_port = htons(host_port);

    memset(&(my_addr.sin_zero), 0, 8);
    my_addr.sin_addr.s_addr = inet_addr(host_name);




//    const char* Node2RTAI_ = configuration[0].Node2RTAI.data();


   if(configuration[0].Node2RTAI.compare("Twist") == 0)
    {
        Node2RTAI = 4;
        structToSend = new struct_Twist;
        structToSend->set_Subscriber("twist");

    }else if(configuration[0].Node2RTAI.compare("Odometry") == 0)
    {
        Node2RTAI = 5;
        structToSend = new struct_Joy;

    }else if(configuration[0].Node2RTAI.compare("Joy") == 0)
    {
        Node2RTAI = 6;
        structToSend = new struct_Joy;
        structToSend->set_Subscriber("joy");

    }else if(configuration[0].Node2RTAI.compare("Pose") == 0)
    {
        Node2RTAI = 6;
        structToSend = new struct_Pose;
        structToSend->set_Subscriber("odom");

    }

//    const char* RTAI2Node_ = configuration[0].RTAI2Node.data();


    if(configuration[0].RTAI2Node.compare("Twist") == 0)
    {
        RTAI2Node = 4;
        structToRecv = new struct_Twist;
        structToRecv->set_Publisher("Twist_pub");

    }else if(configuration[0].RTAI2Node.compare("Odometry") == 0)
    {
        RTAI2Node = 5;
        structToRecv = new struct_Joy;

    }else if(configuration[0].RTAI2Node.compare("Joy") == 0)
    {
        RTAI2Node = 6;
        structToRecv = new struct_Joy;
        structToRecv->set_Publisher("joy_pub");
    }else if(configuration[0].RTAI2Node.compare("Pose") == 0)
    {
        RTAI2Node = 6;
        structToRecv = new struct_Pose;
        structToRecv->set_Publisher("odom_pub");
    }

        //mutex
    configuration[0].mutex = PTHREAD_MUTEX_INITIALIZER;
    configuration[0].canRecv = false;
    configuration[0].canSend = false;

    counter = counter + 1;

    buffer_len = 1024;
    memset(buffer, '\0', buffer_len);

    //memcpy(buffer, process[0].name.data(), process[0].name.length() );
    //cout << "The name of the service is = " << buffer << endl;

    //create_Socket();
    //connect_Socket();
};


int AaronMR_C::SendRecv()
{
    int s1, s2;
    int rv;
    char buf1[256], buf2[256];
    pollfd ufds[2];
    s1 = socket(PF_INET, SOCK_STREAM, 0);
    s2 = socket(PF_INET, SOCK_STREAM, 0);
    // pretend we've connected both to a server at this point
    //connect(s1, ...)...
    //connect(s2, ...)...
    // set up the array of file descriptors.
    //
    // in this example, we want to know when there's normal or out-of-band
    // data ready to be recv()'d...
    ufds[0].fd = s1;
    ufds[0].events = POLLIN | POLLPRI; // check for normal or out-of-band
    ufds[1].fd = s2;
    ufds[1].events = POLLIN; // check for just normal data
    // wait for events on the sockets, 3.5 second timeout
    rv = poll(ufds, 2, 30500);

    if (rv == -1)
    {
        perror("poll"); // error occurred in poll()

    } else if (rv == 0)
    {
        printf("Timeout occurred! No data after 3.5 seconds.\n");

    } else {
        // check for events on s1:
        if (ufds[0].revents & POLLIN)
        {
            recv(s1, buf1, sizeof buf1, 0); // receive normal data
        }
        if (ufds[0].revents & POLLPRI)
        {
            recv(s1, buf1, sizeof buf1, MSG_OOB); // out-of-band data
        }
        // check for events on s2:
        if (ufds[1].revents & POLLIN)
        {
            recv(s1, buf2, sizeof buf2, 0);
        }
    }

    return 0;
}

int AaronMR_C::send_Data()
{

    char maki2[1024];

    structToSend->spinOnce();

    //structToSend->send(hsock);
    //cout << "maki2 = " << strlen((const char*)maki2) << endl;
    int bytecount = 0;


    if(structToSend->canSend())
    {

        //cout << "puedo enviar" << endl;
        structToSend->serialize(maki2);
        if( (bytecount=send(hsock, (const void*)maki2, 1024,0)) < 1)
        {
            fprintf(stderr, "send_Data Error sending data %d\n", errno);
            //goto FINISH;
            exitClient(hsock);
            return -1;
        }

    }
    //else
    //    cout << "NO puedo enviar" << endl;

    return 0;

    //cout << bytecount << endl;

//    unsigned char magic;

//    double maki[9];
//    unsigned int ps2;

//    Joy auxJoy2;
//	unpack((unsigned char*)maki2, "Cddddd", &magic, &maki[0], &maki[1], &maki[2], &maki[3], &maki[4]);
//    unpack((unsigned char*)maki2, "Cddddd", &magic, &maki[0], &maki[1], &maki[2], &maki[3], &maki[4]);

//    printf("'%c' %f %f %f %f %f\n", magic, maki[0], maki[1], maki[2], maki[3], maki[4]);

/*
	unpack((unsigned char*)maki2, "CHffffhhhh",  &magic,
                                            &ps2,
                                            &auxJoy2.axes[0],
                                            &auxJoy2.axes[1],
                                            &auxJoy2.axes[2],
                                            &auxJoy2.axes[3],
                                            &auxJoy2.buttons[0],
                                            &auxJoy2.buttons[1],
                                            &auxJoy2.buttons[2],
                                            &auxJoy2.buttons[3]);

	printf("'%c' %hhu %f %f %f %f %d %d %d %d\n",   magic,
                                                    ps2,
                                                    auxJoy2.axes[0],
                                                    auxJoy2.axes[1],
                                                    auxJoy2.axes[2],
                                                    auxJoy2.axes[3],
                                                    auxJoy2.buttons[0],
                                                    auxJoy2.buttons[1],
                                                    auxJoy2.buttons[2],
                                                    auxJoy2.buttons[3]);
*/

};

int AaronMR_C::recv_Data()
{
    char maki2[1024];

    //structToSend->send(hsock);
    //cout << "maki2 = " << strlen((const char*)maki2) << endl;
    int bytecount = 0;

    if( (bytecount=recv(hsock, (void*)maki2, 1024,0)) < 1)
    {
        fprintf(stderr, "recv_Data Error reciving data %d\n", errno);
        //goto FINISH;
        exitClient(hsock);
        return -1;
    }
    //cout << bytecount << endl;


    structToRecv->Unserialize(maki2);

    return 0;
}



int AaronMR_C::create_Socket()
{
    hsock = socket(AF_INET, SOCK_STREAM, 0);
    //fcntl(hsock, F_SETFL, O_NONBLOCK);
    //setNonblocking(hsock);
    if(hsock == -1)
    {
        printf("Error initializing socket %d\n",errno);
        //goto FINISH;
        return -1;
    }
    return hsock;
}

int AaronMR_C::connect_Socket()
{
    int err;
    int bytecount = 0;
    if( connect( hsock, (struct sockaddr*)&my_addr, sizeof(my_addr)) == -1 )
    {
        if((err = errno) != EINPROGRESS)
        {
            fprintf(stderr, "Error connecting socket %d\n", errno);
            //goto FINISH;
            exitClient(hsock);
            return -1;
        }
    }

    //Now lets do the client related stuff

    buffer_len = 1024;

    memset(buffer, '\0', buffer_len);

    //cout << configuration[0].name.size()<<endl;
    //cout << configuration[0].name<<endl;

    //memcpy(buffer, configuration[0].name.data(), configuration[0].name.size() );

    //configuration.name.copy(buffer,configuration.name.size(),0);
    strcpy(buffer, configuration[0].name.c_str());

    //##########################################################################
    //cout << "The name of the service is = " << buffer << endl;



    char maki2[1024];
    SetupCode(maki2);
    //structToSend->send(hsock);
    //cout << "maki2 = " << strlen((const char*)maki2) << endl;

    if( (bytecount=send(hsock, (const void*)maki2, 1024,0)) < 1)
    {
        fprintf(stderr, "send_Data Error sending data %d\n", errno);
        //goto FINISH;
        exitClient(hsock);
        return -1;
    }
    //##########################################################################

    // the setup message is: S "size_msg" "name_process" "Node2RTAI" "RTAI2Node"
    // example: S "size_msg" Process_1 Joy Joy

    //if( (bytecount=send(hsock, buffer, buffer_len,0)) < 1)
    //{
    //    fprintf(stderr, "Error sending data %d\n", errno);
        //goto FINISH;
    //    exitClient(hsock);
    //}

    memset(buffer, '\0', buffer_len);

    if((bytecount = recv(hsock, buffer, buffer_len, 0)) < 1)
    {
        fprintf(stderr, "Error receiving data %d\n", errno);
        //goto FINISH;
        exitClient(hsock);
    }

    unsigned char magic;
    unsigned int ps2;
    char s2[96];

    unpack((unsigned char*)buffer, "CHs",   &magic,
                                            &ps2,
                                            &s2);

    printf("recv: '%c' %hhu %s\n",  magic,
                                    ps2,
                                    s2);

    if(magic == 69)
    {
        cout << "se cerro"<< endl;
        //goto FINISH;
        exitClient(hsock);
        return -1;
    }
    return 0;
}


char* AaronMR_C::nameProcess()
{
    return (char*)"cosita_1";
}

AaronMR_C::~AaronMR_C()
{
//    int a=0;
};

int AaronMR_C::SetupCode(char* buffer)
{
    unsigned char buf3[1024];
	unsigned char buf[1024];
//	unsigned char buf2[1024]="makiboludo";
	unsigned char magic;
//	int monkeycount;
//	long altitude;
//	double absurdityfactor;
//	char *s = "Maki";
//	char s2[96];
	unsigned int packetsize, ps2;
//    double maki[9];



    cout << configuration->name.data() << endl;
    cout << configuration->Node2RTAI.data() << endl;
    cout << configuration->RTAI2Node.data() << endl;



    char *s = "Maki";
    char s2[96];
    char s3[96];
    char s4[96];

	packetsize = pack(buf, "CHsss", 'S', 0, configuration->name.c_str(),
                                            configuration->Node2RTAI.c_str(),
                                            configuration->RTAI2Node.c_str());

	packi16(buf+1, packetsize);

    memcpy((unsigned char*)buffer, buf, packetsize);

	unpack((unsigned char*)buffer, "CHsss",    &magic,
                                            &ps2,
                                            &s2,
                                            &s3,
                                            &s4);

	printf("send: '%c' %hhu %s %s %s\n",   magic,
                                     ps2,
                                     s2,
                                     s3,
                                     s4);

    return NULL;
}

int exitClient(int hsock)
{
    close(hsock);
    return 0;
    //exit(1);
}


