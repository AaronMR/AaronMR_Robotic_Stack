#include "parameters.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include "parser.h"



//-----------------------------------Types---------------------------------------------
class structType {
public:
    virtual char *serialize(char* maki) = 0;
    virtual char *Unserialize(char* maki) = 0;
    void storeData();
};

class struct_Joy : public structType {
public:
    struct_Joy();

    char* serialize(char* maki);
    char* Unserialize(char* maki);
    void storeData(Joy *joy);
    Joy auxJoy1;
    int sizeof_Joy;
    bool haveSubscriber;
    bool havePublisher;

};

class struct_Twist : public structType {
public:
    struct_Twist();
    char* serialize(char* maki);
    char* Unserialize(char* maki);
    void storeData(Joy *joy);
    Joy auxJoy1;
    int sizeof_Joy;
    bool haveSubscriber;
    bool havePublisher;

};

class struct_Pose : public structType {
public:
    struct_Pose();
    char* serialize(char* maki);
    char* Unserialize(char* maki);
    void storeData(Joy *joy);
    Joy auxJoy1;

    Pose auxPose1;
    int sizeof_Joy;
    bool haveSubscriber;
    bool havePublisher;

};


class AaronMR_S
{
public:
    AaronMR_S(char * aux);
    ~AaronMR_S();
    int create_Socket();
    int connect_Socket();
    //int send_Data();
    //int recv_Data();
    //structType* structToSend;
    //structType* structToRecv;

    //----------------- para el server -----------------
    socklen_t addr_size;
	int csock;
	pthread_t thread_id;
    int hsock;
    sockaddr_in sadr;
    DataLayout configuration[5];

    static void* SocketHandler( void* args ) ;

    int acceptConnection();

    int waitEvent();

    // for the nonblocking sockets
    fd_set master;    // master file descriptor list
    fd_set read_fds;  // temp file descriptor list for select()
    int fdmax;        // maximum file descriptor number

    int listener;     // listening socket descriptor
    int newfd;        // newly accept()ed socket descriptor
    socklen_t addrlen;
    struct sockaddr_storage remoteaddr; // client address
    char remoteIP[INET6_ADDRSTRLEN];

    int startThread(int csock);

private:
    int host_port;
    char* host_name;
    char* Subscriber_name;
    char* Publisher_name;
    struct sockaddr_in my_addr;


    //DataLayout configuration[5];
    int buffer_len;
    char buffer[1024];
    int numProcess;


};



