#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "parameters.h"

// codigo para el cliente TCP
//---------------------------------------------------------------------------------

#include <stdio.h>      /* for printf() and fprintf() */
#include <sys/socket.h> /* for socket(), connect(), send(), and recv() */
#include <arpa/inet.h>  /* for sockaddr_in and inet_addr() */
#include <stdlib.h>     /* for atoi() and exit() */
#include <string.h>     /* for memset() */
#include <unistd.h>     /* for close() */

#define RCVBUFSIZE 32   /* Size of receive buffer */

#include <stdio.h>  /* for perror() */
#include <stdlib.h> /* for exit() */


class TCP_Connection
{
    public:
        TCP_Connection();
        ~TCP_Connection();

        int ConnectToRTAI(char* ip,int port);
        int DisconnectRTAI();
        int SendToRTAI();
        int RecvFromRTAI();
        //variable
        int     sock;                        /* Socket descriptor */
        struct  sockaddr_in echoServAddr; /* Echo server address */
        unsigned short echoServPort;     /* Echo server port */
        char    *servIP;                    /* Server IP address (dotted quad) */
        char    *echoString;                /* String to send to echo server */
        char    echoBuffer[RCVBUFSIZE];     /* Buffer for echo string */
        unsigned int echoStringLen;      /* Length of string to echo */
        int     bytesRcvd, totalBytesRcvd;   /* Bytes read in single recv() and total bytes read */
        int counter;
        void DieWithError(char *errorMessage);
};

void TCP_Connection::DieWithError(char *errorMessage)
{
    perror(errorMessage);
    exit(1);
}

TCP_Connection::TCP_Connection()
{
    //servIP = "140.78.133.55";
    //servIP = "140.78.133.232"; // virtual machine
    servIP = "127.0.0.1"; // localhost
    echoString = "Test_sending";
    echoServPort = 8888;
    totalBytesRcvd = 0;
    counter = 0;
        /* Create a reliable, stream socket using TCP */
    if ((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
        DieWithError("socket() failed");
}

TCP_Connection::~TCP_Connection()
{
    close(sock);
}

int TCP_Connection::ConnectToRTAI(char* ip,int port)
{
    /* Construct the server address structure */
    memset(&echoServAddr, 0, sizeof(echoServAddr));     /* Zero out structure */
    echoServAddr.sin_family      = AF_INET;             /* Internet address family */
    echoServAddr.sin_addr.s_addr = inet_addr(servIP);   /* Server IP address */
    echoServAddr.sin_port        = htons(echoServPort); /* Server port */

    /* Establish the connection to the echo server */
    if (connect(sock, (struct sockaddr *) &echoServAddr, sizeof(echoServAddr)) < 0)
    {
         DieWithError("connect() failed");
    }

    return 1;

}
int TCP_Connection::DisconnectRTAI()
{
    close(sock);
}


int TCP_Connection::SendToRTAI()
{


    echoStringLen = strlen(echoString);          /* Determine input length */



    data_str b;
    b.indx_counter = counter;
    b.cos_value = rand();
    b.sin_value = rand();
    counter = counter + 1;

    int aux = sizeof(b);
    if (send(sock, &b, sizeof(b), 0) != sizeof(b))
        DieWithError("send() sent a different number of bytes than expected");


/*
    geometry_msgs::Point odom;
//    nav_msgs::Odometry odom;
//    data_str2 odom;
    int aux = sizeof(odom);

    data_str b;
    b.indx_counter = counter;
    b.cos_value = rand();
    b.sin_value = rand();
    counter = counter + 1;


    if (send(sock, &odom, sizeof(odom), 0) != sizeof(odom))
        DieWithError("send() sent a different number of bytes than expected");
*/

    /* Send the string to the server */
//    if (send(sock, echoString, echoStringLen, 0) != echoStringLen)
//        DieWithError("send() sent a different number of bytes than expected");

}

int TCP_Connection::RecvFromRTAI()
{
    while (totalBytesRcvd < echoStringLen)
    {


        data_str b;

        // Receive message from client
        if ((bytesRcvd = recv(sock, &b, sizeof(b), 0)) < 0)
            DieWithError("recv() failed");

        printf("------------------Received data------------------ \n");

        printf("b.indx_counter = %d \n", b.indx_counter);

        printf("b.cos_value = %f \n", b.cos_value);

        printf("b.sin_value = %f \n", b.sin_value);


        /*
     	printf("a.name = %s \n", a.name);

     	printf("a.angular.x = %lf \n", a.angular.x);
     	printf("a.angular.y = %lf \n", a.angular.y);
     	printf("a.angular.z = %lf \n", a.angular.z);

     	printf("a.linear.x = %lf \n", a.linear.x);
     	printf("a.linear.y = %lf \n", a.linear.y);
     	printf("a.linear.z = %lf \n", a.linear.z);
*/
        totalBytesRcvd += bytesRcvd;   // Keep tally of total bytes

/*
        // Receive up to the buffer size (minus 1 to leave space for
        //   a null terminator) bytes from the sender

        if ((bytesRcvd = recv(sock, echoBuffer, RCVBUFSIZE - 1, 0)) <= 0)
            DieWithError("recv() failed or connection closed prematurely");
        totalBytesRcvd += bytesRcvd;   // Keep tally of total bytes
        echoBuffer[bytesRcvd] = '\0';  // Terminate the string!
        printf("maki %s \n", echoBuffer);      // Print the echo buffer
*/
    }
    totalBytesRcvd = 0;
}
/*
int main2()
{
    int sock;                        // Socket descriptor
    struct sockaddr_in echoServAddr; // Echo server address
    unsigned short echoServPort;     // Echo server port
    char *servIP;                    // Server IP address (dotted quad)
    char *echoString;                // String to send to echo server
    char echoBuffer[RCVBUFSIZE];     // Buffer for echo string
    unsigned int echoStringLen;      // Length of string to echo
    int bytesRcvd, totalBytesRcvd;   // Bytes read in single recv()
                                        and total bytes read

//    if ((argc < 3) || (argc > 4))    // Test for correct number of arguments
//    {
//       fprintf(stderr, "Usage: %s <Server IP> <Echo Word> [<Echo Port>]\n",
//               argv[0]);
//       exit(1);
//    }

    servIP = "127.0.0.1"; //argv[1];             // First arg: server IP address (dotted quad)
    echoString = "maki"; //argv[2];         // Second arg: string to echo

    //if (argc == 4)
    //    echoServPort = atoi(argv[3]); // Use given port, if any
    //else
        echoServPort = 8888;//7;  // 7 is the well-known port for the echo service

    // Create a reliable, stream socket using TCP
    if ((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
        DieWithError("socket() failed");

    // Construct the server address structure
    memset(&echoServAddr, 0, sizeof(echoServAddr));     // Zero out structure
    echoServAddr.sin_family      = AF_INET;             // Internet address family
    echoServAddr.sin_addr.s_addr = inet_addr(servIP);   // Server IP address
    echoServAddr.sin_port        = htons(echoServPort); // Server port

    // Establish the connection to the echo server
    if (connect(sock, (struct sockaddr *) &echoServAddr, sizeof(echoServAddr)) < 0)
        DieWithError("connect() failed");

    echoStringLen = strlen(echoString);          // Determine input length

    // Send the string to the server
    if (send(sock, echoString, echoStringLen, 0) != echoStringLen)
        DieWithError("send() sent a different number of bytes than expected");

    // Receive the same string back from the server
    totalBytesRcvd = 0;
    printf("Received: ");                // Setup to print the echoed string
    while (totalBytesRcvd < echoStringLen)
    {
        // Receive up to the buffer size (minus 1 to leave space for
        // a null terminator) bytes from the sender
        if ((bytesRcvd = recv(sock, echoBuffer, RCVBUFSIZE - 1, 0)) <= 0)
            DieWithError("recv() failed or connection closed prematurely");
        totalBytesRcvd += bytesRcvd;   // Keep tally of total bytes
        echoBuffer[bytesRcvd] = '\0';  // Terminate the string!
        printf("%s", echoBuffer);      // Print the echo buffer
    }

    printf("\n");    // Print a final linefeed

   // close(sock);
    exit(0);
}
*/
//---------------------------------------------------------------------------------


int main(int argc, char** argv){

    char *publishName = "odom";

    ros::init(argc, argv, "RTAI_node");

    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>(publishName,50); //"odom", 50);

    tf::TransformBroadcaster odom_broadcaster;

    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    double vx = 0.1;
    double vy = -0.1;
    double vth = 0.1;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate r(1);

    //main2();
    TCP_Connection::TCP_Connection RTAI;

    RTAI.ConnectToRTAI("127.0.0.1",8888);
    //  RTAI.SendToRTAI();
    //  RTAI.RecvFromRTAI();
    //  RTAI.DisconnectRTAI();
    while(n.ok()){
        current_time = ros::Time::now();

        //compute odometry in a typical way given the velocities of the robot
        double dt = (current_time - last_time).toSec();
        double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        double delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;


        //publish the message
        odom_pub.publish(odom);

        RTAI.SendToRTAI();
        RTAI.RecvFromRTAI();

        last_time = current_time;
        r.sleep();
    }
}

