#include "transmition.h"
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <errno.h>
#include <iostream>

#include "SocketException.h"


#define DEBUG

using namespace std;

transmision::transmision(int aux)
{
    sock = aux;
}

int transmision::sendData(Struct_1 *a)
{
#ifdef DEBUG
    cout << "\n transmision::sendData(Struct_1 *a) \n" << endl;
#endif

    //char buffer[1024];
    int buffer_len = 1024;
    int bytecount;
//    memset(buffer, 0, buffer_len);
    //strcpy(buffer, "yujuuuu");

    cout << "sock = " << sock << endl;

    try
    {
        if((bytecount = send(sock, a, sizeof(Struct_1), 0)) < 1)
        {
            fprintf(stderr, "Error sending data %d\n", errno);
            //goto FINISH;
            return -1;
        }
    }
    catch ( SocketException& )
    {
        cout << "un error aqui_4" << endl;
    }




    cout << "   --- Datos enviados---"<< endl;
    //cout << "       bytecount = " << bytecount << endl;
    cout << "       a.indx_counter = " << a->indx_counter << endl;
    cout << "       a.cos_value = " << a->cos_value << endl;
    cout << "       a.sin_value = " << a->sin_value << endl;
    return bytecount;
}

int transmision::recvData(Struct_1 *a)
{
#ifdef DEBUG
    cout << "\n transmision::recvData(Struct_1 *a) \n" << endl;
#endif
    //char buffer[1024];
    int buffer_len = 1024;
    int bytecount;
//    memset(buffer, 0, buffer_len);
    //strcpy(buffer, "yujuuuu");

    cout << "sock = " << sock << endl;
    try
    {

        if((bytecount = recv(sock, a, sizeof(Struct_1), 0)) < 1)
        {
            fprintf(stderr, "Error sending data %d\n", errno);
            //goto FINISH;
            return -1;
        }

    }
    catch ( SocketException& )
    {
        cout << "un error aqui_3" << endl;
    }


    cout << "   --- Datos recibidos---"<< endl;
    //cout << "       bytecount = " << bytecount << endl;
    cout << "       a.indx_counter = " << a->indx_counter << endl;
    cout << "       a.cos_value = " << a->cos_value << endl;
    cout << "       a.sin_value = " << a->sin_value << endl;
    return bytecount;
}


// Structura tipo 2
int transmision::sendData(Struct_2 *b)
{

#ifdef DEBUG
    cout << "\n transmision::sendData(Struct_2 *b) \n" << endl;
#endif

    //char buffer[1024];
    int buffer_len = 1024;
    int bytecount;
//    memset(buffer, 0, buffer_len);
    //strcpy(buffer, "yujuuuu");

    cout << "sock = " << sock << endl;

    try
    {

        if((bytecount = send(sock, b, sizeof(Struct_2), 0)) < 1)
        {
            fprintf(stderr, "Error sending data %d\n", errno);
//          goto FINISH;
            return -1;
        }
    }
    catch ( SocketException& )
    {
        cout << "un error aqui_2" << endl;
    }


    cout << "   --- Datos enviados---"<< endl;
    //cout << "       bytecount = " << bytecount << endl;
    cout << "       b.indx_counter = " << b->indx_counter << endl;
    cout << "       b.cos_value = " << b->cos_value << endl;
    cout << "       b.sin_value = " << b->sin_value << endl;
    cout << "       b.>new_value_1 = " << b->new_value_1 << endl;
    cout << "       b.>new_value_2 = " << b->new_value_2 << endl;
    cout << "       b.>new_value_3 = " << b->new_value_3 << endl;
    cout << "       b.>new_value_4 = " << b->new_value_4 << endl;
    return bytecount;
}

int transmision::recvData(Struct_2 *b)
{

#ifdef DEBUG
    cout << "\n transmision::recvData(Struct_2 *b) \n" << endl;
#endif

    //char buffer[1024];
    int buffer_len = 1024;
    int bytecount;
//    memset(buffer, 0, buffer_len);
    //strcpy(buffer, "yujuuuu");
    try
    {
        if((bytecount = recv(sock, b, sizeof(Struct_2), 0)) < 1)
        {
            fprintf(stderr, "Error sending data %d\n", errno);
//          goto FINISH;
            return -1;
        }

    }
    catch ( SocketException& )
    {
        cout << "un error aqui_1" << endl;
    }


    cout << "   --- Datos recibidos---"<< endl;
    //cout << "       bytecount = " << bytecount << endl;
    cout << "       b.indx_counter = " << b->indx_counter << endl;
    cout << "       b.cos_value = " << b->cos_value << endl;
    cout << "       b.sin_value = " << b->sin_value << endl;
    cout << "       b.>new_value_1 = " << b->new_value_1 << endl;
    cout << "       b.>new_value_2 = " << b->new_value_2 << endl;
    cout << "       b.>new_value_3 = " << b->new_value_3 << endl;
    cout << "       b.>new_value_4 = " << b->new_value_4 << endl;
    return bytecount;
}


//------------------------------- Structure type 3----------------------
int transmision::sendData(Struct_3 *c)
{

#ifdef DEBUG
    cout << "\n transmision::sendData(Struct_3 *c) \n" << endl;
#endif

    //char buffer[1024];
    int buffer_len = 1024;
    int bytecount;
//    memset(buffer, 0, buffer_len);
    //strcpy(buffer, "yujuuuu");

    try
    {

        if((bytecount = send(sock, c, sizeof(Struct_3), 0)) < 1)
        {
            fprintf(stderr, "Error sending data %d\n", errno);
//          goto FINISH;
            return -1;
        }
    }
    catch ( SocketException& )
    {
        cout << "un error aqui_2" << endl;
    }


    cout << "   --- Datos enviados---"<< endl;
    //cout << "       bytecount = " << bytecount << endl;
    cout << "       c.axes[0] = " << c->axes[0] << endl;
    cout << "       c.axes[1] = " << c->axes[1] << endl;
    cout << "       c.axes[2] = " << c->axes[2] << endl;
    cout << "       c.axes[3] = " << c->axes[3] << endl;

    cout << "       c.buttons[0] = " << c->buttons[0] << endl;
    cout << "       c.buttons[1] = " << c->buttons[1] << endl;
    cout << "       c.buttons[2] = " << c->buttons[2] << endl;
    cout << "       c.buttons[3] = " << c->buttons[3] << endl;

    return bytecount;
}

int transmision::recvData(Struct_3 *c)
{

#ifdef DEBUG
    cout << "\n transmision::recvData(Struct_3 *c) \n" << endl;
#endif

    //char buffer[1024];
    int buffer_len = 1024;
    int bytecount;
//    memset(buffer, 0, buffer_len);
    //strcpy(buffer, "yujuuuu");
    try
    {
        if((bytecount = recv(sock, c, sizeof(Struct_3), 0)) < 1)
        {
            fprintf(stderr, "Error sending data %d\n", errno);
//          goto FINISH;
            return -1;
        }

    }
    catch ( SocketException& )
    {
        cout << "un error aqui_1" << endl;
    }


    cout << "   --- Datos recibidos---"<< endl;
    //cout << "       bytecount = " << bytecount << endl;
    cout << "       c.axes[0] = " << c->axes[0] << endl;
    cout << "       c.axes[1] = " << c->axes[1] << endl;
    cout << "       c.axes[2] = " << c->axes[2] << endl;
    cout << "       c.axes[3] = " << c->axes[3] << endl;

    cout << "       c.buttons[0] = " << c->buttons[0] << endl;
    cout << "       c.buttons[1] = " << c->buttons[1] << endl;
    cout << "       c.buttons[2] = " << c->buttons[2] << endl;
    cout << "       c.buttons[3] = " << c->buttons[3] << endl;
    return bytecount;
}


//------------------------------- Structure Joy------------------------
int transmision::sendData(Joy *joy)
{

#ifdef DEBUG
    cout << "\n transmision::sendData(Joy *joy) \n" << endl;
#endif

    //char buffer[1024];
    int buffer_len = 1024;
    int bytecount;
//    memset(buffer, 0, buffer_len);
    //strcpy(buffer, "yujuuuu");

    try
    {

        if((bytecount = send(sock, joy, sizeof(Joy), 0)) < 1)
        {
            fprintf(stderr, "Error sending data %d\n", errno);
//          goto FINISH;
            return -1;
        }
    }
    catch ( SocketException& )
    {
        cout << "un error aqui_2" << endl;
    }


    cout << "   --- Datos enviados---"<< endl;
    //cout << "       bytecount = " << bytecount << endl;
    cout << "       joy.axes[0] = " << joy->axes[0] << endl;
    cout << "       joy.axes[1] = " << joy->axes[1] << endl;
    cout << "       joy.axes[2] = " << joy->axes[2] << endl;
    cout << "       joy.axes[3] = " << joy->axes[3] << endl;

    cout << "       joy.buttons[0] = " << joy->buttons[0] << endl;
    cout << "       joy.buttons[1] = " << joy->buttons[1] << endl;
    cout << "       joy.buttons[2] = " << joy->buttons[2] << endl;
    cout << "       joy.buttons[3] = " << joy->buttons[3] << endl;

    return bytecount;
}

int transmision::recvData(Joy *joy)
{

#ifdef DEBUG
    cout << "\n transmision::recvData(Joy *joy) \n" << endl;
#endif

    //char buffer[1024];
    int buffer_len = 1024;
    int bytecount;

//    memset(buffer, 0, buffer_len);
    //strcpy(buffer, "yujuuuu");
    try
    {
        if((bytecount = recv(sock, joy, sizeof(Joy), 0)) < 1)

        {
            fprintf(stderr, "Error sending data %d\n", errno);
//          goto FINISH;

        }

    }
    catch ( SocketException& )
    {
        cout << "un error aqui_1" << endl;

    }



//-----------------
    cout << "   --- Datos recibidos---"<< endl;
    //cout << "       bytecount = " << bytecount << endl;
    cout << "       joy.axes[0] = " << joy->axes[0] << endl;
    cout << "       joy.axes[1] = " << joy->axes[1] << endl;
    cout << "       joy.axes[2] = " << joy->axes[2] << endl;
    cout << "       joy.axes[3] = " << joy->axes[3] << endl;

    cout << "       joy.buttons[0] = " << joy->buttons[0] << endl;
    cout << "       joy.buttons[1] = " << joy->buttons[1] << endl;
    cout << "       joy.buttons[2] = " << joy->buttons[2] << endl;
    cout << "       joy.buttons[3] = " << joy->buttons[3] << endl;
    return bytecount;
}

int transmision::recvData(char *buffer)
{

#ifdef DEBUG
    cout << "\n transmision::recvData(char *buffer) \n" << endl;
#endif

    //char buffer[1024];
    int buffer_len = 1024;
    int bytecount;

//    memset(buffer, 0, buffer_len);
    //strcpy(buffer, "yujuuuu");
    try
    {
        if((bytecount = recv(sock, buffer, 1024, 0)) < 1)

        {
            fprintf(stderr, "Error sending data %d\n", errno);
//          goto FINISH;

        }

    }
    catch ( SocketException& )
    {
        cout << "un error aqui_1" << endl;

    }



//-----------------
    cout << "   --- Datos recibidos---"<< endl;
    //cout << "       bytecount = " << bytecount << endl;
/*
    cout << "       joy.axes[0] = " << joy->axes[0] << endl;
    cout << "       joy.axes[1] = " << joy->axes[1] << endl;
    cout << "       joy.axes[2] = " << joy->axes[2] << endl;
    cout << "       joy.axes[3] = " << joy->axes[3] << endl;

    cout << "       joy.buttons[0] = " << joy->buttons[0] << endl;
    cout << "       joy.buttons[1] = " << joy->buttons[1] << endl;
    cout << "       joy.buttons[2] = " << joy->buttons[2] << endl;
    cout << "       joy.buttons[3] = " << joy->buttons[3] << endl;
*/
    return bytecount;
}

int transmision::sendData(char *buffer)
{

#ifdef DEBUG
    cout << "\n transmision::sendData(char *buffer) \n" << endl;
#endif

    //char buffer[1024];
    int buffer_len = 1024;
    int bytecount;

//    memset(buffer, 0, buffer_len);
    //strcpy(buffer, "yujuuuu");
    try
    {
        if((bytecount = send(sock, buffer, 1024, 0)) < 1)

        {
            fprintf(stderr, "Error sending data %d\n", errno);
//          goto FINISH;

        }

    }
    catch ( SocketException& )
    {
        cout << "un error aqui_1" << endl;

    }



//-----------------
    cout << "   --- Datos recibidos---"<< endl;
    //cout << "       bytecount = " << bytecount << endl;
/*
    cout << "       joy.axes[0] = " << joy->axes[0] << endl;
    cout << "       joy.axes[1] = " << joy->axes[1] << endl;
    cout << "       joy.axes[2] = " << joy->axes[2] << endl;
    cout << "       joy.axes[3] = " << joy->axes[3] << endl;

    cout << "       joy.buttons[0] = " << joy->buttons[0] << endl;
    cout << "       joy.buttons[1] = " << joy->buttons[1] << endl;
    cout << "       joy.buttons[2] = " << joy->buttons[2] << endl;
    cout << "       joy.buttons[3] = " << joy->buttons[3] << endl;
*/
    return bytecount;
}

//------------------------------- Structure Twist-------------------------
int transmision::sendData(Twist *twist)
{

#ifdef DEBUG
    cout << "\n transmision::sendData(Twist *twist) \n" << endl;
#endif

    //char buffer[1024];
    int buffer_len = 1024;
    int bytecount;
//    memset(buffer, 0, buffer_len);
    //strcpy(buffer, "yujuuuu");

    try
    {

        if((bytecount = send(sock, twist, sizeof(Twist), 0)) < 1)
        {
            fprintf(stderr, "Error sending data %d\n", errno);
//          goto FINISH;
            return -1;
        }
    }
    catch ( SocketException& )
    {
        cout << "un error aqui_2" << endl;
    }


    cout << "   --- Datos enviados---"<< endl;
    //cout << "       bytecount = " << bytecount << endl;
    cout << "       twist->angular->x = " << twist->angular.x << endl;
    cout << "       twist->angular->y = " << twist->angular.y << endl;
    cout << "       twist->angular->z = " << twist->angular.z << endl;

    cout << "       twist->linear->x = " << twist->linear.x << endl;
    cout << "       twist->linear->y = " << twist->linear.y << endl;
    cout << "       twist->linear->z = " << twist->linear.z << endl;

    return bytecount;
}

int transmision::recvData(Twist *twist)
{

#ifdef DEBUG
    cout << "\n transmision::recvData(Twist *twist) \n" << endl;
#endif

    //char buffer[1024];
    int buffer_len = 1024;
    int bytecount;
//    memset(buffer, 0, buffer_len);
    //strcpy(buffer, "yujuuuu");
    try
    {
        if((bytecount = recv(sock, twist, sizeof(Twist), 0)) < 1)
        {
            fprintf(stderr, "Error sending data %d\n", errno);
//          goto FINISH;
            return -1;
        }

    }
    catch ( SocketException& )
    {
        cout << "un error aqui_1" << endl;
    }


    cout << "   --- Datos recibidos---"<< endl;
    //cout << "       bytecount = " << bytecount << endl;
    cout << "       \t %d\tA\t twist->angular->x = " << twist->angular.x << endl;
    cout << "       twist->angular->y = " << twist->angular.y << endl;
    cout << "       twist->angular->z = " << twist->angular.z << endl;

    cout << "       twist->linear->x = " << twist->linear.x << endl;
    cout << "       twist->linear->y = " << twist->linear.y << endl;
    cout << "       twist->linear->z = " << twist->linear.z << endl;

    return bytecount;
}


//------------------------------- Structure Odometry-------------------------
int transmision::sendData(Odometry *odometry)
{

#ifdef DEBUG
    cout << "\n transmision::sendData(Odometry *odometry) \n" << endl;
#endif

    //char buffer[1024];
    int buffer_len = 1024;
    int bytecount;
//    memset(buffer, 0, buffer_len);
    //strcpy(buffer, "yujuuuu");

    try
    {

        if((bytecount = send(sock, odometry, sizeof(Odometry), 0)) < 1)
        {
            fprintf(stderr, "Error sending data %d\n", errno);
//          goto FINISH;
            return -1;
        }
    }
    catch ( SocketException& )
    {
        cout << "un error aqui_2" << endl;
    }


    cout << "   --- Datos enviados---"<< endl;
    //cout << "       bytecount = " << bytecount << endl;
//    cout << "odometry.child_frame_id = " << odometry->child_frame_id << endl;

    cout << "odometry.pose.covariance[0] = " << odometry->pose.covariance[0] << endl;
    cout << "odometry.pose.pose.orientation.w = " << odometry->pose.pose.orientation.w << endl;
    cout << "odometry.pose.pose.orientation.x = " << odometry->pose.pose.orientation.x << endl;
    cout << "odometry.pose.pose.orientation.y = " << odometry->pose.pose.orientation.y << endl;
    cout << "odometry.pose.pose.orientation.z = " << odometry->pose.pose.orientation.z << endl;

    cout << "odometry.pose.pose.position.x = " << odometry->pose.pose.position.x << endl;
    cout << "odometry.pose.pose.position.y = " << odometry->pose.pose.position.y << endl;
    cout << "odometry.pose.pose.position.z = " << odometry->pose.pose.position.z << endl;
/*
    cout << "odometry.twist.covariance[0] = " << odometry->twist.covariance[0] << endl;
    cout << "odometry.twist.twist.angular.x = " << odometry->twist.twist.angular.x << endl;
    cout << "odometry.twist.twist.angular.y = " << odometry->twist.twist.angular.y << endl;
    cout << "odometry.twist.twist.angular.z = " << odometry->twist.twist.angular.z << endl;

    cout << "odometry.twist.twist.linear.x = " << odometry->twist.twist.linear.x << endl;
    cout << "odometry.twist.twist.linear.y = " << odometry->twist.twist.linear.y << endl;
    cout << "odometry.twist.twist.linear.z = " << odometry->twist.twist.linear.z << endl;
*/
    return bytecount;
}

int transmision::recvData(Odometry *odometry)
{

#ifdef DEBUG
    cout << "\n transmision::recvData(Odometry *odometry) \n" << endl;
#endif

    //char buffer[1024];
    int buffer_len = 1024;
    int bytecount;
//    memset(buffer, 0, buffer_len);
    //strcpy(buffer, "yujuuuu");
    try
    {
        if((bytecount = recv(sock, odometry, sizeof(Odometry), 0)) < 1)
        {
            fprintf(stderr, "Error sending data %d\n", errno);
//          goto FINISH;
            return -1;
        }

    }
    catch ( SocketException& )
    {
        cout << "un error aqui_1" << endl;
    }

    cout << "   --- Datos recibidos---"<< endl;
    //cout << "       bytecount = " << bytecount << endl;
//    cout << "odometry.child_frame_id = " << odometry->child_frame_id << endl;

    cout << "odometry.pose.covariance[0] = " << odometry->pose.covariance[0] << endl;
    cout << "odometry.pose.pose.orientation.w = " << odometry->pose.pose.orientation.w << endl;
    cout << "odometry.pose.pose.orientation.x = " << odometry->pose.pose.orientation.x << endl;
    cout << "odometry.pose.pose.orientation.y = " << odometry->pose.pose.orientation.y << endl;
    cout << "odometry.pose.pose.orientation.z = " << odometry->pose.pose.orientation.z << endl;

    cout << "odometry.pose.pose.position.x = " << odometry->pose.pose.position.x << endl;
    cout << "odometry.pose.pose.position.y = " << odometry->pose.pose.position.y << endl;
    cout << "odometry.pose.pose.position.z = " << odometry->pose.pose.position.z << endl;
/*
    cout << "odometry.twist.covariance[0] = " << odometry->twist.covariance[0] << endl;
    cout << "odometry.twist.twist.angular.x = " << odometry->twist.twist.angular.x << endl;
    cout << "odometry.twist.twist.angular.y = " << odometry->twist.twist.angular.y << endl;
    cout << "odometry.twist.twist.angular.z = " << odometry->twist.twist.angular.z << endl;

    cout << "odometry.twist.twist.linear.x = " << odometry->twist.twist.linear.x << endl;
    cout << "odometry.twist.twist.linear.y = " << odometry->twist.twist.linear.y << endl;
    cout << "odometry.twist.twist.linear.z = " << odometry->twist.twist.linear.z << endl;
*/
    return bytecount;
}


transmision::~transmision()
{

}
