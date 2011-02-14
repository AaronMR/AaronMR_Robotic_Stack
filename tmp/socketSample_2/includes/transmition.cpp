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
    catch ( SocketException& ) {
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
    catch ( SocketException& ) {
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

        if((bytecount = send(sock, b, sizeof(Struct_2), 0)) < 1){
            fprintf(stderr, "Error sending data %d\n", errno);
//          goto FINISH;
            return -1;
        }
    }
    catch ( SocketException& ) {
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
        if((bytecount = recv(sock, b, sizeof(Struct_2), 0)) < 1){
            fprintf(stderr, "Error sending data %d\n", errno);
//          goto FINISH;
            return -1;
        }

    }
    catch ( SocketException& ) {
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



transmision::~transmision()
{

}
