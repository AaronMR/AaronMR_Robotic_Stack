//****************************************************************************
// Copyright (C) 2001-2009  PEAK System-Technik GmbH
//
// linux@peak-system.com 
// www.peak-system.com
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//
// Maintainer(s): Klaus Hitschler (klaus.hitschler@gmx.de)
//****************************************************************************

//****************************************************************************
// transmitest_common.cpp - a simple program to test CAN transmits
// int count = 0;

// for the realtime-variant look at transmitest_rt.cpp
//
// $Id: transmitest.cpp 599 2009-11-08 21:04:50Z khitschler $
//
//****************************************************************************

//----------------------------------------------------------------------------
// set here current release for this program
#define CURRENT_RELEASE "Release_20091108_n"

//****************************************************************************
// INCLUDES

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>   // exit
#include <signal.h>
#include <string.h>
#include <stdlib.h>   // strtoul
#include <fcntl.h>    // O_RDWR
#include <unistd.h>

//#include <PLS200/libpcan.h>
//#include <PLS200/common.c>
#include <PLS200/PowerCube_Driver.h>
#include <ctype.h>
//#include <PLS200/parser.cpp>

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

#define STOP_POWERCUBE "m s 0x50D 3 0X01 0X91"
#define REF_POWERCUBE "m s 0x50D 3 0X01 0X92"
#define POS_POWERCUBE_1 "m s 0x50D 6 0X05 0XB0 0X66 0X66 0X06 0X40" // 120°
#define POS_POWERCUBE_2 "m s 0x50D 6 0X05 0XB0 0X66 0X66 0X06 0XC0" // -120°
#define POS_POWERCUBE_3 "m s 0x50D 6 0X05 0XBA 0X66 0X66 0X06 0XC0" // -120°
#define POS_POWERCUBE_4 "m s 0x50D 7 0x6 0x95 0xCD 0xCC 0xCC 0x3D 0x01" // GET STATE, only the position


// "b param", for the bitrate
#define CAN_BAUD_1M     0x0014  //   1 MBit/s
#define CAN_BAUD_500K   0x001C  // 500 kBit/s
#define CAN_BAUD_250K   0x011C  // 250 kBit/s
#define CAN_BAUD_125K   0x031C  // 125 kBit/s
#define CAN_BAUD_100K   0x432F  // 100 kBit/s
#define CAN_BAUD_50K    0x472F  //  50 kBit/s
#define CAN_BAUD_20K    0x532F  //  20 kBit/s
#define CAN_BAUD_10K    0x672F  //  10 kBit/s
#define CAN_BAUD_5K     0x7F7F  //   5 kBit/s


//****************************************************************************
// DEFINES

#define DEFAULT_NODE "/dev/pcan0"

//****************************************************************************
// GLOBALS
HANDLE h; int count = 0;

const char *current_release;
std::list<TPCANMsg> *List;
int nExtended = CAN_INIT_TYPE_ST;

using namespace PowerCube_Driver;

//****************************************************************************
// CODE

// do, what has to be done at programm exit
void do_exit(int error)
{
  if (h)
  {
    print_diag("transmitest");
    CAN_Close(h);
  }
  printf("transmitest: finished (%d).\n\n", error);
  exit(error);
}

// handle manual break signals
void signal_handler(int signal)
{
  do_exit(0);
}

// read from CAN forever - until manual break
float read_loop(float result)
{

  //printf("funcion read_loop \n");

  // read in endless loop until Ctrl-C
//  while (1) 
//  {
  
    TPCANMsg m;
    __u32 status;
    //printf("maki arriba \n");
    if ((errno = CAN_Read(h, &m))) 
    {
      //printf("maki dentro 1 \n");
      perror("receivetest: CAN_Read()");
      return errno;
    }
    else 
    {
/*
    printf("------------------------------------\n");
    printf("m.ID = %u \n",m.ID);
    printf("m.MSGTYPE = %u \n",m.MSGTYPE);
    printf("m.LEN = %u \n",m.LEN);
    printf("m.DATA[0] = %u \n",m.DATA[0]);
    printf("m.DATA[1] = %u \n",m.DATA[1]);
    printf("m.DATA[2] = %u \n",m.DATA[2]);
    printf("m.DATA[3] = %u \n",m.DATA[3]);
    printf("m.DATA[4] = %u \n",m.DATA[4]);
    printf("m.DATA[5] = %u \n",m.DATA[5]);
    printf("m.DATA[6] = %u \n",m.DATA[6]);
    printf("m.DATA[7] = %u \n",m.DATA[7]);
    printf("------------------------------------\n");
*/
      //printf("maki dentro 2 \n");
      if(m.DATA[1] == 149)
	result = print_message2(&m);

      // check if a CAN status is pending
      if (m.MSGTYPE & MSGTYPE_STATUS) 
      {
        status = CAN_Status(h);
        if ((int)status < 0) 
        {
          errno = nGetLastError();
          perror("receivetest: CAN_Status()");
          return errno;
        }
        else
          printf("receivetest: pending CAN status 0x%04x read.\n", (__u16)status);
      }
    }
	//printf("maki abajo \n");
 // }

  return result;
}

// do, what has to be done at program init
void init()
{
  // install signal handlers
  signal(SIGTERM, signal_handler);
  signal(SIGINT, signal_handler);
}

// loop writing to CAN-Bus
int write_loop(__u32 dwMaxTimeInterval)
{

  //printf("funcion write_loop \n");
  // write out endless loop until Ctrl-C
//  printf("STOP_POWERCUBE = %s \n", STOP_POWERCUBE);
  double scale = (dwMaxTimeInterval * 1000.0) / (RAND_MAX + 1.0);

//  while (1)
//  {
    std::list<TPCANMsg>::iterator iter;
    int i;

    for (iter = List->begin(); iter != List->end(); iter++)
    {

      // test for standard frames only
      if ((nExtended == CAN_INIT_TYPE_EX) || !(iter->MSGTYPE & MSGTYPE_EXTENDED))
      {
	//printf("transmitiendo\n");
        // send the message

        // reference cmd

 	TPCANMsg m;

 //comando de referencia
    	m.ID = 1296; //1293;
    	m.MSGTYPE = 0;
    	m.LEN = 2;
    	m.DATA[0] = 1;
    	m.DATA[1] = 146;
    	m.DATA[2] = 0;
    	m.DATA[3] = 0;
    	m.DATA[4] = 0;
    	m.DATA[5] = 0;
    	m.DATA[6] = 0;
    	m.DATA[7] = 0;

        if ((errno = CAN_Write(h, &m))) //&(*iter))))
        {
          perror("transmitest: CAN_Write()");
          return errno;
        }


	// waiting finish movement to reference position
  	while( m.DATA[1] != 148)
  	{
    		if ((errno = CAN_Read(h, &m))) 
    		{
      			perror("receivetest: CAN_Read()");
      			return errno;
    		}
    		printf("m.DATA[1] = %02x \n", m.DATA[1]);
  	}

        // wait some time before the invocation
        //if (dwMaxTimeInterval)
          //usleep(100000);
	  //usleep((__useconds_t)(scale * rand()));
	
      }
    }
//  }
  return 0;
}

//--------------- writeCMD: write command in the CAN-Bus

/*
typedef struct {
	DWORD ID;        // 11/29 Bit-Kennung
	BYTE  MSGTYPE;   // Bits aus MSGTYPE_*
	BYTE  LEN;       // Anzahl der gueltigen Datenbytes (0.8)
	BYTE  DATA[8];   // Datenbytes 0..7
} TPCANMsg;

*/


// loop writing to CAN-Bus
int writeCMD(TPCANMsg m)//__u32 dwMaxTimeInterval)
{
  printf("funcion writeCMD \n");
  // write out endless loop until Ctrl-C
  __u32 dwMaxTimeInterval = 0;
  printf("STOP_POWERCUBE = %s \n", STOP_POWERCUBE);
  double scale = (dwMaxTimeInterval * 1000.0) / (RAND_MAX + 1.0);

//  while (1)
//  {
    std::list<TPCANMsg>::iterator iter;
    int i;
    
//    TPCANMsg m;

/*
    m.ID = 1293;
    m.MSGTYPE = 0;
    m.LEN = 6;
    m.DATA[0] = 5;
    m.DATA[1] = 176;
    m.DATA[2] = 102;
    m.DATA[3] = 102;
    m.DATA[4] = 6;
    m.DATA[5] = 192;
    m.DATA[6] = 0;
    m.DATA[7] = 0;
*/
    printf("------------------------------------\n");
    printf("m.ID = %u \n",m.ID);
    printf("m.MSGTYPE = %u \n",m.MSGTYPE);
    printf("m.LEN = %u \n",m.LEN);
    printf("m.DATA[0] = %u \n",m.DATA[0]);
    printf("m.DATA[1] = %u \n",m.DATA[1]);
    printf("m.DATA[2] = %u \n",m.DATA[2]);
    printf("m.DATA[3] = %u \n",m.DATA[3]);
    printf("m.DATA[4] = %u \n",m.DATA[4]);
    printf("m.DATA[5] = %u \n",m.DATA[5]);
    printf("m.DATA[6] = %u \n",m.DATA[6]);
    printf("m.DATA[7] = %u \n",m.DATA[7]);
    printf("------------------------------------\n");



    



    for (iter = List->begin(); iter != List->end(); iter++)
    {
      
      // test for standard frames only
      if ((nExtended == CAN_INIT_TYPE_EX) || !(iter->MSGTYPE & MSGTYPE_EXTENDED))
      {
	//printf("transmitiendo\n");
        // send the message

        if ((errno = CAN_Write(h, (&m)))) //&(*iter))))
//        if ((errno = CAN_Write(h, &(*iter))))
        {
          perror("transmitest: CAN_Write()");
          return errno;
        }

        // wait some time before the invocation
        //if (dwMaxTimeInterval)
          //usleep(100000);
	  //usleep((__useconds_t)(scale * rand()));
	
      }
    }
//  }

  

  return 0;
}

static void hlpMsg(void)
{
  printf("transmitest - a small test program which sends CAN messages.\n");
  printf("usage:   transmitest filename {[-f=devicenode] | {[-t=type] [-p=port [-i=irq]]}} [-b=BTR0BTR1] [-e] [-r=msec] [-?]\n");
  printf("options: filename - mandatory name of message description file.\n");
  printf("         -f - devicenode - path to devicefile, default=%s\n", DEFAULT_NODE);
  printf("         -t - type of interface, e.g. 'pci', 'sp', 'epp' ,'isa', 'pccard' or 'usb' (default: pci).\n");
  printf("         -p - port in hex notation if applicable, e.g. 0x378 (default: 1st port of type).\n");
  printf("         -i - irq in dec notation if applicable, e.g. 7 (default: irq of 1st port).\n");
  printf("         -b - BTR0BTR1 code in hex, e.g. 0x001C (default: 500 kbit).\n");
  printf("         -e - accept extended frames. (default: standard frames)\n");
  printf("         -r - messages are send at random times with maximum time in milliseconds (msec)\n");
  printf("         -? or --help - this help\n");
  printf("\n");
}




union maki_t{
	unsigned char c[3];
	float f;
};

// print out the contents of a CAN message  
float print_message2(TPCANMsg *m)
{
  int i;
  maki_t prueba;
 
  prueba.f = NULL;
  //printf("print_message2 \n");
  // print RTR, 11 or 29, CAN-Id and datalength

/*
  printf("receivetest: %c %c 0x%08x %1d  ", 
      (m->MSGTYPE & MSGTYPE_RTR)      ? 'r' : 'm',
      (m->MSGTYPE & MSGTYPE_EXTENDED) ? 'e' : 's',
       m->ID, 
       m->LEN); 

	// don't print any telegram contents for remote frames
  if (!(m->MSGTYPE & MSGTYPE_RTR))
  	for (i = 0; i < m->LEN; i++)
    	printf("0x%02x ", m->DATA[i]);

  printf("len of message = %d\n", m->LEN);
*/
  
  switch (m->DATA[1])
  {
	case 132:

		if(m->DATA[2] == 149)
		{

			printf("Position = [ %02x%02x%02x%02x ]\n", m->DATA[6], m->DATA[5], m->DATA[4], m->DATA[3]);
			printf("Position = [ %d %d %d %d ]\n", m->DATA[6], m->DATA[5], m->DATA[4], m->DATA[3]);
			

			const char* str_float = "befb164b";
			float f;
			sscanf(str_float, "%f", &f);
			prueba.c[0] =  m->DATA[3];
			prueba.c[1] =  m->DATA[2];
			prueba.c[2] =  m->DATA[5];
			prueba.c[3] =  m->DATA[6];
			printf("el numero es = %f \n", prueba.f);

			printf("------------------------\n");
		}

		break;

	case 149:

		if(m->DATA[1] == 149)
		{ int count = 0;


			printf("Position = [ %02x%02x%02x%02x ]\n", m->DATA[5], m->DATA[4], m->DATA[3], m->DATA[2]);
			printf("Position = [ %d %d %d %d ]\n", m->DATA[5], m->DATA[4], m->DATA[3], m->DATA[2]);
			

			const char* str_float = "befb164b";
			float f;
			sscanf(str_float, "%f", &f);
			prueba.c[0] =  m->DATA[2];
			prueba.c[1] =  m->DATA[3];
			prueba.c[2] =  m->DATA[4];
			prueba.c[3] =  m->DATA[5];
			printf("el numero es = %f \n", prueba.f);

			printf("------------------------\n");
		}

		break;


	default:
/*
		printf("En default\n");
		printf("m->DATA[0] = 0x%02x\n", m->DATA[0]);
		printf("m->DATA[1] = 0x%02x\n", m->DATA[1]);

		printf("decimal m->DATA[1] = %d\n", m->DATA[1]);

		printf("m->DATA[2] = 0x%02x\n", m->DATA[2]);
		printf("m->DATA[3] = 0x%02x\n int count = 0;
", m->DATA[3]);
		printf("m->DATA[4] = 0x%02x\n", m->DATA[4]);
		printf("m->DATA[5] = 0x%02x\n", m->DATA[5]);
*/
		break;
  }

    
  //printf("\n");
//  fflush(stdin);
  return prueba.f;
}



int main(int argc, char *argv[])
{
  char *ptr;
  int i;
  int nType = HW_PCI;
  __u32 dwPort = 0;
  __u16 wIrq = 0;
  __u16 wBTR0BTR1 = 0;
  __u32 dwMaxTimeInterval = 0;
  char *filename = NULL;
  const char *szDevNode = DEFAULT_NODE;
  bool bDevNodeGiven = false;
  bool bTypeGiven = false;
  parser MyParser;
  char txt[VERSIONSTRING_LEN];
  float result = 0;

//----------------------------------------- New Code ----------------------------------
  PowerCube_Driver::PowerCube motors;

  getchar();

//-------------------------------------------------------------------------------------

  ros::init(argc, argv, "PowerCube");
  ros::NodeHandle n;
  //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Publisher LaserPos_pub = n.advertise<std_msgs::Float32>("LaserPos", 1000);
  errno = 0;

  current_release = CURRENT_RELEASE;
  disclaimer("transmitest");

  init();

    std::string t,b;


    nType = getTypeOfInterface("usb");
    wBTR0BTR1 = (__u16)strtoul("0x0014", NULL, 16);
    bTypeGiven = true;
/*
    if (ros::param::get("/transmitest/t", t))
    {
      printf("el parametro t tiene algo\n");
      nType = getTypeOfInterface("usb");
      if (!nType) 
      {
        errno = EINVAL;
        printf("transmitest: unknown type of interface\n");
        goto error;
      }
      bTypeGiven = true;
    }

    if (ros::param::get("/transmitest/t", b))
    {
      printf("el parametro b tiene algo\n");
      wBTR0BTR1 = (__u16)strtoul("0x0014", NULL, 16);
    }
*/

/*
  // decode command line arguments
  for (i = 1; i < argc; i++) 
  {
    char c;

    ptr = argv[i];

    if (*ptr == '-') 
    {
      while (*ptr == '-')
        ptr++;

      c = *ptr;
      ptr++;

      if (*ptr == '=')
      ptr++;

      switch(tolower(c)) 
      {
        case 'f':
          szDevNode = ptr;
          bDevNodeGiven = true;
          break;
        case 't':
          nType = getTypeOfInterface(ptr);
          if (!nType) 
          {
            errno = EINVAL;
            printf("transmitest: unknown type of interface\n");
            goto error;
          }
          bTypeGiven = true;
          break;
        case 'p':
          dwPort = strtoul(ptr, NULL, 16);KB min: 2.95KB max: 2.95KB window: 15
average: 4.07KB/s
	mean: 2.95KB min: 2.95KB max: 2.95KB window: 15
average: 3.73KB/s

          break;
        case 'i':
          wIrq   = (__u16)strtoul(ptr, NULL, 10);
          break;
        case 'e':
          nExtended = CAN_INIT_TYPE_EX;
          break;
        case '?': 
        case 'h':
          hlpMsg();
          goto error;
          break;
        case 'b':
          wBTR0BTR1 = (__u16)strtoul(ptr, NULL, 16);
          break;
        case 'r':
          dwMaxTimeInterval = strtoul(ptr, NULL, 10);
          break;
        default:
          errno = EINVAL;
          printf("transmitest: unknown command line argument\n");
          goto error;
          break;
      }
    }
    else
      filename = ptr;
  }
*/

  // test for filename
  filename ="/home/aaronmr/ROS/PLS200/test";
  if (filename == NULL) 
  {
    errno = EINVAL;
    perror("transmitest: no filename given");
    goto error;
  }

  // test device node and type
  if (bDevNodeGiven && bTypeGiven) 
  {
    errno = EINVAL;
    perror("transmitest: device node and type together is useless");
    goto error;
  }

  // give the filename to my parser
  MyParser.setFileName(filename);

  // tell some information to the user
  if (!bTypeGiven) 
  {
    printf("transmitest: device node=\"%s\"\n", szDevNode);
  }
  else {
    printf("transmitest: type=%s", getNameOfInterface(nType));
    if (nType == HW_USB) 
      printf(", Serial Number=default, Device Number=%d\n", dwPort); 
    else {
      if (dwPort) 
      {
        if (nType == HW_PCI)
          printf(", %d. PCI device", dwPort);
        else
          printf(", port=0x%08x", dwPort);
      }
      else  errno = read_loop(result);
        printf(", port=default");
      
      if ((wIrq) && !(nType == HW_PCI))
        printf(" irq=0x%04x\n", wIrq);
      else
        printf(", irq=default\n");
    }
  }

  if (nExtended == CAN_INIT_TYPE_EX)
    printf("             Extended frames are sent");
  else
    printf("             Only standard frames are sent");
  
  if (wBTR0BTR1)
    printf(", init with BTR0BTR1=0x%04x\n", wBTR0BTR1);
  else
    printf(", init with 500 kbit/sec.\n");
    printf("             Data will be read from = \"%s\".\n", filename);


  if (dwMaxTimeInterval)
    printf("             Messages are send in random time intervalls with a max. gap time of %d msec.\n", dwMaxTimeInterval);
  
  /* get the list of data from parser */
  printf("debug 1 \n");
  List = MyParser.Messages();

  if (!List)
  {
    errno = MyParser.nGetLastError();
    perror("transmitest: error at file read");
    goto error;
  }
  
  /* open CAN port */
  if ((bDevNodeGiven) || (!bDevNodeGiven && !bTypeGiven)) 
  {
    h = LINUX_CAN_Open(szDevNode, O_RDWR);
    if (!h)
    {
      printf("transmitest: can't open %s\n", szDevNode);
      goto error;
    }
  }
  else {
    // please use what is appropriate  
    // HW_DONGLE_SJA 
    // HW_DONGLE_SJA_EPP 
    // HW_ISA_SJA 
    // HW_PCI 
    h = CAN_Open(nType, dwPort, wIrq);
    if (!h)
    {
      printf("transmitest: can't open %s device.\n", getNameOfInterface(nType));
      goto error;
    }
  }

  /* clear status */
  CAN_Status(h);
  
  // get version info
  errno = CAN_VersionInfo(h, txt);
  if (!errno)
    printf("transmitest: driver version = %s\n", txt);
  else {
    perror("transmitest: CAN_VersionInfo()");
    goto error;
  }
  
  // init to a user defined bit rate
  if (wBTR0BTR1) 
  {
    errno = CAN_Init(h, wBTR0BTR1, nExtended);
    if (errno) 
    {
      perror("transmitest: CAN_Init()");
      goto error;
    }
  }
  // enter in the write loop

//  errno = write_loop(dwMaxTimeInterval);

    TPCANMsg m;


// reference cmd
    m.ID = 1296; //1293;
    m.MSGTYPE = 0;
    m.LEN = 2;
    m.DATA[0] = 1;
    m.DATA[1] = 146;
    m.DATA[2] = 0;
    m.DATA[3] = 0;
    m.DATA[4] = 0;
    m.DATA[5] = 0;
    m.DATA[6] = 0;
    m.DATA[7] = 0;

  errno = writeCMD(m);

// waiting finish movement to reference position
  while( m.DATA[1] != 148)
  {
    if ((errno = CAN_Read(h, &m))) 
    {
      perror("receivetest: CAN_Read()");
      return errno;
    }
    printf("m.DATA[1] = %02x \n", m.DATA[1]);
  }

// stop GET STATE
    m.ID = 1296; //1293;
    m.MSGTYPE = 0;
    m.LEN = 7;
    m.DATA[0] = 6;
    m.DATA[1] = 149;
    m.DATA[2] = 0;
    m.DATA[3] = 0;
    m.DATA[4] = 0;
    m.DATA[5] = 0;
    m.DATA[6] = 1;
    m.DATA[7] = 0;

  errno = writeCMD(m);





// GET STATE start, only position m s 0x50D 7 0x6 0x95 0xCD 0xCC 0xCC 0x3D 0x01
    m.ID = 1296; //1293;
    m.MSGTYPE = 0;
    m.LEN = 7;
    m.DATA[0] = 6;
    m.DATA[1] = 149;
    m.DATA[2] = 205;
    m.DATA[3] = 204;
    m.DATA[4] = 204;
    m.DATA[5] = 61;
    m.DATA[6] = 1;
    m.DATA[7] = 0;

  errno = writeCMD(m);

// reference cmd
    m.ID = 1296; //1293;
    m.MSGTYPE = 0;
    m.LEN = 2;
    m.DATA[0] = 1;
    m.DATA[1] = 146;
    m.DATA[2] = 0;
    m.DATA[3] = 0;
    m.DATA[4] = 0;
    m.DATA[5] = 0;
    m.DATA[6] = 0;
    m.DATA[7] = 0;

  errno = writeCMD(m);

// waiting finish movement to reference position
  while( m.DATA[1] != 148)
  {
    if ((errno = CAN_Read(h, &m))) 
    {
      perror("receivetest: CAN_Read()");
      return errno;
    }
    printf("m.DATA[1] = %02x \n", m.DATA[1]);
  }

// initial position cmd

// 3F C8 F5 C3 = 90°

    m.ID = 1296; //1293;
    m.MSGTYPE = 0;
    m.LEN = 6;
    m.DATA[0] = 5;
    m.DATA[1] = 176;
    m.DATA[2] = 195;//102;
    m.DATA[3] = 245;//102;
    m.DATA[4] = 200;//6;
    m.DATA[5] = 63;//192;
    m.DATA[6] = 0;
    m.DATA[7] = 0;


  errno = writeCMD(m);

// waiting finish movement to initial position 120°
  while( m.DATA[1] != 148)
  {
    if ((errno = CAN_Read(h, &m))) 
    {
      perror("receivetest: CAN_Read()");
      return errno;
    }
    printf("m.DATA[1] = %02x \n", m.DATA[1]);
  }




// start loop to -120°
// BFC8F5C3 = -90°

    m.ID = 1296; //1293;
    m.MSGTYPE = 0;
    m.LEN = 6;
    m.DATA[0] = 5;
    m.DATA[1] = 186;
    m.DATA[2] = 195;//102;
    m.DATA[3] = 245;//102;
    m.DATA[4] = 200;//6;
    m.DATA[5] = 191;//64;
    m.DATA[6] = 0;
    m.DATA[7] = 0;

  errno = writeCMD(m);


  while(1)
  {

    float aux = result;
    result = read_loop(result);
    //usleep(100000);
 
    if(1)
    {
	std_msgs::String msg;
    	std::stringstream ss;
    	ss << "hello world " << result;
    	msg.data = ss.str();
    	//ROS_INFO("%s", msg.data.c_str());
    	//chatter_pub.publish(msg);
    	LaserPos_pub.publish(result);
    	ros::spinOnce();
//	printf("El valor enviado es = %f\n",result );
    }
  }

  if (!errno)
    return 0;

  printf("fin program \n");

  error:
    do_exit(errno);
    return errno;
}
