
#include <PLS200/pls200_driver.h>
#include <angles/angles.h>

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <sys/time.h>

#define LOBYTE(w) ((uint8_t) (w & 0xFF))
#define HIBYTE(w) ((uint8_t) ((w >> 8) & 0xFF))
#define MAKEUINT16(lo, hi) ((((uint16_t) (hi)) << 8) | ((uint16_t) (lo)))


#define MAXNDATA 802
//#define DEBUG

////////////////////////////////////////////////////////////////////////////////
// Device codes

#define STX     0x02
#define ACK     0xA0
#define NACK    0x15
#define CRC16_GEN_POL 0x8005

const int CMD_BUFFER_SIZE = 255;



////////////////////////////////////////////////////////////////////////////////
// Constructor.
sick_pls200::SickPLS200::SickPLS200 ()
{
  
  //verbose_  = debug_mode;
  memset (command_, 0, BUF_SIZE);
  MeasurementQueue_ = new std::vector<MeasurementQueueElement_t >;
  
  scan_min_segment = 0;
  scan_max_segment = 360;
  
  texto = "Esto parece que funciona";
  Aux = 1;
}



int initport(int fd) {
	struct termios options;
	// Get the current options for the port...
	tcgetattr(fd, &options);
	// Set the baud rates to 19200...
	cfsetispeed(&options, B38400);
	cfsetospeed(&options, B38400);
	// Enable the receiver and set local mode...
	options.c_cflag |= (CLOCAL | CREAD);

	options.c_cflag |= PARENB;
	options.c_cflag &= ~PARODD;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;

	//printf("%d",options.c_cflag);
	// Set the new options for the port...
	tcsetattr(fd, TCSANOW, &options);
	return 1;
}

////////////////////////////////////////////////////////////////////////////////
// Open the terminal
// Returns 0 on success
int sick_pls200::SickPLS200::writeport(int fd, char *chars, int len)
{

	chars[len] = 0x0d; // stick a <CR> after the command
	chars[len+1] = 0x00; // terminate the string properly
	int n = write(this->laser_fd, chars, len);
	if (n < 0) {
		fputs("write failed!\n", stderr);
		return 0;
	}
	return 1;
}


////////////////////////////////////////////////////////////////////////////////
// Open the terminal
// Returns 0 on success
int sick_pls200::SickPLS200::OpenTerm()
{
  
#ifdef DEBUG
	printf("Funcion OpenTerm \n");
#endif
  this->laser_fd = ::open(this->device_name, O_RDWR | O_SYNC , S_IRUSR | S_IWUSR );

  printf("laser_fd = %d \n", this->laser_fd);
//  this->laser_fd = ::open(this->device_name, O_RDWR | O_NOCTTY | O_NDELAY);
  //this->laser_fd = ::open(this->device_name, O_RDWR | O_SYNC , S_IRUSR | S_IWUSR );
  
  if (this->laser_fd < 0)
  {
    printf("unable to open serial port [%s]; [%s]",
                  (char*) this->device_name, strerror(errno));
    return 1;
  }
  
  
  // set the serial port speed to 9600 to match the laser
  // later we can ramp the speed up to the SICK's 38K
  //
  struct termios term;
  if( tcgetattr( this->laser_fd, &term ) < 0 )
    printf("Unable to get serial port attributes");

  cfmakeraw( &term );
  cfsetispeed( &term, B38400 );
  cfsetospeed( &term, B38400 );
  
  term.c_cflag |= (CLOCAL | CREAD);
  term.c_cflag |= PARENB;
  term.c_cflag &= ~PARODD;
  term.c_cflag &= ~CSTOPB;
  term.c_cflag &= ~CSIZE;
  term.c_cflag |= CS8;
  
  if( tcsetattr( this->laser_fd, TCSAFLUSH, &term ) < 0 )
    printf("Unable to set serial port attributes");

  // Make sure queue is empty
  //
  tcflush(this->laser_fd, TCIOFLUSH);
  
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Close the terminal
// Returns 0 on success
//
int sick_pls200::SickPLS200::CloseTerm()
{
  
#ifdef DEBUG
	printf("Funcion CloseTerm \n");
#endif

  RequestLaserStopStream();

  /* REMOVE
#ifdef HAVE_HI_SPEED_SERIAL
  if (ioctl(this->laser_fd, TIOCSSERIAL, &this->old_serial) < 0) {
    //RETURN_ERROR(1, "error trying to reset serial to old state");
    PLAYER_WARN("ioctl() failed while trying to reset serial to old state");
  }
#endif
  */
  printf("laser_fd = %d \n", this->laser_fd);
  close(this->laser_fd);
  printf("laser_fd = %d \n", this->laser_fd);
  return 0;
}


////////////////////////////////////////////////////////////////////////////////
// Read range data from laser
//
int sick_pls200::SickPLS200::ReadLaserData(uint16_t *data, size_t datalen)
{
 
#ifdef DEBUG
	printf("Funcion: ReadLaserData \n");
#endif

  uint8_t raw_data[1024];

  // Read a packet from the laser
  //
  
  int len = ReadFromLaser(raw_data, sizeof(raw_data));
  
  if (len == 0)
  {
    printf("empty packet");
    return 1;
  }

  // Process raw packets
  //
  if (raw_data[0] == 0xB0)
  {
    // Determine the number of values returned
    //
    //int units = raw_data[2] >> 6;
    int count = (int) raw_data[1] | ((int) (raw_data[2] & 0x3F) << 8);
    assert((size_t) count <= datalen);

    // Strip the status info and shift everything down a few bytes
    // to remove packet header.
    //

    for (int i = 0; i < count; i++)
    {
      int src = 2 * i + 3;
      data[i] = raw_data[src + 0] | (raw_data[src + 1] << 8);
    }

  }
  else if (raw_data[0] == 0xB7)
  {
    // Determine which values were returned
    //
    //int first = ((int) raw_data[1] | ((int) raw_data[2] << 8)) - 1;
    //int last =  ((int) raw_data[3] | ((int) raw_data[4] << 8)) - 1;

    // Determine the number of values returned
    //
    //int units = raw_data[6] >> 6;
    int count = (int) raw_data[5] | ((int) (raw_data[6] & 0x3F) << 8);
    assert((size_t) count <= datalen);

    // Strip the status info and shift everything down a few bytes
    // to remove packet header.
    //
    for (int i = 0; i < count; i++)
    {
      int src = 2 * i + 7;
      data[i] = raw_data[src + 0] | (raw_data[src + 1] << 8);
    }
  }
  else
  {
    printf("unexpected packet type");
    return 0;
  }

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Read a packet from the laser
// Set ack to true to ignore all packets except ack and nack
// Set timeout to -1 to make this blocking, otherwise it will return in timeout ms.ReadFromLaser
// Returns the packet length (0 if timeout occurs)
//
ssize_t sick_pls200::SickPLS200::ReadFromLaser(uint8_t *data, ssize_t maxlen, bool ack, int timeout, int timeout_header)
{

#ifdef DEBUG
	printf("Funcion ReadFromLaser \n");
#endif



  if(timeout_header == -1)
    timeout_header = timeout;


  int64_t start_time = ros::Time::now().toNSec();//GetTime();
  int64_t stop_time = start_time + timeout;
  int64_t stop_time_header = start_time + timeout_header;
  //PLAYER_MSG2(2, "%Ld %Ld", start_time, stop_time);




  int bytes = 0;
  uint8_t header[5] = {0};
  uint8_t footer[3];


  // Read until we get a valid header
  // or we timeout
  //
  while (true)
  {

/*
    if (timeout >= 0)
    {
      fd_set rfds, efds;
      FD_ZERO(&rfds);
      FD_SET(this->laser_fd, &rfds);
      FD_ZERO(&efds);
      FD_SET(this->laser_fd, &efds);
      int64_t delay = stop_time_header - GetTime();
      if (delay < 0) delay = 0;
      struct timeval tv;
      tv.tv_usec = (delay % 1000) * 1000;
      tv.tv_sec = delay / 1000;
      int retval = ::select(this->laser_fd + 1, &rfds, 0, &efds, &tv);
      if (retval < 0)
      {
	if (errno == EINTR)
	  continue;
        PLAYER_ERROR("error on select (1)");
        return 0;
      }
      if (!retval)
      {
        PLAYER_MSG0(2, "timeout on select (1)");
        return 0;
      }

    }
*/

    //printf("reading %d\n", timeout); fflush(stdout);
    //sleep(1);
    bytes = ::read(this->laser_fd, header + sizeof(header) - 1, 1);
    //printf("aki \n");
    if (bytes < 0)
    {
      if (errno == EINTR)
	continue;
      printf("error on read (1)");
      return 0;
    }
    if (!bytes)
    {
      printf("eof on read (1)");
      return 0;
    }
    

#ifdef DEBUG
    printf("read = [ %02x %02x %02x %02x %02x ]\n", header[0],
						header[1],
						header[2],
						header[3],
						header[4],
						header[5]); fflush(stdout);
#endif


    if (header[0] == STX && header[1] == 0x80)
    {

      if (!ack)
      {
	//printf("1 ack = %d \n", ack);
	break;
      }

      if (header[4] == ACK || header[4] == NACK)
      {
	//printf("2 ack = %d \n", ack);
	break;
      }

    }

    memmove(header, header + 1, sizeof(header) - 1);

  }


  // Determine data length
  // Includes status, but not CRC, so subtract status to get data packet length.
  //
  ssize_t len = ((int) header[2] | ((int) header[3] << 8)) - 1;

#ifdef DEBUG
	printf("Longitud del mensaje + 1= %d\n", len+1);
#endif
  
  // Check for buffer overflows
  //
  if (len > maxlen)
  {
     //RETURN_ERROR(0, "buffer overflow (len > maxlen)");
     return 0;
  } 

  // Read in the data
  // Note that we smooge the packet type from the header
  // onto the front of the data buffer.
  //
  bytes = 0;
  data[bytes++] = header[4];
 
  while (bytes < len)
  {
/*
    if (timeout >= 0)
    {
      fd_set rfds, efds;
      FD_ZERO(&rfds);
      FD_SET(this->laser_fd, &rfds);
      FD_ZERO(&efds);
      FD_SET(this->laser_fd, &efds);
      int64_t delay = stop_time - GetTime();
      if (delay < 0) delay = 0;
      struct timeval tv;
      tv.tv_usec = (delay % 1000) * 1000;
      tv.tv_sec = delay / 1000;
      int retval = ::select(this->laser_fd + 1, &rfds, 0, &efds, &tv);
      if (retval < 0)
      {
	if (errno == EINTR)
	  continue;
        //PLAYER_ERROR("error on select (2)");
        return 0;
      }
      if (!retval)
      {
        //PLAYER_MSG0(2, "timeout on select (2)");
        return 0;
      }
    }
*/
    int retval = ::read(this->laser_fd, data + bytes, len - bytes);
    if (retval < 0)
    {
      if (errno == EINTR)
	continue;
      //PLAYER_ERROR("error on read (2)");
      return 0;
    }
    if (!retval)
    {
      //PLAYER_MSG0(2, "eof on read (2)");
      return 0;
    }
    bytes += retval;
  }

  // Read in footer
  //
  bytes = 0;
  while (bytes < 3)
  {
    if (timeout >= 0)
    {
      fd_set rfds, efds;
      FD_ZERO(&rfds);
      FD_SET(this->laser_fd, &rfds);
      FD_ZERO(&efds);
      FD_SET(this->laser_fd, &efds);
      int64_t delay = 0;// stop_time - GetTime();
      if (delay < 0) delay = 0;
      struct timeval tv;
      tv.tv_usec = (delay % 1000) * 1000;
      tv.tv_sec = delay / 1000;
      int retval = ::select(this->laser_fd + 1, &rfds, 0, &efds, &tv);
      if (retval < 0)
      {
	if (errno == EINTR)
	  continue;
        //PLAYER_ERROR("error on select (3)");
        return 0;
      }
      if (!retval)
      {
        //PLAYER_MSG0(2, "timeout on select (3)");
        return 0;
      }
    }
    int retval = ::read(this->laser_fd, footer + bytes, 3 - bytes);
    if (retval < 0)
    {
      if (errno == EINTR)
	continue;
      //PLAYER_ERROR("error on read (3)");
      return 0;
    }
    if (!retval)
    {
      //PLAYER_MSG0(2, "eof on read (3)");
      return 0;
    }
    bytes += retval;
  }

  // Construct entire packet
  // And check the CRC
  //
  uint8_t buffer[4 + 1024 + 1];
  assert(4 + len + 1 < (ssize_t) sizeof(buffer));
  memcpy(buffer, header, 4);
  memcpy(buffer + 4, data, len);
  memcpy(buffer + 4 + len, footer, 1);
  uint16_t crc = CreateCRC(buffer, 4 + len + 1);
  if (crc != MAKEUINT16(footer[1], footer[2]))
  { 

#ifdef DEBUG
	printf("CRC = %02x\n footer[1] y footer[2]= %02x, %02x \n", crc, footer[1], footer[2]);
#endif


    //getchar();
//    RETURN_ERROR(0, "CRC error, ignoring packet");
    return 0;
  }
  
#ifdef DEBUG
	printf("CRC = %02x\n footer[1] y footer[2]= %02x, %02x \n", crc, footer[1], footer[2]);
#endif

  //getchar();
  return len;


}

////////////////////////////////////////////////////////////////////////////////
// Create a CRC for the given packet
//
unsigned short sick_pls200::SickPLS200::CreateCRC(uint8_t* data, ssize_t len)
{

#ifdef DEBUG
	printf("Funcion CreateCRC \n");
#endif

#ifdef DEBUG
  printf("len = %d, data = [ ", len);
  int i=0;
  for(i=0;i<len+1;i++)
  {
	printf(" %02x ", data[i]);
  }
  printf(" ] \n");
#endif


  uint16_t uCrc16;
  uint8_t abData[2];

  uCrc16 = 0;
  abData[0] = 0;

  while(len-- )
  {
    abData[1] = abData[0];
    abData[0] = *data++;

    if( uCrc16 & 0x8000 )
    {
      uCrc16 = (uCrc16 & 0x7fff) << 1;
      uCrc16 ^= CRC16_GEN_POL;
    }
    else
    {
      uCrc16 <<= 1;
    }
    uCrc16 ^= MAKEUINT16(abData[0],abData[1]);
  }

  return (uCrc16);
}

////////////////////////////////////////////////////////////////////////////////
// Write a packet to the laser
//
ssize_t sick_pls200::SickPLS200::WriteToLaser(uint8_t *data, ssize_t len)
{

#ifdef DEBUG
	printf("Funcion WriteToLaser \n");
#endif

  uint8_t buffer[4 + 1024 + 2];
  assert(4 + len + 2 < (ssize_t) sizeof(buffer));

  // Create header
  //
  buffer[0] = STX;
  buffer[1] = 0;
  buffer[2] = LOBYTE(len);
  buffer[3] = HIBYTE(len);

  // Copy body
  //
  memcpy(buffer + 4, data, len);

  // Create footer (CRC)
  //
  uint16_t crc = CreateCRC(buffer, 4 + len);
  buffer[4 + len + 0] = LOBYTE(crc);
  buffer[4 + len + 1] = HIBYTE(crc);

  // Make sure both input and output queues are empty
  //
  tcflush(this->laser_fd, TCIOFLUSH);

  ssize_t bytes = 0;

#ifdef DEBUG
  printf("WriteToLaser: [ ");
  for(int i = 0; i < len + 6; i++)
    printf("%02x ", buffer[i]);
  printf("]\n");
#endif



#ifdef HAVE_HI_SPEED_SERIAL

  if(this->serial_high_speed_mode == 0 && current_rate > 38400)
  {
 	  printf("estoy en Hi Speed \n");
	  struct timeval start, end;
	  // have to write one char at a time, because if we're
	  // high speed, then must take no longer than 55 us between
	  // chars

	  int ret;
	  printf("LASER: writing %d bytes\n", 6+len);
	  for (int i =0; i < 6 + len; i++) {
	    do {
	      gettimeofday(&start, NULL);
	      ret = ::write(this->laser_fd, buffer + i, 1);
	    } while (!ret);
	    if (ret > 0) {
	      bytes += ret;
	    }

     // need to do this sort of busy wait to ensure the right timing
     // although I've noticed you will get some anamolies that are
     // in the ms range; this could be a problem...
     int usecs;
     do {
       gettimeofday(&end, NULL);
       usecs= (end.tv_sec - start.tv_sec)*1000000 +
         (end.tv_usec - start.tv_usec);
     } while (usecs < 60);

     printf("usecs: %d bytes=%02X\n", (end.tv_sec - start.tv_sec)*1000000 +
          (end.tv_usec - start.tv_usec), *(buffer + i));
    }
  } else {
  /* Notice for high_speed_mode != 0 but 500k we still use this method */
 	 bytes = ::write( this->laser_fd, buffer, 4 + len + 2);
  }
#else
  bytes = ::write( this->laser_fd, buffer, 4 + len + 2);
#endif

  // Make sure the queue is drained
  // Synchronous IO doesnt always work
  //
  ::tcdrain(this->laser_fd);

  // Return the actual number of bytes sent, including header and footer
  //
  return bytes;
}


////////////////////////////////////////////////////////////////////////////////
// Request status from the laser
// Returns 0 on success
//
int sick_pls200::SickPLS200::RequestLaserStopStream()
{
 
#ifdef DEBUG
	printf("Funcion RequestLaserStopStream \n");
#endif

  int tries;
  ssize_t len;
  uint8_t packet[20];
  int DEFAULT_LASER_RETRIES = 1;
  for (tries = 0; tries < DEFAULT_LASER_RETRIES; tries++)
  {

    packet[0] = 0x20;
    packet[1] = 0x25;
    len = 2;

// test remove this  !!!!!!
/*
    packet[0] = 0x20;
    packet[1] = 0x00;
    packet[2] = 'S';
    packet[3] = 'I';
    packet[4] = 'C';
    packet[5] = 'K';
    packet[6] = '_';
    packet[7] = 'P';
    packet[8] = 'L';
    packet[9] = 'S';
    len = 10;
*/
//-------------------------

// test remove this  !!!!!!
/*
    packet[0] = 0x20;
    packet[1] = 0x40;
    len = 2;
*/
//-------------------------

// test remove this  !!!!!!
/*
    packet[0] = 0x66;
    packet[1] = 0x01;
    len = 2;
*/
//-------------------------

    printf("sending LMS stop continuous mode [RequestLaserStopStream()]\n");
    if (WriteToLaser(packet, len) < 0)
    {
      
      return 1;
    }

    // Wait for laser to return ack
    // This could take a while...
    //
    printf("waiting for acknowledge, before ReadFromLaser\n");
    len = ReadFromLaser(packet, sizeof(packet), true, 3000, 1000);
    printf("len = %d \n", len);
    if (len < 0)
    {
      printf("len < 0 \n");
      return 1;
    }
    else if (len < 1)
    {
      printf("RequestLaserStopStream(): timeout in ReadFromLaser -- may be using wrong baud rate.");
      continue;
    }
    else if (packet[0] == NACK)
    {
      printf("RequestLaserStopStream(): request denied by laser");
      return 1;
    }
    else if (packet[0] != ACK)
    {
      printf("RequestLaserStopStream(): unexpected packet type");
      return 1;
    }
    break;
  }
  return (tries >= DEFAULT_LASER_RETRIES);
}

////////////////////////////////////////////////////////////////////////////////
// Request data from the laser
// Returns 0 on success
//
int sick_pls200::SickPLS200::RequestLaserData(int min_segment, int max_segment)
{
 
#ifdef DEBUG
	printf("Funcion RequestLaserData \n");
#endif

  int tries;
  ssize_t len;
  uint8_t packet[20];
  int DEFAULT_LASER_RETRIES = 1;

  for (tries = 0; tries < DEFAULT_LASER_RETRIES; tries++)
  {
    len = 0;
    packet[len++] = 0x20; /* mode change command */

    if (min_segment == 0 && max_segment == 360)
    {
      // Use this for raw scan data...
      //
      packet[len++] = 0x24;
    }
    else
    {
      // Or use this for selected scan data...
      //
      int first = min_segment + 1;
      int last = max_segment + 1;
      packet[len++] = 0x27;
      packet[len++] = (first & 0xFF);
      packet[len++] = (first >> 8);
      packet[len++] = (last & 0xFF);
      packet[len++] = (last >> 8);
    }

    //PLAYER_MSG0(2, "sending scan data request to laser");
    //printf("LASER: RLD: writing scan data\n");
    if (WriteToLaser(packet, len) < 0)
      return 1;

    // Wait for laser to return ack
    // This should be fairly prompt
    len = ReadFromLaser(packet, sizeof(packet), true, 2000);
    if (len < 0)
      return 1;
    else if (len < 1)
    {
      //PLAYER_WARN("timeout");
      continue;
    }
    else if (packet[0] == NACK)
    {
      //PLAYER_ERROR("request denied by laser");
      return 1;
    }
    else if (packet[0] != ACK)
    {
      //PLAYER_ERROR("unexpected packet type");;
      return 1;
    }

    break;
  }

  return (tries >= DEFAULT_LASER_RETRIES);
}

////////////////////////////////////////////////////////////////////////////////
// Close the terminal
// Returns 0 on success
//
int sick_pls200::SickPLS200::StartLaser()
{
	int fd;
	char sCmd[254];
	uint8_t sResult[1024];
	uint16_t mm_ranges[1024];

	//fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
	//fd = open("/dev/ttyUSB0", O_RDWR | O_SYNC , S_IRUSR | S_IWUSR );
			          

/*
 	//initport(fd);	
	// ponerlo en espera
	sCmd[0] = 0x02;
	sCmd[1] = 0x00;
	sCmd[2] = 0x02;
	sCmd[3] = 0x00;
	sCmd[4] = 0x20;
	sCmd[5] = 0x25;
	sCmd[6] = 0x35;
	sCmd[7] = 0x08;

	
        if (!writeport(fd, sCmd, 8)) {
		printf("write failed\n");
		close(this->laser_fd);
		return 1;
	}
*/
	printf("Apagando el laser \n");
	RequestLaserStopStream();

	printf("Encendiendo el laser \n");
	RequestLaserData(this->scan_min_segment, this->scan_max_segment);


/*
	// ponerlo a enviar datos
	sCmd[0] = 0x02;
	sCmd[1] = 0x00;
	sCmd[2] = 0x02;
	sCmd[3] = 0x00;
	sCmd[4] = 0x20;
	sCmd[5] = 0x24;
	sCmd[6] = 0x34;
	sCmd[7] = 0x08;

	printf("Encendiendo el laser \n");
        if (!writeport(fd, sCmd, 8)) {
		printf("write failed\n");
		close(this->laser_fd);
		return 1;
	}
*/

	fcntl(fd, F_SETFL, FNDELAY); // don't block serial read
	


	
	close(fd);
	return 0;
}

