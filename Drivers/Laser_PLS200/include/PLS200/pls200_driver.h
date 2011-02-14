#include <ros/ros.h>
#include <stdlib.h>
#include <netdb.h>
#include <sensor_msgs/LaserScan.h>

#define BUF_SIZE 1024


int connectLaser();
namespace sick_pls200
{
  ////////////////////////////////////////////////////////////////////////////////
  typedef struct
  {
    unsigned char* string;
    int length;
  } MeasurementQueueElement_t;

  ////////////////////////////////////////////////////////////////////////////////
  typedef struct
  {
    uint16_t Format;
    uint16_t DistanceScaling;
    int32_t  StartingAngle;
    uint16_t AngularStepWidth;
    uint16_t NumberMeasuredValues;
    uint16_t ScanningFrequency;
    uint16_t RemissionScaling;
    uint16_t RemissionStartValue;
    uint16_t RemissionEndValue;
  } MeasurementHeader_t;


  ////////////////////////////////////////////////////////////////////////////////
  class SickPLS200
  {
    public:
      char* texto;
      int Aux;
      //SickPLS200 () { }
      SickPLS200 ();
     
      // connection and disconnection with the laser, serial port
      int Connect ();
      int Disconnect ();

      // laser file descriptor     
      int laser_fd;

      // device name 
      const char *device_name;

      int OpenTerm();

      int CloseTerm();

      int StartLaser();

      int writeport(int fd, char *chars, int len);

      int ReadLaserData(uint16_t *data, size_t datalen);

      ssize_t ReadFromLaser(uint8_t *data, ssize_t maxlen, bool ack = false, int timeout = -1, int timeout_header = -1);

      int RequestLaserStopStream();

      ssize_t WriteToLaser(uint8_t *data, ssize_t len);

      // Start and end scan segments (for restricted scan).  These are
      // the values used by the laser.
      int scan_min_segment, scan_max_segment;

      // Request data from the laser
      // Returns 0 on success
      int RequestLaserData(int min_segment, int max_segment);

      // Calculates CRC for a telegram
      unsigned short CreateCRC(uint8_t *data, ssize_t len);

      // Internal Parameters:
      int verbose_;

      // for sending:
      unsigned char command_[BUF_SIZE];
      int commandlength_;
      std::vector<MeasurementQueueElement_t>* MeasurementQueue_;
/*
      // Creates socket, connects
      int Connect ();
      int Disconnect ();

      // Configuration parameters
      int SetAngularResolution (const char* password, float ang_res, float angle_start, float angle_range);
      int SetScanningFrequency (const char* password, float freq, float angle_start, float angle_range);
      int SetResolutionAndFrequency (float freq, float ang_res, float angle_start, float angle_range);

      int StartMeasurement (bool intensity = true);
      sensor_msgs::LaserScan ReadMeasurement  ();
      int StopMeasurement  ();

      int SetUserLevel  (int8_t userlevel, const char* password);
      int GetMACAddress (char** macadress);

      int SetIP         (char* ip);
      int SetGateway    (char* gw);
      int SetNetmask    (char* mask);
      int SetPort       (uint16_t port);

      int ResetDevice            ();
      int TerminateConfiguration ();

      int SendCommand   (const char* cmd);
      int ReadResult    ();
      // for "Variables", Commands that only reply with one Answer message
      int ReadAnswer    ();
      // for "Procedures", Commands that reply with a Confirmation message and an Answer message
      int ReadConfirmationAndAnswer ();

      int EnableRIS (int onoff);
      int SetMeanFilterParameters (int num_scans);
      int SetRangeFilterParameters (float range_min, float range_max);
      int EnableFilters (int filter_mask);

      // turns a string holding an ip address into long
      unsigned char* ParseIP (char* ip);

    private:
      // assembles STX's, length field, message, checksum ready to be sent. Cool.
      int AssembleCommand (unsigned char* command, int len);

      const char* hostname_;
      int sockfd_, portno_, n_;
      struct sockaddr_in serv_addr_;
  #if HAVE_GETADDRINFO
      struct addrinfo *addr_ptr_;
  #else
      struct hostent *server_;
  #endif

      // Internal Parameters:
      int verbose_;
      int ExtendedRIS_;
      int MeanFilterNumScans_;
      float RangeFilterTopLimit_;
      float RangeFilterBottomLimit_;
      int FilterMask_;

      long int scanning_frequency_, resolution_;

      // for reading:
      unsigned char buffer_[4096];
      unsigned int bufferlength_;

      // for sending:
      unsigned char command_[BUF_SIZE];
      int commandlength_;
      std::vector<MeasurementQueueElement_t>* MeasurementQueue_;

*/
   };

}

 
