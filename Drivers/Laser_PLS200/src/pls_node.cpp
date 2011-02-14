#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sensor_msgs/LaserScan.h>
#include <PLS200/pls200_driver.h>

using namespace sick_pls200;

int main(int argc, char** argv){
  ros::init(argc, argv, "laser_scan_publisher");


  // creacion del objeto laser.....
  SickPLS200 laser;
  laser.device_name = "/dev/ttyUSB0";

  ros::NodeHandle n;
  ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("tilt_scan1", 50);

  unsigned int num_readings = 362;
  double laser_frequency = 40;
  double ranges[num_readings];
  double intensities[num_readings];
  uint16_t mm_ranges[1024];

  int count = 0;
  ros::Rate r(50);

  
  // connection of Laser
  if (laser.OpenTerm() == 1)
  {
	std::cout << "yeah\n";
  }
  else
  {
	std::cout << "NOOOO\n";
  }

  laser.StartLaser();




// para ver cuanto tarda en recibir un dato del laser
/*
  struct tm *current;
  time_t now;
*/
  float escala = -1.30;	
  escala = 0.0;
  while(n.ok()){
// actualmente hace de 4 a 5 escaneos por segundo, esto es porque esta a una velocidad de 38400 bps, con lo que solo podemos recibir unos 4.6875 KBps, si tenemos en cuenta que cada vez que lee, se leen unos 1000 Bytes (casi un KiloByte), solo podemos hacer ese numeros de escaneos.
// Solucion: aumentar la velocidad del puerto.

/*
    time(&now);
    current = localtime(&now);
    printf("the time is %i:%i:%i\n", current->tm_hour, current->tm_min, current->tm_sec);
*/ 
    laser.ReadLaserData(mm_ranges, 1024);
    //generate some fake data for our laser scan
    for(unsigned int i = 0; i < num_readings; ++i){
      ranges[i] = (mm_ranges[i]) * 10 / 1e3;
      intensities[i] = 100 ;
    }

    ros::Time scan_time = ros::Time::now();
    
   
    //populate the LaserScan message
    sensor_msgs::LaserScan scan;
    scan.header.stamp = scan_time;
    scan.header.frame_id = "PLS200";
    scan.angle_min = -1.57;
    scan.angle_max = 1.57;
    scan.angle_increment = 3.14 / num_readings;
    scan.time_increment = (1 / laser_frequency) / (num_readings);
    scan.range_min = 0.0;
    scan.range_max = 100.0;


    scan.set_ranges_size(num_readings);
    scan.set_intensities_size(num_readings);
    
    for(unsigned int i = 0; i < num_readings; ++i){
      scan.ranges[i] = ranges[i] + escala;
    //  scan.intensities[i] = intensities[i];
    }

    scan_pub.publish(scan);
    ++count;
    //r.sleep();
//    printf("escala = %f\n",escala);
//    escala = escala + 0.001;
  }
}

