#include <stdio.h>
#include <math.h>
//#include <unistd.h>
//#include <sys/types.h>
//#include <sys/mman.h>
//#include <sys/stat.h>
//#include <fcntl.h>
//#include <signal.h>
#include <rtai_shm.h>
#include "comStruc.h"




int main (void) {
	struct comStruc_IN *dataIN = rtai_malloc (nam2num(SHMNAM_IN), sizeof(struct comStruc_IN)) ;
	struct comStruc_OUT *dataOUT = rtai_malloc (nam2num(SHMNAM_OUT), sizeof(struct comStruc_OUT)) ;
	float pause = 100000;
	float t = 0;



	dataIN->index_counter = 0;
	while (1) {
		dataIN->index_counter = dataIN->index_counter + 1;
		dataIN->sin_value = (float)t*10000;
		dataIN->cos_value = t*10000+100.0;
		printf(" Counter : %d Sine : %f Cosine : %f \n", dataOUT->index_counter, dataOUT->sin_value,     dataOUT->cos_value);
		usleep(pause);
		t=t+1.0/pause;
		printf("%f\n",t*1000);
	}

	rtai_free (nam2num(SHMNAM_IN), dataIN);
	rtai_free (nam2num(SHMNAM_OUT), dataOUT);
	return 0;
}
