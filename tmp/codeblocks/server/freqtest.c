/* RTAI LXRT Frequency Test - www.captain.at
        This example is for RTAI3.2 and magma (which will be 3.3)
        Produces a square wave on the parallel port
*/

#include <stdio.h>
#include <errno.h>
#include <sys/mman.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <rtai_lxrt.h>
#include <rtai_sem.h>
#include <rtai_usi.h>
#include <sys/io.h>

#include <rtai_shm.h>
#include <rtai_nam2num.h>
#include "parameters.h"
       
static volatile int end = 0;
static volatile int endsquare = 1;
#define BASEPORT 0x378
//             s  m  μ  n
#define PERIOD     100000000


static struct data_str *data;

static void *square_handler(void *args) {
        RT_TASK *handler;
        RTIME period;

        if (!(handler = rt_task_init_schmod(nam2num("SQHDLR"), 0, 0, 0, SCHED_FIFO, 0xF))) {
                printf("CANNOT INIT HANDLER TASK > SQHDLR <\n");
                exit(1);
        }
        rt_allow_nonroot_hrt();
        mlockall(MCL_CURRENT | MCL_FUTURE);
        rt_set_oneshot_mode();
        start_rt_timer(0);
        period = nano2count(PERIOD);
        rt_make_hard_real_time();
        endsquare = 0;
        rt_task_make_periodic(handler, rt_get_time() + period, period);

data = rtai_malloc(nam2num(SHMNAM), sizeof(struct data_str)); //sizeof(struct data_str));



	int counter = 20000;
//        while ( coudatanter > 0 )//!endsquare ) {
	while ( counter >= 0) {
                //outb_p(0, BASEPORT);
                //outb_p(255, BASEPORT);
                //rt_task_wait_period();

		//printf("prueba Test %d \n", counter);
		//usleep(100000);

/*
		data->indx_counter = counter;
		data->sin_value = data->sin_value + 1; //rand();
		data->cos_value = data->cos_value +1; //rand();
*/

		printf(" Counter : %d Sine : %f Cosine : %f \n", data->indx_counter, data->sin_value,     data->cos_value);

		counter = counter - 1;
        	rt_task_wait_period();
                
	}

rtai_free(nam2num(SHMNAM),data);
        stop_rt_timer();
        rt_make_soft_real_time();
        rt_task_delete(handler);
        return 0;
}

// signal-handler, to ensure clean exit on Ctrl-C
void clean_exit(int dummy) { end = 1; }

int main(void) {
        RT_TASK *maint; //, *squaretask;
        int squarethread;

        // install signal handler
        signal(SIGTERM, clean_exit);    
        signal(SIGINT, clean_exit);

	



        if (!(maint = rt_task_init(nam2num("MAIN"), 1, 0, 0))) {
                printf("CANNOT INIT MAIN TASK > MAIN <\n");
                exit(1);
        }

        // ask for permission to access the parallel port from user-space
        if (iopl(3)) {
                printf("iopl err\n");
                rt_task_delete(maint);
                exit(1);
        }

        squarethread = rt_thread_create(square_handler, NULL, 10000);  // create thread
        while (endsquare) {   // wait until thread went to hard real time
                usleep(100000);
		//printf("main \n");
        }

        while (!end) { 
		usleep(100000); 
		//printf("main 2 \n");
	}

        endsquare = 1;
        //printf("TEST ENDS\n");
        rt_thread_join(squarethread);
        rt_task_delete(maint);
        return 0;
}
