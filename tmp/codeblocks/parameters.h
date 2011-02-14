#define TICK_PERIOD 1000000
#define TASK_PRIORITY 1
#define STACK_SIZE 10000
#define SHMNAM "MIRSHM"

struct data_str
{
	int indx_counter;
	float sin_value;
	float cos_value;
};

struct data_str2
{
	double x;
	double y;
	double z;
};



