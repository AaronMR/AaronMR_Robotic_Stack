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

struct Struct_1
{
    int indx_counter;
	float sin_value;
	float cos_value;
};

struct Struct_2
{
    int indx_counter;
	float sin_value;
	float cos_value;
	float new_value_1;
	float new_value_2;
	float new_value_3;
	float new_value_4;
};

struct Struct_3 // Joystick
{
    float axes[4];
    int buttons[5];
};
