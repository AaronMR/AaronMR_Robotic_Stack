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

/*
// Point
struct Point
{
    float64 x;
    float64 y;
    float64 z;
};


struct Quaternion
{
    float64 x
    float64 y
    float64 z
    float64 w
}



struct Pose
{
    Point position
    Quaternion orientation
}

struct Vector3
{
    float64 x
    float64 y
    float64 z
}

struct Twist
{
    Vector3  linear
    Vector3  angular
}

struct Odometry
{
    Header header
    string child_frame_id
    geometry_msgs/PoseWithCovariance pose
    geometry_msgs/TwistWithCovariance twist
}


'/'
