
#define SHMNAM_IN "SHMIN"
#define SHMNAM_OUT "SHMOUT"




//#######################################

/**********************************************************************/

#ifndef PARAMETERS
#define PARAMETERS



#define TICK_PERIOD 1000000
#define TASK_PRIORITY 1
#define STACK_SIZE 10000
#define SHMNAM "MIRSHM"

#include <sstream>

using namespace std;


struct comStruc_IN
{
	double x1;
	double y1;
    double x2;
	double y2;
    int a;
	int b;
    int c;
	int d;
	bool newValue; // flag to true, when have new value
};

struct comStruc_OUT
{
	double x1;
	double y1;
    double x2;
	double y2;
    int a;
	int b;
    int c;
	int d;
};

struct data_str // OK
{
    int indx_counter;
    float sin_value;
    float cos_value;
};

struct Struct_1 // OK
{
    int indx_counter;
    float sin_value;
    float cos_value;
};

struct Struct_2 // OK
{
    int indx_counter;
    float sin_value;
    float cos_value;
    float new_value_1;
    float new_value_2;
    float new_value_3;
    float new_value_4;
};

struct Struct_3 // OK
{
    float axes[4];
    int buttons[4];
};


struct Joy // Joystick
{
    float axes[4];
    int buttons[4];
};

struct Joy2 // Joystick
{
    double axes[4];
    int buttons[4];
};

struct Point
{
    double x;
    double y;
    double z;
};

struct Quaternion
{
    double x;
    double y;
    double z;
    double w;
};

struct Pose
{
    Point position;
    Quaternion orientation;
};

struct Vector3
{
    double x;
    double y;
    double z;
};

struct Twist // OK
{
    bool newValue;
    Vector3  linear;
    Vector3  angular;

};


struct Header
{
    // Standard metadata for higher-level stamped data types.
    // This is generally used to communicate timestamped data
    // in a particular coordinate frame.
    //
    // sequence ID: consecutively increasing ID
    //uint32 seq;

    // Two-integer timestamp that is expressed as:
    // * stamp.secs: seconds (stamp_secs) since epoch
    // * stamp.nsecs: nanoseconds since stamp_secs
    // time-handling sugar is provided by the client library
    //time stamp;

    //Frame this data is associated with
    // 0: no frame
    // 1: global frame
    //string frame_id;
};

struct PoseWithCovariance
{
    // This represents a pose in free space with uncertainty.
    Pose pose;

    // Row-major representation of the 6x6 covariance matrix
    // The orientation parameters use a fixed-axis representation.
    // In order, the parameters are:
    // (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
    double covariance[36];
};

struct TwistWithCovariance
{
    // This expresses velocity in free space with uncertianty.
    Twist twist;

    // Row-major representation of the 6x6 covariance matrix
    // The orientation parameters use a fixed-axis representation.
    // In order, the parameters are:
    // (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
    double covariance[36];
};

struct odometry_t
{
    //Header header;  // Falta por hacer el Header
    //string child_frame_id;
    PoseWithCovariance pose;
    TwistWithCovariance twist;
};

struct struct_Template_t
{
    double val1;
    double val2;
    double val3;
};

struct posWheels_t
{
    Point pos_W[4];
};


#endif /*PARAMETERS*/
/**********************************************************************/
