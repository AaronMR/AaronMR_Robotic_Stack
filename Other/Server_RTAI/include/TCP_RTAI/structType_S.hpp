
class structType {
public:
    virtual char *serialize(char* maki) = 0;
    virtual char *Unserialize(char* maki) = 0;
    virtual void iniSHM(int shm_in, int shm_out, char* SHM_name) = 0;
    void storeData();
};

class struct_Joy : public structType {
public:
    struct_Joy();

    char* serialize(char* maki);
    char* Unserialize(char* maki);
    void storeData(Joy *joy);
    Joy auxJoy1;
    int sizeof_Joy;
    bool haveSubscriber;
    bool havePublisher;

    void iniSHM(int shm_in, int shm_out, char* SHM_name);
    //#############

    struct Joy *dataIN;
	struct Joy *dataOUT;
	float pause ;
	float t;

    //##############

};

class struct_Twist : public structType {
public:
    struct_Twist();
    char* serialize(char* maki);
    char* Unserialize(char* maki);
    void storeData(Joy *joy);
    Joy auxJoy1;
    int sizeof_Joy;

    void iniSHM(int shm_in, int shm_out, char* SHM_name);
    //#############

    struct Twist *dataIN;
	struct Twist *dataOUT;
	float pause ;
	float t;

    //##############
};

class struct_Pose : public structType {
public:
    struct_Pose();
    char* serialize(char* maki);
    char* Unserialize(char* maki);
    void storeData(Joy *joy);
    Joy auxJoy1;

    void iniSHM(int shm_in, int shm_out, char* SHM_name);

    Pose auxPose1;
    int sizeof_Joy;
    bool haveSubscriber;
    bool havePublisher;

    struct Pose *dataIN;
	struct Pose *dataOUT;

};

class struct_posWheels : public structType {
public:
    struct_posWheels();
    char* serialize(char* data2s);
    char* Unserialize(char* data2us);


    posWheels_t posWheels;


    posWheels_t auxSerialize;
    posWheels_t auxUnSerialize;

    void iniSHM(int shm_in, int shm_out, char* SHM_name);
    int sizeof_Joy;
    bool haveSubscriber;
    bool havePublisher;

    struct posWheels_t *dataIN;
	struct posWheels_t *dataOUT;

};
