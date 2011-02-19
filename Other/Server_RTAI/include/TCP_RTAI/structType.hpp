
class structType {
public:
    virtual char *serialize(char* maki) = 0;
    virtual char *Unserialize(char* maki) = 0;
    virtual void iniSHM(int shm_in, int shm_out) = 0;
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

    void iniSHM(int shm_in, int shm_out);
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
    bool haveSubscriber;
    bool havePublisher;

    void iniSHM(int shm_in, int shm_out);
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

    void iniSHM(int shm_in, int shm_out);

    Pose auxPose1;
    int sizeof_Joy;
    bool haveSubscriber;
    bool havePublisher;

    struct Pose *dataIN;
	struct Pose *dataOUT;

};
