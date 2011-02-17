
class structType {
public:
    virtual char *serialize(char* maki) = 0;
    virtual char *Unserialize(char* maki) = 0;
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

    //#############

    struct comStruc_IN *dataIN;
	struct comStruc_OUT *dataOUT;
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

    Pose auxPose1;
    int sizeof_Joy;
    bool haveSubscriber;
    bool havePublisher;

};
