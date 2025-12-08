#ifndef MODEL_HPP
#define MODEL_HPP

class ModelListener;

class Model
{
public:
    Model();

    void bind(ModelListener* listener)
    {
        modelListener = listener;
    }

    void tick();
protected:
    ModelListener* modelListener;
    bool RFID_state;
    bool Finger_state;
    bool Voice_state;
    bool Grabando_state;
    bool Fail_state;
    bool OK_state;
    bool Diego_state;
    bool Alan_state;
    bool Adrian_state;
    bool Blocked_state;
};

#endif // MODEL_HPP
