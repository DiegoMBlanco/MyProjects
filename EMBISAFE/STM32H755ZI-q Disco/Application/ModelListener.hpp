#ifndef MODELLISTENER_HPP
#define MODELLISTENER_HPP

#include <gui/model/Model.hpp>

class ModelListener
{
public:
    ModelListener() : model(0) {}
    
    virtual ~ModelListener() {}

    void bind(Model* m)
    {
        model = m;
    }

    virtual void setScreen(bool State1, bool State2, bool State3, bool State4, bool State5, bool State6, bool State7, bool State8, bool State9, bool State10){};
protected:
    Model* model;
};

#endif // MODELLISTENER_HPP
