#ifndef SCREEN1VIEW_HPP
#define SCREEN1VIEW_HPP

#include <gui_generated/screen1_screen/Screen1ViewBase.hpp>
#include <gui/screen1_screen/Screen1Presenter.hpp>

class Screen1View : public Screen1ViewBase
{
public:
    Screen1View();
    virtual ~Screen1View() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    virtual void setScreen (bool State1, bool State2, bool State3, bool State4, bool State5, bool State6, bool State7, bool State8, bool State9, bool State10);
protected:
};

#endif // SCREEN1VIEW_HPP
