#include <gui/screen1_screen/Screen1View.hpp>
#include <gui/screen1_screen/Screen1Presenter.hpp>

Screen1Presenter::Screen1Presenter(Screen1View& v)
    : view(v)
{

}

void Screen1Presenter::activate()
{

}

void Screen1Presenter::deactivate()
{

}

void Screen1Presenter::setScreen (bool State1, bool State2, bool State3, bool State4, bool State5, bool State6, bool State7, bool State8, bool State9, bool State10)
{
	view.setScreen(State1, State2, State3, State4, State5, State6, State7, State8, State9, State10);
}
