#include <gui/screen1_screen/Screen1View.hpp>

Screen1View::Screen1View()
{

}

void Screen1View::setupScreen()
{
    Screen1ViewBase::setupScreen();
}

void Screen1View::tearDownScreen()
{
    Screen1ViewBase::tearDownScreen();
}

void Screen1View::setScreen (bool State1, bool State2, bool State3, bool State4, bool State5, bool State6, bool State7, bool State8, bool State9, bool State10)
{
	icon_base.setVisible(State1);
	icon_base.invalidate();

	icon_1.setVisible(State2);
	icon_1.invalidate();

	icon_2.setVisible(State3);
	icon_2.invalidate();

	icon_3.setVisible(State4);
	icon_3.invalidate();

	icon_4.setVisible(State5);
	icon_4.invalidate();

	icon_5.setVisible(State6);
	icon_5.invalidate();

	icon_6.setVisible(State7);
	icon_6.invalidate();

	icon_7.setVisible(State8);
	icon_7.invalidate();

	icon_8.setVisible(State9);
	icon_8.invalidate();

	icon_9.setVisible(State10);
	icon_9.invalidate();
}
