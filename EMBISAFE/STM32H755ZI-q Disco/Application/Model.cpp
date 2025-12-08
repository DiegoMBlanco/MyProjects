#include <gui/model/Model.hpp>
#include <gui/model/ModelListener.hpp>

#ifndef SIMULATOR
#include "main.h"
#endif

Model::Model() : modelListener(0), RFID_state(false), Finger_state(false), Voice_state(false), Grabando_state(false), Fail_state(false), OK_state(false), Diego_state(false), Alan_state(false), Adrian_state(false), Blocked_state(false)
{

}

void Model::tick()
{
#ifndef SIMULATOR

	int inputState =
			(HAL_GPIO_ReadPin(GPIOK, GPIO_PIN_1) << 3)|
			(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_6) << 2)|
			(HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_8) << 1)|
			(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) << 0);

	switch(inputState)
	{
	    case 0b0000:
	        RFID_state = false;
	        Finger_state = false;
	        Voice_state = false;
	        Grabando_state = false;
	        Fail_state = false;
	        OK_state = false;
	        Diego_state = false;
	        Alan_state = false;
	        Adrian_state = false;
	        Blocked_state = false;

	        break;

	    case 0b0001:
	        RFID_state = true;
	        Finger_state = false;
	        Voice_state = false;
	        Grabando_state = false;
	        Fail_state = false;
	        OK_state = false;
	        Diego_state = false;
	        Alan_state = false;
	        Adrian_state = false;
	        Blocked_state = false;

	        break;

	    case 0b0010:
	        RFID_state = false;
	        Finger_state = true;
	        Voice_state = false;
	        Grabando_state = false;
	        Fail_state = false;
	        OK_state = false;
	        Diego_state = false;
	        Alan_state = false;
	        Adrian_state = false;
	        Blocked_state = false;

	        break;

	    case 0b0011:
	        RFID_state = false;
	        Finger_state = false;
	        Voice_state = true;
	        Grabando_state = false;
	        Fail_state = false;
	        OK_state = false;
	        Diego_state = false;
	        Alan_state = false;
	        Adrian_state = false;
	        Blocked_state = false;

	        break;

	    case 0b0100:
	        RFID_state = false;
	        Finger_state = false;
	        Voice_state = false;
	        Grabando_state = true;
	        Fail_state = false;
	        OK_state = false;
	        Diego_state = false;
	        Alan_state = false;
	        Adrian_state = false;
	        Blocked_state = false;

	        break;

	    case 0b0101:
	        RFID_state = false;
	        Finger_state = false;
	        Voice_state = false;
	        Grabando_state = false;
	        Fail_state = true;
	        OK_state = false;
	        Diego_state = false;
	        Alan_state = false;
	        Adrian_state = false;
	        Blocked_state = false;

	       break;

	    case 0b0110:
	        RFID_state = false;
	        Finger_state = false;
	        Voice_state = false;
	        Grabando_state = false;
	        Fail_state = false;
	        OK_state = true;
	        Diego_state = false;
	        Alan_state = false;
	        Adrian_state = false;
	        Blocked_state = false;

	    	break;

	    case 0b0111:
	        RFID_state = false;
	        Finger_state = false;
	        Voice_state = false;
	        Grabando_state = false;
	        Fail_state = false;
	        OK_state = false;
	        Diego_state = true;
	        Alan_state = false;
	        Adrian_state = false;
	        Blocked_state = false;

	    	break;

	    case 0b1000:
	        RFID_state = false;
	        Finger_state = false;
	        Voice_state = false;
	        Grabando_state = false;
	        Fail_state = false;
	        OK_state = false;
	        Diego_state = false;
	        Alan_state = true;
	        Adrian_state = false;
	        Blocked_state = false;

	    	break;

	    case 0b1001:
	        RFID_state = false;
	        Finger_state = false;
	        Voice_state = false;
	        Grabando_state = false;
	        Fail_state = false;
	        OK_state = false;
	        Diego_state = false;
	        Alan_state = false;
	        Adrian_state = true;
	        Blocked_state = false;
	    	break;

	    case 0b1010:
	        RFID_state = false;
	        Finger_state = false;
	        Voice_state = false;
	        Grabando_state = false;
	        Fail_state = false;
	        OK_state = false;
	        Diego_state = false;
	        Alan_state = false;
	        Adrian_state = false;
	        Blocked_state = true;

	        break;

	    default:
	        RFID_state = true;
	        Finger_state = false;
	        Voice_state = false;
	        Grabando_state = false;
	        Fail_state = false;
	        OK_state = false;
	        Diego_state = false;
	        Alan_state = false;
	        Adrian_state = false;
	        Blocked_state = false;

	        break;

	}

	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13))
	{
		HAL_GPIO_WritePin(GPIOH, GPIO_PIN_15, GPIO_PIN_SET);
	}
	else HAL_GPIO_WritePin(GPIOH, GPIO_PIN_15, GPIO_PIN_RESET);


#endif
	modelListener->setScreen(RFID_state, Finger_state, Voice_state, Grabando_state, Fail_state, OK_state, Diego_state, Alan_state, Adrian_state, Blocked_state);
}
