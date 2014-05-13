#include <signal.h>
#include <unistd.h>
#include <iostream>

#include "youbot/YouBotBase.hpp"
#include "youbot/YouBotManipulator.hpp"


bool END = false;

void sigintHandler(int signal)
{
	END = true;
	printf("End!\n\r");
}

int init()
{
	//check if base hardware exists
	try
	{
		youBotBase = new youbot::YouBotBase("youbot-base", YOUBOT_CONFIGURATIONS_DIR);
		youBotBase->doJointCommutation();
		baseExisting = true;

		// TODO: figure out what this does
		quantity<angular_acceleration> angAcc;
		youbot::MotorAcceleration acceleration;
		acceleration.setParameter( angAcc.from_value(100000.0) );
	}
	catch ( exception& ex )
	{
		std::cout << "BASE Exception: " << ex.what() << endl;
		baseExisting = false;
	}

	return 0;
}

int main()
{
	// Starting the event loop
	while (!END)
	{
		unsigned int i = 0

		try
		{
			if(baseExisting)
			{
				youBotBase->setBaseVelocity((float)i * 0.1 * meter_per_second,
					0.0 * meter_per_second, 0.0 * M_PI *radian_per_second);
			}
			if (i < 12) i++;
		}
		catch(exception& ex)
		{
			cout << "BaseWHAT: " << ex.what() << endl;
		}

		sleep(1);
	}

	return 0;
}
