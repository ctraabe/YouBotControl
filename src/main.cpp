#include <iomanip>
#include <iostream>
#include <inttypes.h>
#include <signal.h>
#include <unistd.h>

#include "youbot/YouBotBase.hpp"
#include "youbot/YouBotManipulator.hpp"
#include "serial.h"


static youbot::YouBotBase* youbot_base = NULL;
static youbot::YouBotManipulator* youbot_arm = NULL;
static youbot::ConfigFile* config_file = NULL;
static bool arm_exists = false, base_exists = false;
static volatile int received_sigterm = 0;
static volatile int received_nb_signals = 0;
static float arm_up[5] = {1.0, 1.0, -1.0, 1.0, 1.0},
	arm_down[5] = {0.11, 0.11, -0.11, 0.11, 0.12};

// Definition of packet from PTAM
struct PTAMPacket_t {
	uint8_t header;
	uint64_t time; // In microseconds since epoch
	float position[3];
	float yaw;
	float pitch;
	float roll;
	uint8_t has_tracking;
	uint16_t checksum;
} __attribute__((packed));

// Definition of packet from PTAM
struct VehiclePacket_t {
	uint16_t motor_mask[2];
	uint16_t checksum;
	int16_t collective;
	int16_t yaw;
	int16_t pitch;
	int16_t roll;
} __attribute__((packed));


static void sigterm_handler(int sig)
{
	received_sigterm = sig;
	received_nb_signals++;
	if (received_nb_signals > 3) exit(123);
}

static int init()
{
	//check if base hardware exists
	try
	{
		youbot_base = new youbot::YouBotBase("youbot-base",
			YOUBOT_CONFIGURATIONS_DIR);
		youbot_base->doJointCommutation();
		base_exists = true;

		// TODO: figure out what this does
		quantity<angular_acceleration> angAcc;
		youbot::MotorAcceleration acceleration;
		acceleration.setParameter( angAcc.from_value(100000.0) );
	}
	catch (exception& ex)
	{
		cout << "BASE Exception: " << ex.what() << endl;
		base_exists = false;
	}

	try
	{
		youbot_arm = new youbot::YouBotManipulator("youbot-manipulator",
			YOUBOT_CONFIGURATIONS_DIR);
		youbot_arm->doJointCommutation();
		youbot_arm->calibrateManipulator();

		arm_exists = true;
	}
	catch (exception& ex)
	{
		cout << "ARM Exception: " << ex.what() << endl;
		arm_exists = false;
	}

	config_file = new youbot::ConfigFile("youbot_control", CONFIG_DIR);
	if( !config_file )
	{
		cout << "Could not load file: " << CONFIG_DIR << config_file
			<< " Check if it exists and the path is correct" << endl;
		return -1;
	}

	if ( config_file->sectionExists("ARM_UP") )
	{
		config_file->readInto(arm_up[0], "ARM_UP", "A1");
		config_file->readInto(arm_up[1], "ARM_UP", "A2");
		config_file->readInto(arm_up[2], "ARM_UP", "A3");
		config_file->readInto(arm_up[3], "ARM_UP", "A4");
		config_file->readInto(arm_up[4], "ARM_UP", "A5");
	}
	else
	{
		cout << "No \"ARM_UP\" section found in the config-file" << endl;
	}

	if ( config_file->sectionExists("ARM_DOWN") )
	{
		config_file->readInto(arm_down[0], "ARM_DOWN", "A1");
		config_file->readInto(arm_down[1], "ARM_DOWN", "A2");
		config_file->readInto(arm_down[2], "ARM_DOWN", "A3");
		config_file->readInto(arm_down[3], "ARM_DOWN", "A4");
		config_file->readInto(arm_down[4], "ARM_DOWN", "A5");
	}
	else
	{
		cout << "No \"ARM_DOWN\" section found in the config-file" << endl;
	}

	return 0;
}

static uint16_t checksum(const uint8_t* data, size_t length)
{
	uint16_t sum = 0;

	for (size_t i = 0; i < length; ++i)
		sum += data[i];

	return sum;
}

static void convert_endienness(uint16_t* word)
{
	uint8_t* byte_array = static_cast<uint8_t*>(static_cast<void*>(word));
	uint8_t temp = byte_array[0];
	byte_array[0] = byte_array[1];
	byte_array[1] = temp;
}

static void convert_endienness(int16_t* word)
{
	uint8_t* byte_array = static_cast<uint8_t*>(static_cast<void*>(word));
	uint8_t temp = byte_array[0];
	byte_array[0] = byte_array[1];
	byte_array[1] = temp;
}

static void convert_endienness(uint32_t* word) {
	uint8_t* byte_array = static_cast<uint8_t*>(static_cast<void*>(word));
	uint8_t temp = byte_array[0];
	byte_array[0] = byte_array[1];
	byte_array[1] = byte_array[2];
	byte_array[2] = byte_array[0];
	byte_array[0] = byte_array[3];
	byte_array[3] = temp;
}

static bool parse_ptam_packet(PTAMPacket_t* const ptam_packet,
	const uint8_t* const rx_buffer, const int num_bytes_received)
{
	const uint8_t ptam_packet_size = sizeof(PTAMPacket_t);
	static union bus {
		PTAMPacket_t ptam_packet;
		uint8_t bytes[ptam_packet_size];
	} rx_packet;

	static int num_bytes_stored = -1;  // -1 indicates header not yet received

	for (int i = 0; (i < num_bytes_received)
		&& (num_bytes_stored < ptam_packet_size); ++i)
	{
		if ((num_bytes_stored > 0) || (rx_buffer[i] == 0xE5))
			rx_packet.bytes[num_bytes_stored++] = rx_buffer[i];
	}

	if (num_bytes_stored == ptam_packet_size)
	{
		num_bytes_stored = 0;
		if (checksum(rx_packet.bytes, ptam_packet_size - 2)
			== rx_packet.ptam_packet.checksum)
		{
			*ptam_packet = rx_packet.ptam_packet;
			return true;
		}
		else
		{
			cout << "Checksum error." << endl;
		}
	}

	return false;
}

static bool parse_vehicle_packet(VehiclePacket_t* const vehicle_packet,
	const uint8_t* const rx_buffer, const int num_bytes_received)
{
	const uint8_t vehicle_packet_size = sizeof(VehiclePacket_t);
	static union bus {
		VehiclePacket_t vehicle_packet;
		uint8_t bytes[vehicle_packet_size];
	} rx_packet;

	static int num_bytes_stored = -1;  // -1 indicates header not yet received
	static uint8_t last_byte_received = 0;

	for (int i = 0; i < num_bytes_received; ++i)
	{
		if (num_bytes_stored < 0)  // Header not yet received
		{
			if ((last_byte_received == '~') && (rx_buffer[i] == '~'))
				num_bytes_stored = 0;  // indicates that header has been received
			else
				last_byte_received = rx_buffer[i];
		}
		else  // Header received
		{
			if (num_bytes_stored < vehicle_packet_size)
				rx_packet.bytes[num_bytes_stored++] = rx_buffer[i];
		}
	}

	if (num_bytes_stored == vehicle_packet_size)
	{
		// cout << "Collective: " << rx_packet.vehicle_packet.collective
		// 	<< " Yaw:" << rx_packet.vehicle_packet.yaw
		// 	<< " Pitch:" << rx_packet.vehicle_packet.pitch
		// 	<< " Roll:" << rx_packet.vehicle_packet.roll << endl;

		num_bytes_stored = -1;  // Start looking for the header again
		last_byte_received = 0;

		convert_endienness(&rx_packet.vehicle_packet.motor_mask[0]);
		convert_endienness(&rx_packet.vehicle_packet.motor_mask[1]);
		convert_endienness(&rx_packet.vehicle_packet.checksum);
		convert_endienness(&rx_packet.vehicle_packet.collective);
		convert_endienness(&rx_packet.vehicle_packet.yaw);
		convert_endienness(&rx_packet.vehicle_packet.pitch);
		convert_endienness(&rx_packet.vehicle_packet.roll);

		uint16_t computed_checksum = rx_packet.vehicle_packet.motor_mask[0]
			+ rx_packet.vehicle_packet.motor_mask[1]
			+ (uint16_t)(rx_packet.vehicle_packet.collective
			+ rx_packet.vehicle_packet.yaw + rx_packet.vehicle_packet.pitch
			+ rx_packet.vehicle_packet.roll);
		if (rx_packet.vehicle_packet.checksum == computed_checksum)
		{
			*vehicle_packet = rx_packet.vehicle_packet;
			return true;
		}
		else
		{
			cout << "Checksum error: expected " << rx_packet.vehicle_packet.checksum
				<< " but computed " << computed_checksum << endl;
		}
	}

	return false;
}

void arm_position(const float position_array[5])
{
	youbot::JointAngleSetpoint target_joint_angle;
	target_joint_angle.angle = position_array[0] * radian;
	youbot_arm->getArmJoint(1).setData(target_joint_angle);
	target_joint_angle.angle = position_array[1] * radian;
	youbot_arm->getArmJoint(2).setData(target_joint_angle);
	target_joint_angle.angle = position_array[2] * radian;
	youbot_arm->getArmJoint(3).setData(target_joint_angle);
	target_joint_angle.angle = position_array[3] * radian;
	youbot_arm->getArmJoint(4).setData(target_joint_angle);
	target_joint_angle.angle = position_array[4] * radian;
	youbot_arm->getArmJoint(5).setData(target_joint_angle);
}

int main()
{
	signal(SIGQUIT, sigterm_handler); /* Quit (POSIX).  */
	signal(SIGINT , sigterm_handler); /* Interrupt (ANSI).  */
	signal(SIGTERM, sigterm_handler); /* Termination (ANSI).  */

	init();

	if (!arm_exists)
		return 1;

	if (!base_exists)
		return 1;

	// Serial serial("/dev/ttyS0", 38400);  // Serial port
	Serial serial("/tmp/pty2", 38400);  // Virtual serial port
	if (!serial.IsOpen())
		return 1;

	arm_position(arm_up);

	// Starting the event loop
	while (received_sigterm == 0)
	{
		uint8_t rx_buffer[1024] = {0};
		int bytes_received = serial.Read(rx_buffer, 128);
/*
		PTAMPacket_t ptam_packet;
		if (parse_ptam_packet(&ptam_packet, rx_buffer, bytes_received))
		{
			cout << ptam_packet.position[2] << endl;

			try
			{
				youbot_base->setBaseVelocity((1.0 - ptam_packet.position[2])
					* meter_per_second, 0.0 * meter_per_second, 0.0
					* M_PI * radian_per_second);
			}
			catch(exception& ex)
			{
				cout << "BaseWHAT: " << ex.what() << endl;
			}
		}
*/

		VehiclePacket_t vehicle_packet;
		parse_vehicle_packet(&vehicle_packet, rx_buffer, bytes_received);

		usleep(10000);
	}

	serial.Close();

	cout << "Please wait while stowing arm." << endl;
	arm_position(arm_down);

	SLEEP_MILLISEC(4000);

	return 0;
}
