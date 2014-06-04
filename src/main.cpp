#include <iomanip>
#include <iostream>
#include <inttypes.h>
#include <signal.h>
#include <termios.h>
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

// Definition of packet from vehicle wrapper
struct VehiclePacket_t {
	uint16_t motor_mask[2];
	uint16_t checksum;
	int16_t collective;
	int16_t yaw;
	int16_t pitch;
	int16_t roll;
} __attribute__((packed));

// Definition of health packet for vehicle wrapper
struct HealthPacket_t {
	uint8_t sync_char[2];
	uint16_t timestamp;
	uint32_t header;
	uint16_t checksum;
	uint16_t data[28];
} __attribute__((packed));

static void sigterm_handler(int sig)
{
	cout << "Received interrupt signal " << sig << endl;
	received_sigterm = sig;
	received_nb_signals++;
	if (received_nb_signals > 3) exit(123);
}

static int youbot_init()
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

	config_file = new youbot::ConfigFile("csl", CONFIG_DIR);
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

static bool parse_vehicle_packet(VehiclePacket_t* const vehicle_packet,
	const uint8_t* const rx_buffer, const int num_bytes_received)
{
	const uint8_t vehicle_packet_size = sizeof(VehiclePacket_t);
	static union bus {
		VehiclePacket_t vehicle_packet;
		uint8_t bytes[vehicle_packet_size];
	} rx;

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
				rx.bytes[num_bytes_stored++] = rx_buffer[i];
		}
	}

	if (num_bytes_stored == vehicle_packet_size)
	{
		// cout << "Collective: " << rx.vehicle_packet.collective
		// 	<< " Yaw:" << rx.vehicle_packet.yaw
		// 	<< " Pitch:" << rx.vehicle_packet.pitch
		// 	<< " Roll:" << rx.vehicle_packet.roll << endl;

		num_bytes_stored = -1;  // Start looking for the header again
		last_byte_received = 0;

		convert_endienness(&rx.vehicle_packet.motor_mask[0]);
		convert_endienness(&rx.vehicle_packet.motor_mask[1]);
		convert_endienness(&rx.vehicle_packet.checksum);
		convert_endienness(&rx.vehicle_packet.collective);
		convert_endienness(&rx.vehicle_packet.yaw);
		convert_endienness(&rx.vehicle_packet.pitch);
		convert_endienness(&rx.vehicle_packet.roll);

		uint16_t computed_checksum = rx.vehicle_packet.motor_mask[0]
			+ rx.vehicle_packet.motor_mask[1]
			+ (uint16_t)(rx.vehicle_packet.collective
			+ rx.vehicle_packet.yaw + rx.vehicle_packet.pitch
			+ rx.vehicle_packet.roll);
		if (rx.vehicle_packet.checksum == computed_checksum)
		{
			*vehicle_packet = rx.vehicle_packet;
			return true;
		}
		else
		{
			cout << "Checksum error: expected " << rx.vehicle_packet.checksum
				<< " but computed " << computed_checksum << endl;
		}
	}

	return false;
}

int form_health_packet(HealthPacket_t* const health_packet)
{
// current
#define MASK_CURRENT1            0x00000001
#define MASK_CURRENT2            0x00000002
#define MASK_CURRENT3            0x00000004
#define MASK_CURRENT4            0x00000008
// temperature
#define MASK_TEMP1               0x00000010
#define MASK_TEMP2               0x00000020
#define MASK_TEMP3               0x00000040
#define MASK_TEMP4               0x00000080
// gyros
#define MASK_X_POS               0x00001000
#define MASK_Y_POS               0x00002000
// voltage
#define MASK_MEASURED_VOLTAGE    0x00010000
#define MASK_ESTIMATED_VOLTAGE   0x00040000
// comm
#define MASK_COMM_ERROR          0x00100000
#define MASK_LATENCY             0x00200000
#define MASK_TICKS_SINCE_LAST    0x00400000
#define MASK_SETTINGS            0x00800000
// motors
#define MASK_DUTY_MOTOR1         0x01000000
#define MASK_DUTY_MOTOR2         0x02000000
#define MASK_DUTY_MOTOR3         0x04000000
#define MASK_DUTY_MOTOR4         0x08000000
// debug
#define MASK_DEBUG_DATA1         0x10000000
#define MASK_DEBUG_DATA2         0x20000000
#define MASK_DEBUG_DATA3         0x40000000
#define MASK_DEBUG_DATA4         0x80000000

	static uint16_t counter = 0;

	health_packet->sync_char[0] = '~';
	health_packet->sync_char[1] = '~';
	health_packet->timestamp = ++counter;
	health_packet->header = MASK_MEASURED_VOLTAGE | MASK_ESTIMATED_VOLTAGE
		| MASK_SETTINGS;
	health_packet->data[0] = (uint16_t)(12.0 * 204.6);
	health_packet->data[1] = (uint16_t)(12.0 * 204.6);
	health_packet->data[2] = 0x0001;

	health_packet->checksum = health_packet->timestamp
		+ (uint16_t)((health_packet->header & 0xFFFF0000) >> 16)
		+ (uint16_t)(health_packet->header & 0x0000FFFF)
		+ health_packet->data[0] + health_packet->data[1] + health_packet->data[2];

	convert_endienness(&health_packet->timestamp);
	convert_endienness(&health_packet->header);
	convert_endienness(&health_packet->checksum);
	convert_endienness(&health_packet->data[0]);
	convert_endienness(&health_packet->data[1]);
	convert_endienness(&health_packet->data[2]);

	// Determine length of: sync characters, timestamp, header, checksum & data
	return sizeof(health_packet->sync_char) + sizeof(health_packet->timestamp)
		+ sizeof(health_packet->header) + sizeof(health_packet->checksum)
		+ 3 * sizeof(uint16_t);
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

	youbot_init();

	if (!arm_exists)
		return 1;

	if (!base_exists)
		return 1;
/*
	Serial serial_csl("/tmp/pty2", 38400);  // vehicle wrapper serial comms
	if (!serial_csl.IsOpen())
		return 1;

	Serial serial_odometry("/tmp/pty4", 38400);  // Odometry data output
	if (!serial_odometry.IsOpen())
		return 1;
*/
	static struct termios oldt, newt;
	tcgetattr( STDIN_FILENO, &oldt);
	newt = oldt;
	// newt.c_iflag &= ~(BRKINT);
	newt.c_lflag &= ~(ICANON | ECHO | VINTR | VQUIT);
	// newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr( STDIN_FILENO, TCSANOW, &newt);

/* c_iflag bits */
cout << "--- c_iflag ---" << endl;
cout << "IGNBRK:  " << (int)((newt.c_iflag & 0000001) == 0000001) << endl;
cout << "BRKINT:  " << (int)((newt.c_iflag & 0000002) == 0000002) << endl;
cout << "IGNPAR:  " << (int)((newt.c_iflag & 0000004) == 0000004) << endl;
cout << "PARMRK:  " << (int)((newt.c_iflag & 0000010) == 0000010) << endl;
cout << "INPCK:   " << (int)((newt.c_iflag & 0000020) == 0000020) << endl;
cout << "ISTRIP:  " << (int)((newt.c_iflag & 0000040) == 0000040) << endl;
cout << "INLCR:   " << (int)((newt.c_iflag & 0000100) == 0000100) << endl;
cout << "IGNCR:   " << (int)((newt.c_iflag & 0000200) == 0000200) << endl;
cout << "ICRNL:   " << (int)((newt.c_iflag & 0000400) == 0000400) << endl;
cout << "IUCLC:   " << (int)((newt.c_iflag & 0001000) == 0001000) << endl;
cout << "IXON:    " << (int)((newt.c_iflag & 0002000) == 0002000) << endl;
cout << "IXANY:   " << (int)((newt.c_iflag & 0004000) == 0004000) << endl;
cout << "IXOFF:   " << (int)((newt.c_iflag & 0010000) == 0010000) << endl;
cout << "IMAXBEL: " << (int)((newt.c_iflag & 0020000) == 0020000) << endl;
cout << "IUTF8:   " << (int)((newt.c_iflag & 0040000) == 0040000) << endl;
/* c_oflag bits */
cout << "--- c_oflag ---" << endl;
cout << "OPOST:   " << (int)((newt.c_oflag & 0000001) == 0000001) << endl;
cout << "OLCUC:   " << (int)((newt.c_oflag & 0000002) == 0000002) << endl;
cout << "ONLCR:   " << (int)((newt.c_oflag & 0000004) == 0000004) << endl;
cout << "OCRNL:   " << (int)((newt.c_oflag & 0000010) == 0000010) << endl;
cout << "ONOCR:   " << (int)((newt.c_oflag & 0000020) == 0000020) << endl;
cout << "ONLRET:  " << (int)((newt.c_oflag & 0000040) == 0000040) << endl;
cout << "OFILL:   " << (int)((newt.c_oflag & 0000100) == 0000100) << endl;
cout << "OFDEL:   " << (int)((newt.c_oflag & 0000200) == 0000200) << endl;
cout << "NLDLY:   " << (int)((newt.c_oflag & 0000400) == 0000400) << endl;
cout << "NL0:     " << (int)((newt.c_oflag & 0000000) == 0000000) << endl;
cout << "NL1:     " << (int)((newt.c_oflag & 0000400) == 0000400) << endl;
cout << "CRDLY:   " << (int)((newt.c_oflag & 0003000) == 0003000) << endl;
cout << "CR0:     " << (int)((newt.c_oflag & 0000000) == 0000000) << endl;
cout << "CR1:     " << (int)((newt.c_oflag & 0001000) == 0001000) << endl;
cout << "CR2:     " << (int)((newt.c_oflag & 0002000) == 0002000) << endl;
cout << "CR3:     " << (int)((newt.c_oflag & 0003000) == 0003000) << endl;
cout << "TABDLY:  " << (int)((newt.c_oflag & 0014000) == 0014000) << endl;
cout << "TAB0:    " << (int)((newt.c_oflag & 0000000) == 0000000) << endl;
cout << "TAB1:    " << (int)((newt.c_oflag & 0004000) == 0004000) << endl;
cout << "TAB2:    " << (int)((newt.c_oflag & 0010000) == 0010000) << endl;
cout << "TAB3:    " << (int)((newt.c_oflag & 0014000) == 0014000) << endl;
cout << "XTABS:   " << (int)((newt.c_oflag & 0014000) == 0014000) << endl;
cout << "BSDLY:   " << (int)((newt.c_oflag & 0020000) == 0020000) << endl;
cout << "BS0:     " << (int)((newt.c_oflag & 0000000) == 0000000) << endl;
cout << "BS1:     " << (int)((newt.c_oflag & 0020000) == 0020000) << endl;
cout << "VTDLY:   " << (int)((newt.c_oflag & 0040000) == 0040000) << endl;
cout << "VT0:     " << (int)((newt.c_oflag & 0000000) == 0000000) << endl;
cout << "VT1:     " << (int)((newt.c_oflag & 0040000) == 0040000) << endl;
cout << "FFDLY:   " << (int)((newt.c_oflag & 0100000) == 0100000) << endl;
cout << "FF0:     " << (int)((newt.c_oflag & 0000000) == 0000000) << endl;
cout << "FF1:     " << (int)((newt.c_oflag & 0100000) == 0100000) << endl;
/* c_lflag bits */
cout << "--- c_lflag ---" << endl;
cout << "ISIG:    " << (int)((newt.c_lflag & 0000001) == 0000001) << endl;
cout << "ICANON:  " << (int)((newt.c_lflag & 0000002) == 0000002) << endl;
cout << "XCASE:   " << (int)((newt.c_lflag & 0000004) == 0000004) << endl;
cout << "ECHO:    " << (int)((newt.c_lflag & 0000010) == 0000010) << endl;
cout << "ECHOE:   " << (int)((newt.c_lflag & 0000020) == 0000020) << endl;
cout << "ECHOK:   " << (int)((newt.c_lflag & 0000040) == 0000040) << endl;
cout << "ECHONL:  " << (int)((newt.c_lflag & 0000100) == 0000100) << endl;
cout << "NOFLSH:  " << (int)((newt.c_lflag & 0000200) == 0000200) << endl;
cout << "TOSTOP:  " << (int)((newt.c_lflag & 0000400) == 0000400) << endl;
cout << "ECHOCTL: " << (int)((newt.c_lflag & 0001000) == 0001000) << endl;
cout << "ECHOPRT: " << (int)((newt.c_lflag & 0002000) == 0002000) << endl;
cout << "ECHOKE:  " << (int)((newt.c_lflag & 0004000) == 0004000) << endl;
cout << "FLUSHO:  " << (int)((newt.c_lflag & 0010000) == 0010000) << endl;
cout << "PENDIN:  " << (int)((newt.c_lflag & 0040000) == 0040000) << endl;
cout << "IEXTEN:  " << (int)((newt.c_lflag & 0100000) == 0100000) << endl;
cout << "EXTPROC: " << (int)((newt.c_lflag & 0200000) == 0200000) << endl;

cout << "VEOL:    " << (int)((newt.c_lflag & VEOL) == VEOL) << endl;
cout << "VINTR:   " << (int)((newt.c_lflag & VINTR) == VINTR) << endl;
cout << "VKILL:   " << (int)((newt.c_lflag & VKILL) == VKILL) << endl;
cout << "VQUIT:   " << (int)((newt.c_lflag & VQUIT) == VQUIT) << endl;

cout << SIGQUIT << endl;
cout << SIGINT << endl;
cout << SIGTERM << endl;

	char keypress = 0;
	while (keypress != 's' && keypress != 'a' && keypress != 3 && keypress != 28
		&& received_sigterm == 0)
	{
		// cout << endl << "Commands:" << endl << endl;
		// cout << "  c - Set the camera" << endl;
		// cout << "  s - start the program" << endl;
		// cout << "  a - abort" << endl << endl;

		// Read a character from stdin then flush the buffer.
		cin >> keypress;
		// while ( getchar() != '\n' ) continue;

		cout << (int)keypress << endl;

		if (keypress == 'c')
		{
			youbot::GripperBarSpacingSetPoint barSetPoint;
			youbot::GripperSensedBarSpacing barSensed;
			barSetPoint.barSpacing = 0.023 * meter;
			youbot_arm->getArmGripper().setData(barSetPoint);
			do {
				SLEEP_MILLISEC(50);
				youbot_arm->getArmGripper().getData(barSensed);
			} while (barSensed.barSpacing < 0.023 * meter);

			SLEEP_MILLISEC(1000);

			barSetPoint.barSpacing = 0.0165 * meter;
			youbot_arm->getArmGripper().setData(barSetPoint);
			do {
				SLEEP_MILLISEC(50);
				youbot_arm->getArmGripper().getData(barSensed);
			} while (barSensed.barSpacing > 0.0165 * meter);

			cout << "Press any key to clamp..." << endl;
			getchar();
			barSetPoint.barSpacing = 0.014 * meter;
			youbot_arm->getArmGripper().setData(barSetPoint);
			do {
				SLEEP_MILLISEC(50);
				youbot_arm->getArmGripper().getData(barSensed);
			} while (barSensed.barSpacing > 0.014 * meter);
		}
	}
/*
	youbot::JointAngleSetpoint target_joint_angle;
	youbot::JointSensedAngle jointSensed;
	target_joint_angle.angle = arm_up[4] * radian;
	youbot_arm->getArmJoint(5).setData(target_joint_angle);
	do {
		SLEEP_MILLISEC(50);
		youbot_arm->getArmJoint(5).getData(jointSensed);
	} while (jointSensed.angle < 2.93 * radian);
	target_joint_angle.angle = arm_up[0] * radian;
	youbot_arm->getArmJoint(1).setData(target_joint_angle);
	SLEEP_MILLISEC(4000);
	arm_position(arm_up);

	// Starting the event loop
	while (received_sigterm == 0)
	{
		uint8_t rx_buffer[1024] = {0};
		int bytes_received = serial_csl.Read(rx_buffer, 128);

		VehiclePacket_t vehicle_packet;
		if (parse_vehicle_packet(&vehicle_packet, rx_buffer, bytes_received))
		{

			try
			{
				youbot_base->setBaseVelocity(
					-0.75 * ((float)vehicle_packet.pitch / 1250. - 1.) * meter_per_second,
					0.75 * ((float)vehicle_packet.roll / 1250. - 1.) * meter_per_second,
					2.0 * ((float)vehicle_packet.yaw / 1250. - 1.) * radian_per_second);
			}
			catch(exception& ex)
			{
				cout << "BaseWHAT: " << ex.what() << endl;
			}

			cout << "pitch: " << -((float)vehicle_packet.pitch / 1250. - 1.)
			<< ", roll: " << ((float)vehicle_packet.roll / 1250. - 1.)
			<< ", yaw: " << ((float)vehicle_packet.yaw / 1250. - 1.) << endl;
		}

		// TODO: form this better (rather than declare a data portion of the packet,
		// declare each component separately...)
		const uint8_t health_packet_size = sizeof(HealthPacket_t);
		static union bus {
			HealthPacket_t health_packet;
			uint8_t bytes[health_packet_size];
		} tx;
		int tx_length = form_health_packet(&tx.health_packet);
		serial_csl.SendBuffer(&tx.bytes[0], tx_length);

		quantity<si::length> longitudinalPosition, transversalPosition;
		quantity<plane_angle> orientation;
		youbot_base->getBasePosition(longitudinalPosition, transversalPosition,
			orientation);

		float x = longitudinalPosition / meter;
		float y = transversalPosition / meter;
		float psi = orientation / radian;

		usleep(10000);
	}

	serial_csl.Close();

	serial_odometry.Close();

	cout << "Please wait while stowing arm." << endl;
	target_joint_angle.angle = arm_down[4] * radian;
	youbot_arm->getArmJoint(5).setData(target_joint_angle);
	SLEEP_MILLISEC(4000);
	target_joint_angle.angle = arm_down[0] * radian;
	youbot_arm->getArmJoint(1).setData(target_joint_angle);
	SLEEP_MILLISEC(4000);
	arm_position(arm_down);

	SLEEP_MILLISEC(4000);
*/
	tcsetattr( STDIN_FILENO, TCSANOW, &oldt);

	return 0;
}
