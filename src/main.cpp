#include <iomanip>
#include <iostream>
#include <inttypes.h>
#include <signal.h>
#include <unistd.h>

#include "youbot/YouBotBase.hpp"
#include "youbot/YouBotManipulator.hpp"

#include "csl_youbot.h"
#include "read_csl_config.h"
#include "serial.h"
#include "start_menu.h"

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
  received_sigterm = sig;
  received_nb_signals++;
  if (received_nb_signals > 3) exit(123);
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
    //  << " Yaw:" << rx.vehicle_packet.yaw
    //  << " Pitch:" << rx.vehicle_packet.pitch
    //  << " Roll:" << rx.vehicle_packet.roll << endl;

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

int main()
{
  signal(SIGQUIT, sigterm_handler); /* Quit (POSIX).  */
  signal(SIGINT , sigterm_handler); /* Interrupt (ANSI).  */
  signal(SIGTERM, sigterm_handler); /* Termination (ANSI).  */
/*
  Serial serial_csl("/tmp/pty2", 38400);  // vehicle wrapper serial comms
  if (!serial_csl.IsOpen())
    return 1;

  Serial serial_odometry("/tmp/pty4", 38400);  // Odometry data output
  if (!serial_odometry.IsOpen())
    return 1;
*/
  try
  {
    ReadCSLConfig(arm_up, arm_down);

    CSLYouBot csl_youbot;

    StartMenu(received_sigterm);

    //Quit here if abort was called during start menu.
    if (received_sigterm)
      return 0;

    for (int joint = 1; joint <= ARMJOINTS; ++joint)
      csl_youbot.SetJointAngle(joint, arm_down[joint-1] * radian);

    csl_youbot.SetJointAngle(5, arm_up[4] * radian);
    csl_youbot.WaitForArm(received_sigterm);

    csl_youbot.SetJointAngle(1, arm_up[0] * radian);
    csl_youbot.WaitForArm(received_sigterm);

    csl_youbot.SetJointAngle(2, arm_up[1] * radian);
    csl_youbot.SetJointAngle(3, arm_up[2] * radian);
    csl_youbot.SetJointAngle(4, arm_up[3] * radian);
    csl_youbot.WaitForArm(received_sigterm);

    // Clear the interrupt (user must push ctrl-C again to cancel stow).
    received_sigterm = 0;

    csl_youbot.SetJointAngle(2, arm_down[1] * radian);
    csl_youbot.SetJointAngle(3, arm_down[2] * radian);
    csl_youbot.SetJointAngle(4, arm_down[3] * radian);
    csl_youbot.WaitForArm(received_sigterm);

    csl_youbot.SetJointAngle(1, arm_down[0] * radian);
    csl_youbot.WaitForArm(received_sigterm);

    csl_youbot.SetJointAngle(5, arm_down[4] * radian);
    csl_youbot.WaitForArm(received_sigterm);
  }
  catch (exception& ex)
  {
    cout << "Exception: " << ex.what() << endl;
  }
/*
  // Starting the event loop
  while (!received_sigterm)
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
*/
  return 0;
}
