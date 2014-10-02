#include "vehicle_wrapper_comms.h"

#include <iostream>

// Health packet data specifiers:
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

// Definition of health packet for vehicle wrapper
struct HealthPacket_t {
  uint8_t sync_char[2];
  uint16_t timestamp;
  uint32_t header;
  uint16_t checksum;
  uint16_t measured_voltage;
  uint16_t estimated_voltage;
  uint16_t ticks_since_last;
  uint16_t settings;
} __attribute__((packed));

static void ConvertEndienness(uint16_t* word)
{
  uint8_t* byte_array = static_cast<uint8_t*>(static_cast<void*>(word));
  uint8_t temp = byte_array[0];
  byte_array[0] = byte_array[1];
  byte_array[1] = temp;
}

static void ConvertEndienness(int16_t* word)
{
  uint8_t* byte_array = static_cast<uint8_t*>(static_cast<void*>(word));
  uint8_t temp = byte_array[0];
  byte_array[0] = byte_array[1];
  byte_array[1] = temp;
}

static void ConvertEndienness(uint32_t* word) {
  uint8_t* byte_array = static_cast<uint8_t*>(static_cast<void*>(word));
  uint8_t temp = byte_array[0];
  byte_array[0] = byte_array[1];
  byte_array[1] = byte_array[2];
  byte_array[2] = byte_array[0];
  byte_array[0] = byte_array[3];
  byte_array[3] = temp;
}

void SendHealthPacket(Serial& serial)
{
  static uint16_t counter = 0;
  const uint8_t health_packet_size = sizeof(HealthPacket_t);
  static union bus {
    HealthPacket_t health_packet;
    uint8_t bytes[health_packet_size];
  } tx;

  tx.health_packet.sync_char[0] = '~';
  tx.health_packet.sync_char[1] = '~';
  tx.health_packet.timestamp = ++counter;
  tx.health_packet.header = MASK_MEASURED_VOLTAGE | MASK_ESTIMATED_VOLTAGE
    | MASK_TICKS_SINCE_LAST | MASK_SETTINGS;
  tx.health_packet.measured_voltage = (uint16_t)(12.0 * 204.6 / 2.6);
  tx.health_packet.estimated_voltage = (uint16_t)(12.0 * 204.6 / 2.6);
  tx.health_packet.ticks_since_last = 2;  // Harcoded value matches segway
  tx.health_packet.settings = 0x0001;

  tx.health_packet.checksum = tx.health_packet.timestamp
    + (uint16_t)((tx.health_packet.header & 0xFFFF0000) >> 16)
    + (uint16_t)(tx.health_packet.header & 0x0000FFFF)
    + tx.health_packet.measured_voltage
    + tx.health_packet.estimated_voltage
    + tx.health_packet.ticks_since_last
    + tx.health_packet.settings;

  ConvertEndienness(&tx.health_packet.timestamp);
  ConvertEndienness(&tx.health_packet.header);
  ConvertEndienness(&tx.health_packet.checksum);
  ConvertEndienness(&tx.health_packet.measured_voltage);
  ConvertEndienness(&tx.health_packet.estimated_voltage);
  ConvertEndienness(&tx.health_packet.ticks_since_last);
  ConvertEndienness(&tx.health_packet.settings);

  serial.SendBuffer(&tx.bytes[0], health_packet_size);
}

bool ParseVehiclePacket(Serial& serial, VehiclePacket_t& vehicle_packet)
{
  uint8_t rx_buffer[1024] = {0};
  int num_bytes_received = serial.Read(rx_buffer, 1024);

  const uint8_t vehicle_packet_size = sizeof(VehiclePacket_t);
  static union bus {
    VehiclePacket_t packet;
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
    num_bytes_stored = -1;  // Start looking for the header again
    last_byte_received = 0;

    union {
      uint16_t uint16;
      uint8_t bytes[2];
    } computed_crc;

    // CRC
    computed_crc.bytes[0] = '~' + '~';
    computed_crc.bytes[1] = '~' + computed_crc.bytes[0];
    for (unsigned i = 0; i < vehicle_packet_size - 2; ++i)
    {
      computed_crc.bytes[0] += rx.bytes[i];
      computed_crc.bytes[1] += computed_crc.bytes[0];
    }

    if (rx.packet.crc == computed_crc.uint16)
    {
      vehicle_packet = rx.packet;
      return true;
    }
    else
    {
      std::cout << "Checksum error: expected " << rx.packet.crc
        << " but computed " << computed_crc.uint16 << std::endl;
    }
  }

  return false;
}
