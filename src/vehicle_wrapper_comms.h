#ifndef VEHICLE_WRAPPER_COMMS_H_
#define VEHICLE_WRAPPER_COMMS_H_

#include <inttypes.h>

#include "serial.h"

// Definition of packet from vehicle wrapper
struct VehiclePacket_t {
  float channel1;
  float channel2;
  float channel3;
  float channel4;
  uint16_t crc;
} __attribute__((packed));

void SendHealthPacket(Serial& serial);

bool ParseVehiclePacket(Serial& serial, VehiclePacket_t& vehicle_packet);

#endif // VEHICLE_WRAPPER_COMMS_H_
