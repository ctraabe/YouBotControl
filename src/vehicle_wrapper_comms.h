#ifndef VEHICLE_WRAPPER_COMMS_H_
#define VEHICLE_WRAPPER_COMMS_H_

#include <inttypes.h>

#include "serial.h"

// Definition of packet from vehicle wrapper
struct VehiclePacket_t {
  uint16_t motor_mask[2];
  uint16_t checksum;
  int16_t collective;
  int16_t yaw;
  int16_t pitch;
  int16_t roll;
} __attribute__((packed));

void SendHealthPacket(Serial& serial);

bool ParseVehiclePacket(Serial& serial, VehiclePacket_t& vehicle_packet);

#endif // VEHICLE_WRAPPER_COMMS_H_
