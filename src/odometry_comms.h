#ifndef ODOMETRY_COMMS_H_
#define ODOMETRY_COMMS_H_

#include <inttypes.h>

#include "serial.h"

// Definition of packet from vehicle wrapper
struct PoseInfoPacket_t {
  uint8_t Header;
  uint64_t Time; // In microseconds since epoch
  float Position[3];
  float Rotation[4];
  uint8_t TrackingStatus;
  uint16_t Checksum;
} __attribute__((packed));

void SendOdometryPacket(float &x, float &y, float &psi, Serial& serial);

#endif // ODOMETRY_COMMS_H_
