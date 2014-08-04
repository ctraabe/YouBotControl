#include "odometry_comms.h"

// #include "<chrono>"

static uint16_t Checksum(const uint8_t* data, size_t length)
{
  uint16_t sum = 0;

  for (size_t i = 0; i < length; ++i)
    sum += data[i];

  return sum;
}

void SendOdometryPacket(float &x, float &y, float &psi, Serial& serial)
{
  const uint8_t pose_packet_size = sizeof(PoseInfoPacket_t);
  union bus {
    PoseInfoPacket_t packet;
    uint8_t bytes[pose_packet_size];
  } tx;

  // auto timestamp = std::chrono::duration_cast<
  //     std::chrono::microseconds>(mtpPoseTime.time_since_epoch()).count();
  static unsigned timestamp = 0;

  tx.packet.Header = 0xE5;
  tx.packet.Time = ++timestamp;
  tx.packet.Position[0] = x;
  tx.packet.Position[1] = y;
  tx.packet.Position[2] = 0.0;
  tx.packet.Yaw = psi;
  tx.packet.Pitch = 0.0;
  tx.packet.Roll = 3.14;
  tx.packet.HasTracking = 3;  // Temporarily match PTAM output
  tx.packet.Checksum = 0;

  tx.packet.Checksum = Checksum(tx.bytes, sizeof(tx.packet));

  serial.SendBuffer(&tx.bytes[0], pose_packet_size);
}
