#include "odometry_comms.h"

#include <boost/date_time/posix_time/posix_time.hpp>

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

  boost::posix_time::time_duration timestamp
    = boost::posix_time::microsec_clock::universal_time()
    - boost::posix_time::time_from_string("1970-01-01 00:00:00.000");

  tx.packet.Header = 0xE5;
  tx.packet.Time = timestamp.total_microseconds();
  tx.packet.Position[0] = x;
  tx.packet.Position[1] = y;
  tx.packet.Position[2] = 0.0;
  tx.packet.Rotation[0] = std::cos(0.5 * psi);
  tx.packet.Rotation[1] = 0.0;
  tx.packet.Rotation[2] = 0.0;
  tx.packet.Rotation[3] = std::sin(0.5 * psi);
  tx.packet.TrackingStatus = 3;
  tx.packet.Checksum = 0;

  tx.packet.Checksum = Checksum(tx.bytes, sizeof(tx.packet));

  serial.SendBuffer(&tx.bytes[0], pose_packet_size);
}
