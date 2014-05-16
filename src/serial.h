#ifndef SERIAL_H_
#define SERIAL_H_

#include <string>

#include <termios.h>

class Serial
{
public:
  Serial(const std::string &comport, const int baudrate);
  bool IsOpen() { return id_ != -1; }
  int Read(unsigned char* const buffer, const int length);
  int SendByte(const unsigned char byte);
  int SendBuffer(const unsigned char* const buffer, const int length);
  void Close();

private:
  int id_;
  struct termios original_port_settings_;
};

#endif // SERIAL_H_