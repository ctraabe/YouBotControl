#ifndef READ_CONFIG_H_
#define READ_CONFIG_H_

#include <string>

int ReadCSLConfig(float (&arm_up)[5], float (&arm_down)[5],
  std::string &serial_csl_port, std::string &serial_odometry_port);

#endif // READ_CONFIG_H_