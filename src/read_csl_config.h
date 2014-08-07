#ifndef READ_CONFIG_H_
#define READ_CONFIG_H_

#include <string>

int ReadConfigArm(bool &use, float (&arm_up)[5], float (&arm_down)[5]);

int ReadConfigCSLSerial(bool &use, std::string &port, int &baudrate);

int ReadConfigOdometrySerial(bool &use, std::string &port, int &baudrate);

#endif // READ_CONFIG_H_