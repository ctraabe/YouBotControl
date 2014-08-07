#include "read_csl_config.h"

#include <iostream>

#include "youbot/YouBotBase.hpp"

int ReadConfigArm(bool &use, float (&arm_up)[5], float (&arm_down)[5])
{
  youbot::ConfigFile config_file("csl", CONFIG_DIR);
  if ( config_file.sectionExists("ARM") )
  {
    config_file.readInto(use, "ARM", "USE");
    if (!use) return(0);

    config_file.readInto(arm_up[0], "ARM", "A1_UP");
    config_file.readInto(arm_up[1], "ARM", "A2_UP");
    config_file.readInto(arm_up[2], "ARM", "A3_UP");
    config_file.readInto(arm_up[3], "ARM", "A4_UP");
    config_file.readInto(arm_up[4], "ARM", "A5_UP");

    config_file.readInto(arm_down[0], "ARM", "A1_DOWN");
    config_file.readInto(arm_down[1], "ARM", "A2_DOWN");
    config_file.readInto(arm_down[2], "ARM", "A3_DOWN");
    config_file.readInto(arm_down[3], "ARM", "A4_DOWN");
    config_file.readInto(arm_down[4], "ARM", "A5_DOWN");
    return 0;
  }
  else
  {
    std::cout << "No \"ARM\" section found in the config-file" << std::endl;
    return 1;
  }
}

int ReadConfigSerial(const std::string section, bool &use, std::string &port,
  int &baudrate)
{
  youbot::ConfigFile config_file("csl", CONFIG_DIR);
  if ( config_file.sectionExists(section) )
  {
    config_file.readInto(use, section, "PORT");
    if (!use) return 0;
    config_file.readInto(port, section, "PORT");
    config_file.readInto(baudrate, section, "BAUDRATE");
    return 0;
  }
  else
  {
    std::cout << "No " << section << " section found in the config-file"
      << std::endl;
    return 1;
  }
}

int ReadConfigCSLSerial(bool &use, std::string &port, int &baudrate)
{
  return ReadConfigSerial("CSL_SERIAL", use, port, baudrate);
}

int ReadConfigOdometrySerial(bool &use, std::string &port, int &baudrate)
{
  return ReadConfigSerial("ODOMETRY_SERIAL", use, port, baudrate);
}
