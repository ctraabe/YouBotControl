#include "read_csl_config.h"

#include <iostream>

#include "youbot/YouBotBase.hpp"

int ReadCSLConfig(float (&arm_up)[5], float (&arm_down)[5],
  std::string &serial_csl_port, std::string &serial_odometry_port)
{
  youbot::ConfigFile config_file("csl", CONFIG_DIR);
  // if(config_file)
  // {
  //   std::cout << "Could not load file: " << CONFIG_DIR << config_file
  //     << " Check if it exists and the path is correct" << std::endl;
  //   return -1;
  // }

  if ( config_file.sectionExists("ARM_UP") )
  {
    config_file.readInto(arm_up[0], "ARM_UP", "A1");
    config_file.readInto(arm_up[1], "ARM_UP", "A2");
    config_file.readInto(arm_up[2], "ARM_UP", "A3");
    config_file.readInto(arm_up[3], "ARM_UP", "A4");
    config_file.readInto(arm_up[4], "ARM_UP", "A5");
  }
  else
  {
    std::cout << "No \"ARM_UP\" section found in the config-file" << std::endl;
  }

  if ( config_file.sectionExists("ARM_DOWN") )
  {
    config_file.readInto(arm_down[0], "ARM_DOWN", "A1");
    config_file.readInto(arm_down[1], "ARM_DOWN", "A2");
    config_file.readInto(arm_down[2], "ARM_DOWN", "A3");
    config_file.readInto(arm_down[3], "ARM_DOWN", "A4");
    config_file.readInto(arm_down[4], "ARM_DOWN", "A5");
  }
  else
  {
    std::cout << "No \"ARM_DOWN\" section found in the config-file"
      << std::endl;
  }

  if ( config_file.sectionExists("SERIAL_PORTS") )
  {
    config_file.readInto(serial_csl_port, "SERIAL_PORTS", "VEHICLE_WRAPPER");
    config_file.readInto(serial_odometry_port, "SERIAL_PORTS", "ODOMETRY");
  }
  else
  {
    std::cout << "No \"SERIAL_PORTS\" section found in the config-file"
      << std::endl;
  }

  return 0;
}