//
// Created by Anshuman Funkwal on 3/13/23.
//

#pragma once

#include "../Algorithm/MyAlgorithm.h"
#include "../Common/AbstractAlgorithm.h"
#include "BatteryMeterImpl.h"
#include "DirtSensorImpl.h"
#include "RobotState.h"
#include "Utils.h"
#include "WallsSensorImpl.h"

#include <fstream>
#include <iostream>

class Simulator {
private:
  std::size_t max_steps_;
  House houseState_;
  RobotState robotState_;

  AbstractAlgorithm *algo;
  // sensors
  DirtSensorImpl dirt_sensor_;
  WallsSensorImpl wall_sensor_;
  BatteryMeterImpl battery_meter_;

  int initSensors();

public:
  Simulator();
  int readHouseFile(const std::string &house_file_path);
  void setAlgorithm(AbstractAlgorithm &algorithm);
  void run();
};
