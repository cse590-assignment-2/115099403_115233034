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

class Simulator { // : public WallsSensor, public DirtSensor, public
                  // BatteryMeter {
private:
  // cannot directly instanciate abstract algorithm, so using smart pointers
  // std::unique_ptr<MyAlgorithm> algo;
  AbstractAlgorithm *algo;

  std::unique_ptr<DirtSensorImpl> dirt_sensor;
  std::unique_ptr<WallsSensorImpl> walls_sensor;
  std::unique_ptr<BatteryMeterImpl> battery_meter;

  DirtSensorImpl dirt_sensor_;
  WallsSensorImpl wall_sensor_;
  BatteryMeterImpl battery_meter_;

  std::size_t max_steps_;
  House houseState_;
  std::shared_ptr<House> house_;
  RobotState robotState_;
  std::shared_ptr<RobotState> robot_;
  int initSensors();

public:
  Simulator();
  int readHouseFile(const std::string &house_file_path);
  void setAlgorithm(AbstractAlgorithm &algorithm);
  void run();

  // bool isWall(Direction d) const override { return false; }
  // int dirtLevel() const override { return 0; }
  // std::size_t getBatteryState() const override { return 0; }
};
