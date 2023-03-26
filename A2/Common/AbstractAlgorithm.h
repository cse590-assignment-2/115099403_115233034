// DO NOT CHANGE THIS FILE
// Created by Anshuman Funkwal on 3/13/23.
//

#pragma once

#include "BatteryMeter.h"
#include "DirtSensor.h"
#include "WallsSensor.h"
#include "common_types.h"
#include <cstdlib>
#include <iostream>
#include <memory>

class AbstractAlgorithm {

public:
  virtual ~AbstractAlgorithm() {}
  virtual void setMaxSteps(std::size_t maxSteps) = 0;
  virtual void setWallsSensor(const WallsSensor &) = 0;
  virtual void setDirtSensor(const DirtSensor &) = 0;
  virtual void setBatteryMeter(const BatteryMeter &) = 0;
  virtual Step nextStep() = 0;
};
