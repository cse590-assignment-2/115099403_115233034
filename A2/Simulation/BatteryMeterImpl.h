#pragma once

#include "../Common/BatteryMeter.h"
#include "../Common/common_types.h"
#include "House.h"
#include "RobotState.h"
#include <memory>

class BatteryMeterImpl : public BatteryMeter {
private:
  std::shared_ptr<RobotState> robot_;
  const RobotState *robotState_;

public:
  BatteryMeterImpl(std::shared_ptr<RobotState> robot);
  BatteryMeterImpl(const RobotState &robotState);
  void setRobot(const RobotState &robotState);
  std::size_t getBatteryState() const override;
};
