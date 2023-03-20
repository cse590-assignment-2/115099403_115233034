#include "BatteryMeterImpl.h"

BatteryMeterImpl::BatteryMeterImpl(std::shared_ptr<RobotState> robot)
    : robot_(robot) {}

BatteryMeterImpl::BatteryMeterImpl(const RobotState &robotState)
    : robotState_(&robotState) {}

std::size_t BatteryMeterImpl::getBatteryState() const {
  return robot_->battery();
}
