#include "MyAlgorithm.h"

#include "Utils.h"

#include <queue>

MyAlgorithm::MyAlgorithm()
    : house_manager_(), current_position_(DOCK_POS),
      state_(AlgoState::CHARGING) {
  house_manager_.setDirt(current_position_, int(LocType::Dock));
}

MyAlgorithm::MyAlgorithm(AbstractAlgorithm &algorithm) { *this = algorithm; }

void MyAlgorithm::setMaxSteps(std::size_t max_steps) { max_steps_ = max_steps; }

void MyAlgorithm::setWallsSensor(const WallsSensor &walls_sensor) {
  walls_sensor_ = &walls_sensor;
}

void MyAlgorithm::setDirtSensor(const DirtSensor &dirt_sensor) {
  dirt_sensor_ = &dirt_sensor;
}

void MyAlgorithm::setBatteryMeter(const BatteryMeter &battery_meter) {
  battery_meter_ = &battery_meter;
  max_battery_ = battery_meter_->getBatteryState();
}

void MyAlgorithm::updateNeighbors() {
  house_manager_.eraseUnexplored(current_position_);

  for (auto dir : dirPriority()) {
    house_manager_.updateNeighbor(dir, current_position_,
                                  walls_sensor_->isWall(dir));
  }
}

bool MyAlgorithm::needCharge() {
  auto st = house_manager_.getShortestPath(current_position_, DOCK_POS);
  if (st.size() + BATTERY_BUFF < battery_meter_->getBatteryState())
    return true;
  return false;
}

Step MyAlgorithm::work() {
  // Assuming current_pos exists in percieved
  // priority to cleaning
  if (house_manager_.dirt(current_position_) > 0)
    return Step::Stay;

  Direction dir;
  int max_dirt = -1;
  // it is guaranteed to be in perceived_house_ or unexplored_
  // due to updateNeighbors()
  for (auto d : dirPriority()) {
    auto point = getPosition(current_position_, d);
    if (house_manager_.checkUnexplored(point)) {
      return static_cast<Step>(d);
    } else if (house_manager_.exists(point) && house_manager_.dirt(point) > 0) {
      if (max_dirt < house_manager_.dirt(point)) {
        dir = d;
        max_dirt = house_manager_.dirt(point);
      }
    } else if (!house_manager_.exists(point)) {
      std::cout << "ERROR!! INVALID SCENARIO - NOT IN UNEXPLORED OR PERCIEVED"
                << std::endl;
    }
  }
  if (max_dirt > 0) {
    return static_cast<Step>(dir);
  }
  // BFS ALGORITHM
  state_ = AlgoState::TO_POS;
  // populate stack
  stack_ = house_manager_.getShortestPath(current_position_, {}, true);
  dir = stack_.top();
  stack_.pop();
  return static_cast<Step>(dir);
}

/**
 * @todo
 * 1. handle total dirt
 */
Step MyAlgorithm::nextStep() {
  if (house_manager_.isUnexploredEmpty() && house_manager_.total_dirt() == 0 &&
      current_position_ == DOCK_POS)
    state_ = AlgoState::FINISH;

  if (state_ == AlgoState::FINISH) {
    return Step::Finish;
  }

  house_manager_.setDirt(current_position_, dirt_sensor_->dirtLevel());

  updateNeighbors();
  house_manager_.clean(current_position_, dirt_sensor_->dirtLevel());

  if (state_ == AlgoState::CHARGING) {
    if (battery_meter_->getBatteryState() != max_battery_) {
      return Step::Stay;
    }
    state_ = AlgoState::WORKING;
  }

  if (state_ == AlgoState::TO_DOCK || state_ == AlgoState::TO_POS) {
    auto dir = stack_.top();
    stack_.pop();
    // @todo check for correctness
    // if position is valid i.e unexplored or perceived
    // if position is
    current_position_ = getPosition(current_position_, dir);
    if (stack_.empty())
      state_ = DOCK_POS == current_position_ ? AlgoState::CHARGING
                                             : AlgoState::WORKING;
    return static_cast<Step>(dir);
  } else {
    if (needCharge() || house_manager_.total_dirt() == 0) {
      state_ = AlgoState::TO_DOCK;
      // populate stack
      stack_ = house_manager_.getShortestPath(current_position_, DOCK_POS);
      auto dir = stack_.top();
      stack_.pop();
      current_position_ = getPosition(current_position_, dir);
      if (stack_.empty())
        state_ = AlgoState::CHARGING;
      return static_cast<Step>(dir);
    }
    return work();
  }
}
