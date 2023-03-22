#include "MyAlgorithm.h"

#include "Utils.h"

#include <queue>
#include <vector>

MyAlgorithm::MyAlgorithm()
    : current_position_(dockPos), state_(AlgoState::CHARGING) {
  percieved_house_[current_position_] = int(LocType::Dock);
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

void MyAlgorithm::updateNeighbor(Direction dir) {
  auto pos = getPosition(current_position_, dir);
  if (walls_sensor_->isWall(dir)) {
    percieved_house_[pos] = int(LocType::Wall);
  } else {
    if (percieved_house_.count(pos) == 0)
      unexplored_points_[pos];
  }
}

void MyAlgorithm::updateNeighbors() {
  if (unexplored_points_.count(current_position_) != 0)
    unexplored_points_.erase(current_position_);

  updateNeighbor(Direction::North);
  updateNeighbor(Direction::South);
  updateNeighbor(Direction::East);
  updateNeighbor(Direction::West);
}

bool MyAlgorithm::needCharge() { return false; }

void MyAlgorithm::cleanCurrent() {
  if (percieved_house_.count(current_position_) != 0 &&
      percieved_house_[current_position_] > 0 &&
      percieved_house_[current_position_] <= MAX_DIRT) {
    // pos already exists
    percieved_house_[current_position_]--;
  } else {
    // visiting new pos
    percieved_house_[current_position_] = dirt_sensor_->dirtLevel();
  }
}

Step MyAlgorithm::work() {
  // priority to cleaning
  if (percieved_house_[current_position_] > 0)
    return Step::Stay;

  Direction dir;
  int max_dirt = -1;
  // it is guaranteed to be in perceived_house_ or unexplored_
  // due to updateNeighbors()
  for (auto d : dirPriority()) {
    auto point = getPosition(current_position_, d);
    if (unexplored_points_.count(point) != 0) {
      return static_cast<Step>(d);
    } else if (percieved_house_[point] > 0) {
      if (max_dirt < percieved_house_[point]) {
        dir = d;
        max_dirt = percieved_house_[point];
      }
    }
  }
  if (max_dirt > 0) {
    return static_cast<Step>(dir);
  }
  // BFS ALGORITHM
  state_ = AlgoState::TO_POS;
  // populate stack
  stack_ = getShortestPath(current_position_, {}, true);
  dir = stack_.top();
  stack_.pop();
  return static_cast<Step>(dir);
}

/**
 * @todo
 * 1. handle total dirt
 * 2.
 */
Step MyAlgorithm::nextStep() {
  // @todo : write finish condition
  // if (total_dirt == 0 && unexplored_points_.size() == 0 &&
  //     current_position_ == dockPos)
  //   state_ == AlgoState::FINISH;

  if (state_ == AlgoState::FINISH) {
    return Step::Finish;
  }

  percieved_house_[current_position_] = dirt_sensor_->dirtLevel();

  updateNeighbors();
  cleanCurrent();

  if (state_ == AlgoState::CHARGING) {
    if (battery_meter_->getBatteryState() != max_battery_) {
      return Step::Stay;
    }
    state_ = AlgoState::WORKING;
  }

  /**
   * @todo use TO_POS
   */
  if (state_ == AlgoState::TO_DOCK || state_ == AlgoState::TO_POS) {
    auto dir = stack_.top();
    stack_.pop();
    if (stack_.empty())
      state_ = AlgoState::CHARGING;
    // @todo check for correctness
    current_position_ = getPosition(current_position_, dir);
    return static_cast<Step>(dir);
  } else {
    if (needCharge()) {
      state_ = AlgoState::TO_DOCK;
      // populate stack
      stack_ = getShortestPath(current_position_, dockPos);
      auto dir = stack_.top();
      stack_.pop();
      return static_cast<Step>(dir);
    }
    return work();
  }
}

/**
 * @brief Returns stack of directions to take to reach given destination if
 * search is false and if search is true returns path to closest dirt or
 * unexplored location
 *
 * @param src Starting location
 * @param dst
 * @param search if true returns closest dirty/unexplored
 * @return std::stack<Direction>
 */
std::stack<Direction> MyAlgorithm::getShortestPath(std::pair<int, int> src,
                                                   std::pair<int, int> dst,
                                                   bool search) {
  std::stack<Direction> path;

  std::queue<Pos> st; // stack for DFS traversal
  std::map<Pos, bool> visited;
  std::map<Pos, Pos> parent;

  st.push(src);
  visited[src] = true;

  bool found = false;

  /**
   * @todo
   * - multiple paths till some battery level
   * - Priority queue of paths (with total dirt, distance)
   * - Store all paths and choose best based on difference battery left and
   *   min path coupled with dirt cleaned on way back
   *
   *  ** for A2 we can implement best path on given depth
   */
  while (!st.empty()) {
    auto t = st.front();
    st.pop();
    for (std::pair<int, int> v : neighbors(t)) {
      if (!visited[v]) {
        st.push(v);
        visited[v] = true;
        parent[v] = t;
      }
    }
    if (search &&
        !((percieved_house_.count(t) != 0 && percieved_house_[t] > 0) ||
          unexplored_points_.count(t) != 0)) { // found dirt
      continue;
    }
    if (!search && (!path.empty() || t != dst))
      continue; // if path is already found or not target node
    auto v = t;
    while (v != src) {
      // path.push(v);
      path.push(getDirection(parent[v], v));
      v = parent[v];
    }
  }
  return path;
}

std::vector<std::pair<int, int>>
MyAlgorithm::neighbors(std::pair<int, int> point) {
  static std::vector<std::pair<int, int>> directions = {
      {1, 0}, {0, 1}, {-1, 0}, {0, 1}};
  std::vector<std::pair<int, int>> neighbors;

  for (auto dir : directions) {
    std::pair<int, int> temp = {point.first + dir.first,
                                point.second + dir.second};
    // do not add walls
    // do not add unvisited nodes
    if ((percieved_house_.count(temp) != 0 &&
         percieved_house_[temp] != int(LocType::Wall) &&
         percieved_house_[temp] != int(LocType::Dock)) ||
        unexplored_points_.count(temp) != 0) {
      neighbors.push_back(temp);
    }
  }
  return neighbors;
}
