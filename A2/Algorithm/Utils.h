#pragma once

#include "types.h"

#include <cstdlib>
#include <utility>

#define Pos std::pair<int, int>

constexpr Pos dockPos = {0, 0};

/**
 * @brief get std::pair corresponding to step direction
 *
 * @param dir
 * @param invert
 * @return Pos
 */
inline Pos toPair(Direction dir, bool invert = false) {
  int multiplier = invert ? -1 : 1;
  switch (dir) {
  case Direction::North:
    return {0, 1 * multiplier};
  case Direction::South:
    return {0, -1 * multiplier};
  case Direction::East:
    return {1, 0 * multiplier};
  case Direction::West:
    return {0, 1 * multiplier};
  default:
    return {0, 0};
  }
}

/**
 * @brief get direction std::pair corresponding to direction
 *
 * @param dir
 * @param invert
 * @return Pos
 */
inline Direction toDir(int x, int y) {
  /**
   * @todo handle errors for x, y
   */
  if (x == 1)
    return Direction::East;
  if (x == -1)
    return Direction::West;
  if (y == 1)
    return Direction::North;
  if (y == -1)
    return Direction::South;
  // default error value
  return Direction::North;
}

/**
 * @brief Get the Position in the given direction
 *
 * @param pos current position
 * @param dir direction
 * @return Pos
 */
Pos getPosition(Pos pos, Direction dir) {
  auto direction = toPair(dir);
  return {pos.first + direction.first, pos.second + direction.second};
}

/**
 * @brief Prioirty of direction traversal
 *
 * @param dir
 * @return Direction
 */
Direction nextDir(const Direction &dir) {
  /**
   * @note
   * UP
   * DOWN
   * EAST
   * WEST
   */
  if (dir == Direction::North)
    return Direction::South;
  else if (dir == Direction::South)
    return Direction::East;
  else if (dir == Direction::East)
    return Direction::West;
  else // (dir == Direction::West)
    return Direction::North;
}

/**
 * @brief Prioirty of direction traversal
 */
std::vector<Direction> dirPriority() {
  return {Direction::North, Direction::South, Direction::East, Direction::West};
}

Direction getDirection(Pos src, Pos dst) {
  Pos diff = {dst.first - src.first, dst.second - src.second};
  if ((std::abs(diff.first) + std::abs(diff.second)) != 1) {
    std::cout << __FUNCTION__ << "ERROR!! invalid parameters in getDirection"
              << std::endl;
  }
  return toDir(diff.first, diff.second);
}
