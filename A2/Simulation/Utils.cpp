#include "Utils.h"
#include "ErrorCodes.h"

using std::string;

double Utils::parseInt(string str) {
  std::string input = str.substr(str.find('=') + 1);
  if (input.empty())
    return (size_t)FileReadError::Invalid;

  // ltrim
  input.erase(input.begin(),
              std::find_if(input.begin(), input.end(),
                           [](unsigned char ch) { return !std::isspace(ch); }));
  // rtrim
  input.erase(std::find_if(input.rbegin(), input.rend(),
                           [](unsigned char ch) { return !std::isspace(ch); })
                  .base(),
              input.end());

  try {
    return std::stoi(input);
  } catch (...) {
    return (size_t)FileReadError::InvalidValue;
  }
}

size_t Utils::readAEqb(string input, string varname) {
  string valString = input.substr(input.find('=') + 1);

  // TODO:
  // 1. check varname FileReadError::InvalidName;
  // 2. check double  FileReadError::InvalidValue;
  // 3. other scenarios

  // handle val < 0 at caller
  return parseInt(valString);
}
std::ostream &operator<<(std::ostream &out, const Position &pos) {
  out << "(" << pos.r << "," << pos.c << ")";
  return out;
}
Direction reverse(Direction d) {
  switch (d) {
  case Direction::North:
    return Direction::South;
  case Direction::South:
    return Direction::North;
  case Direction::West:
    return Direction::East;
  case Direction::East:
    return Direction::West;
  default:
    return Direction::West;
  }
}

Step reverse(Step s) {
  switch (s) {
  case Step::North:
    return Step::South;
  case Step::South:
    return Step::North;
  case Step::West:
    return Step::East;
  case Step::East:
    return Step::West;
  default:
    return Step::Stay;
  }
}
std::ostream &operator<<(std::ostream &out, const Step &step) {
  out << str(step);
  return out;
}
std::string str(const Step &step) {
  switch (step) {
  case Step::North:
    return "North";
    break;
  case Step::South:
    return "South";
    break;
  case Step::East:
    return "East";
    break;
  case Step::West:
    return "West";
    break;
  case Step::Stay:
    return "stay";
    break;
  case Step::Finish:
    return "Finish";
    break;
  }
}
