#include "Utils.h"
#include "ErrorCodes.h"

using std::string;

double Utils::parseDouble(string str) {
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
    return std::stod(input);
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
  return parseDouble(valString);
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
