#pragma once

#include "../Common/common_types.h"

#include <iostream>

enum class AlgoState {
  CHARGING,
  TO_DOCK,
  TO_POS,
  WORKING,
  FINISH,
  EXPLORE,
  CLEANING
};
