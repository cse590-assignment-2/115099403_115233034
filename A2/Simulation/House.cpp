#include "House.h"

void House::init(std::vector<std::vector<int>> &data) {

  data_.resize(data.size() + 2);
  for (int i = 0; i < data_.size(); i++)
    data_[i].resize(data[0].size() + 2, 0);

  // Pad with wall
  for (int i = 0; i < data_[0].size(); i++) {
    data_[0][i] = -1;
    data_[data_.size() - 1][i] = -1;
  }

  for (int i = 0; i < data_.size(); i++) {
    data_[i][0] = -1;
    data_[i][data_[0].size() - 1] = -1;
  }

  for (int i = 0; i < data.size(); i++) {
    for (int j = 0; j < data[0].size(); j++) {
      if (data[i][j] == 100) {
        dock_pos_ = {i + 1, j + 1};
        data_[i + 1][j + 1] = 0;
      } else {
        data_[i + 1][j + 1] = data[i][j];
      }
    }
  }
}

int House::dirtLevel(const Position &position) const {
  return data_[position.r][position.c];
}

bool House::isWall(const Position &position) const {
  return data_[position.r][position.c] == -1;
}

std::ostream &operator<<(std::ostream &out, const House &house) {
  out << "Printing House object: " << std::endl;
  for (int row = 0; row < house.data_.size(); row++) {
    for (int col = 0; col < house.data_[0].size(); col++) {
      if (row == house.dock_pos_.r && col == house.dock_pos_.c)
        out << "D";
      else
        out << ((house.data_[row][col] == -1)
                    ? "W"
                    : std::to_string(house.data_[row][col]));
      out << " ";
    }
    out << std::endl;
  }
  out << std::endl;
  return out;
}
