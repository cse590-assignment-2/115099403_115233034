#include "Simulator.h"

Simulator::Simulator() {
  house_ = std::make_shared<House>();
  robot_ = std::make_shared<RobotState>();

  dirt_sensor = std::make_unique<DirtSensorImpl>(house_);
  walls_sensor = std::make_unique<WallsSensorImpl>(house_);
  battery_meter = std::make_unique<BatteryMeterImpl>(robot_);
}

void Simulator::setAlgorithm(AbstractAlgorithm &algorithm) {

  // if (walls_sensor || dirt_sensor || battery_meter) {
  //   std::cout << "ERROR!! SETTING ALGORITHM BEFORE SENSORS INITIALIZATION"
  //             << std::endl;
  // }
  algo = &algorithm;
  algo->setMaxSteps(max_steps_);

  DirtSensor *ds = (DirtSensor *)dirt_sensor.get();
  algo->setDirtSensor(*ds);
  WallsSensor *ws = (WallsSensor *)walls_sensor.get();
  algo->setWallsSensor(*ws);
  BatteryMeter *bm = (BatteryMeter *)battery_meter.get();
  algo->setBatteryMeter(*bm);
}

int Simulator::readHouseFile(const std::string &houseFilePath) {
  // populate house_ structure here

  std::string house_name, max_steps_s, max_battery_s, num_rows_s, num_cols_s;
  std::ifstream myfile;
  myfile.open(houseFilePath);

  if (!myfile.is_open()) {
    return -1;
  }

  std::getline(myfile, house_name);
  std::getline(myfile, max_steps_s);
  std::getline(myfile, max_battery_s);
  std::getline(myfile, num_rows_s);
  std::getline(myfile, num_cols_s);

  if (myfile.eof()) {
    return -1;
  }

  std::cout << max_steps_s << std::endl
            << max_battery_s << std::endl
            << num_rows_s << std::endl
            << num_cols_s << std::endl;

  max_steps_ = Utils::parseDouble(max_steps_s);
  int max_robot_battery_ = Utils::parseDouble(max_battery_s);
  int n_rows_ = Utils::parseDouble(num_rows_s);
  int n_cols_ = Utils::parseDouble(num_cols_s);

  std::cout << max_steps_ << std::endl
            << max_robot_battery_ << std::endl
            << n_rows_ << std::endl
            << n_cols_ << std::endl;

  std::vector<std::vector<int>> data(n_rows_, std::vector<int>(n_cols_, 0));

  int row_number = 0, col_number = 0, dock_found = 0;
  std::string line;
  std::getline(myfile, line);
  while (!myfile.eof()) {
    for (col_number = 0; col_number < line.size(); col_number++) {
      if (col_number >= n_cols_)
        break;
      if (std::isdigit(line[col_number])) {
        data[row_number][col_number] = line[col_number] - '0';
      } else if (line[col_number] == ' ') {
        data[row_number][col_number] = 0;
      } else if (line[col_number] == 'W') {
        data[row_number][col_number] = -1; // replace with LocType
      } else if (line[col_number] == 'D' && !dock_found) {
        dock_found = 1;
        data[row_number][col_number] = 100; // replace with LocType
      } else if (line[col_number] == 'D') {
        return -1;
      } else {
        return -1;
      }
    }
    std::getline(myfile, line);
    row_number++;
  }

  house_->init(data);
  robot_->init(max_robot_battery_, house_->getDockPos());
  std::cout << "Robot: max_robot_battery:" << max_robot_battery_ << std::endl;
  std::cout << "House:\n";
  std::cout << *house_;

  return 1;
}

void Simulator::run() {
  // TODO : Implement run() using the following function
  algo->nextStep();
}
