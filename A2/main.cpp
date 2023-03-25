#include "Algorithm/MyAlgorithm.h"
#include "Simulation/Simulator.h"
#include <iostream>

// getting command line arguments for the house file
int main(int argc, char **argv) {
  Simulator simulator;
  //  TODO: Handle empty command line args etc.
  if (simulator.readHouseFile(argv[1]) < 0)
    std::cout << "File read error. Stopping Simulator" << std::endl;
  MyAlgorithm algo;
  simulator.setAlgorithm(algo);
  simulator.run();
  simulator.dump("output.txt");
}
