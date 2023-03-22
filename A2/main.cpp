#include "Algorithm/MyAlgorithm.h"
#include "Simulation/Simulator.h"
#include <iostream>

// getting command line arguments for the house file
int main(int argc, char **argv) {
  Simulator simulator;
  //  TODO: Handle empty command line args etc.
  simulator.readHouseFile(argv[1]);
  MyAlgorithm algo;
  simulator.setAlgorithm(algo);
  simulator.run();
}
