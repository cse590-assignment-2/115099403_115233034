## Sukesh Cheripalli(115099403) & Puneet Udupi(115233034) - Graduate team

### Setup and Run:

##### Setup

1. Install CMake
##### Run

```shell
# How to build
cd A2/
mkdir build
cp docs/inputs/sample-input1.txt build/
cd build/
cmake ..
make 
# How to run
./A2 sample-intput1.txt

# Folder structure is very straightforward
# docs contain input and output and High-Level-Design PDF
# A2/Simulator contains simulator code
# A2/Algorithm contains algorithm code
# main in the A2/ folder
```

#### Input format

Line 1: house name / description - internal for the file, can be ignored by the simulation
Line 2: MaxSteps for the simulation on this house, as: MaxSteps = <NUM>
Line 3: MaxBattery (in steps) for the simulation on this house, as: MaxBattery = <NUM>
Line 4: Number of Rows in house, as: Rows = <NUM>
Line 5: Number of Cols in house, as: Cols = <NUM>
Lines 6 and on: the house itself

> In case the file contains additional rows and/or cols beyond the required number as set in lines 4 and 5, the program shall ignore the redundant lines / cols.
> Numbers from 0 to 9 should represent dirt level. Though 0 is valid and shall be supported, it is in fact the same as space.
> Docking Station: D. There shall be one and only one docking station otherwise the file is invalid.


#### Solution Approach
The solution keeps track of unexplored list, total found dirt and uses to return FINISH whenever the conditions are met.

neighbours data are updated by querying the sensors at every step.

the algorithm takes decisions based on the current state among : CHARGING, TO_DOCK, TO_POS, WORKING. 

Normally, the doc will be in state WORKING, where it will keep updating the perceived map of the house and gives priority to cleaning the current tile till it becomes 0. Once it cleans the current tile, it explores points that have not been explored yet. 
If there are unexplored tiles in multiple directions, it uses a determinisitic priority list of directions. 
If all neighbors are explored, it picks the direction of tile which has the max dirt. 
If none of the above conditions are met, the algorithm populates a stack with directions by performing BFS algorithm on the percieved house and changes the state to TO_POS. In TO_POS state, the algorithm just follows the stack of directions populated earlier by the bfs algorithm.

At every step, it checks if we will have enough charge to go back to dock for charging or if the condition for FINISH has reached. if yes, it will change to TO_DOCK state and march towards the dock, while cleaning each tile once and updating the house on the way as well. Once it reaches dock, it goes to CHARGING state until it reaches full charge and changes its state back to WORKING.




---
