#include "brushee_glocal_goal_creator/brushee_glocal_goal_creator.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "bruhsee_glocal_goal_creator");
  BrusheeGlocalGoalCreator brusheeglocalgoalcreator;
  brusheeglocalgoalcreator.process();
  return 0;
}