#include "brushee_glocal_path_planner/brushee_glocal_path_planner.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "brushee_glocal_path_planner");
  BrusheeGlocalPathPlanner brushee_glocal_path_planner;
  brushee_glocal_path_planner.process();
  return 0;
}