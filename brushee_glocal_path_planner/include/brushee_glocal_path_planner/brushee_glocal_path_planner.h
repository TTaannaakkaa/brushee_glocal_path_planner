#ifndef BRUSHEE_GLOCAL_PATH_PLANNER_H
#define BRUSHEE_GLOCAL_PATH_PLANNER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h> 
#include <geometry_msgs/PoseStamped.h>

class BrusheeGlocalPathPlanner {
  public:
    BrusheeGlocalPathPlanner();
    void process();
  private:
    