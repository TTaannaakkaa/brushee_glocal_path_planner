#include "brushee_obstacle_detector/brushee_obstacle_detector.h"

/**
 * @brief main function 
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_detector");
    BrusheeObstacleDetector obstacle_detector;
    obstacle_detector.process();
    
    return 0;
}