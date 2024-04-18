/**
 * @brief main function 
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */

#include "brushee_local_map_creator/brushee_local_map_creator.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "local_map_creator");
    BrusheeLocalMapCreator localmap_creator;
    localmap_creator.process();
    
    return 0;
}