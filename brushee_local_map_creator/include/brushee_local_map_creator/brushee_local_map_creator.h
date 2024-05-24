/**
 * @file local_map_creator.h
 * @author Takuma Tanaka
 * @brief header file for local_map_creator class
 * @version 0.1
 * @date 2023-09-12
 * 
 */

#ifndef BRUSHEELOCAL_MAP_CREATOR_H
#define BRUHSEELOCAL_MAP_CREATOR_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

/**
 * @brief Class for local map creator
 */
class BrusheeLocalMapCreator
{
public:
    /**
     * @brief Constructor for the LocalMapCreator
     */
    BrusheeLocalMapCreator();

    /**
     * @brief Caluculate local map
     */
    void process();

private:
    /**
     * @brief A callback to hadldle obstacle poses message
     */
    void obs_poses_callback(const geometry_msgs::PoseArrayConstPtr& msg);

    /**
     * @brief A callback to handle costmap message
     */
    void costmap_callback(const nav_msgs::OccupancyGridConstPtr& msg);

    /**
     * @brief Check if the point is in the local map
     * @param x The point x coordinate
     * @param y The point y coordinate
     * @return true if the point is in the local map
     * @return false if the point is not in the local map
     */
    bool is_in_local_map(const double x, const double y);

    /**
     * @brief Get the grid index in the local map
     * @param dist The distance from the robot
     * @param angle The angle from the front of the robot
     * @return The grid index in the local map
     */
    int get_grid_index(const double dist, const double angle);

    /**
     * @brief Convert the point coordinate to the grid index
     * @param x The point x coordinate
     * @param y The point y coordinate
     * @return The grid index in the local map
     */
    int xy_to_index(const double x, const double y);

    /**
     * @brief Initialize the local map
     */
    void init_local_map();

    /**
     * @brief Update the local map
     */
    void update_local_map();

    int hz_;
    int expand_size_;
    double local_map_size_;
    double local_map_resolution_;
 
    bool is_expand_;
    bool is_costmap_;
    bool is_get_obs_poses_ = false;
    bool is_get_costmap_ = false;
    

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber sub_obs_poses_;
    ros::Subscriber sub_costmap_;

    ros::Publisher pub_local_map_;

    nav_msgs::OccupancyGrid local_map_raw_;
    nav_msgs::OccupancyGrid local_map_;
    nav_msgs::OccupancyGrid costmap_;
    geometry_msgs::PoseArray obs_poses_;

};

#endif // BRUSHEELOCAL_MAP_CREATOR_H