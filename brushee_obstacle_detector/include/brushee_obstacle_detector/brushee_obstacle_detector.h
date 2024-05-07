/**
 * @file obstacle_detector.h
 * @author Takuma Tanaka
 * @brief detection obstacles from laser scan
 * @version 0.1
 * @date 2023-08-30
 */

#ifndef BRUSHEE_OBSTACLE_DETECTOR_H
#define BRUSHEE_OBSTACLE_DETECTOR_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

/**
 * @brief detection obstacles from laser scan
 */
class BrusheeObstacleDetector {
public:
  /**
   * @brief Construct a new Obstacle Detector object
   */
  BrusheeObstacleDetector();
  /**
   * @brief main process
   */
  void process();

private:
  /**
   * @brief callback function for laser scan
   * @param msg The laser scan message
   */
  void laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg);

  /**
   * @brief scan obstacles from laser scan
   */
  void scan_obstacle();

  /**
   * @brief The frequency of the process
   */
  int hz_;

  /**
   * @brief The step of the laser scan
   */
  int laser_step_;

  /**
   * @brief The frame id of the rviz
   */
  std::string robot_frame_;

  /**
   * @brief The flag of the laser scan is received or not
   */
  bool flag_laser_scan_ = false;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Subscriber laser_scan_sub_;

  ros::Publisher obstacle_pose_pub_;

  geometry_msgs::PoseArray obstacle_pose_array_;
  sensor_msgs::LaserScan laser_scan_;
};

#endif // OBSTACLE_DETECTOR_H