#ifndef BRUSHEE_GLOCAL_GOAL_CREATOR_H
#define BRUSHEE_GLOCAL_GOAL_CREATOR_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf2/utils.h>
#include <tf/transform_broadcaster.h>

class BrusheeGlocalGoalCreator {
  public:
    BrusheeGlocalGoalCreator();
    void process();

  private:
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    void publishGoal();
    void setInertia();
    void getAngle();
    double getDistance();
    double getGoalDistance();

    int HZ_;
    int INDEX_STEP_;
    int goal_index_;
    int pre_weight_;
    int tmp_weight_;
    double goal_distance_weight_;
    double goal_yaw_;
    bool is_path_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber sub_path_;
    ros::Publisher pub_goal_;
    ros::Publisher pub_tmp_goal_;

    nav_msgs::Path path_;
    geometry_msgs::PoseStamped pre_goal_;
    geometry_msgs::PoseStamped tmp_goal_;
    geometry_msgs::PoseStamped goal_;
};

#endif // BRUSHEE_GLOCAL_GOAL_CREATOR_H