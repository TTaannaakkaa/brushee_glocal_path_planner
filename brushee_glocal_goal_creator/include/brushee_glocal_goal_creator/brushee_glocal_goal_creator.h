#ifndef BRUSHEE_GLOCAL_GOAL_CREATOR_H
#define BRUSHEE_GLOCAL_GOAL_CREATOR_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

class BrusheeGlocalGoalCreator {
  public:
    BrusheeGlocalGoalCreator();
    void process();

  private:
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    void publishGoal();
    double getDistance();

    int HZ_;
    int INDEX_STEP_;
    int goal_index_;
    double goal_distance_;
    bool is_path_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber sub_path_;
    ros::Publisher pub_goal_;

    nav_msgs::Path path_;
    geometry_msgs::PoseStamped goal_;
};

#endif // BRUSHEE_GLOCAL_GOAL_CREATOR_H