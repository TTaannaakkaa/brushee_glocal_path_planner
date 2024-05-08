#include "brushee_glocal_goal_creator/brushee_glocal_goal_creator.h"

BrusheeGlocalGoalCreator::BrusheeGlocalGoalCreator() : private_nh_("~") {
  private_nh_.param("hz", HZ_, 10);
  private_nh_.param("index_step", INDEX_STEP_, 1);
  private_nh_.param("goal_distance", goal_distance_, 1.0);
  is_path_ = false;
  goal_index_ = 0;

  goal_.header.frame_id = "base_link";

  sub_path_ = nh_.subscribe("/glocal_path", 1, &BrusheeGlocalGoalCreator::pathCallback, this);
  pub_goal_ = nh_.advertise<geometry_msgs::PointStamped>("/glocal_goal", 1);
}

void BrusheeGlocalGoalCreator::pathCallback(const nav_msgs::Path::ConstPtr& msg) {
  path_ = *msg;
  is_path_ = true;
}

void BrusheeGlocalGoalCreator::process() {
  ros::Rate LoopRate(HZ_);

  while(ros::ok()) {
    if(is_path_) publishGoal();
    ros::spinOnce();
    LoopRate.sleep();
  }
}

void BrusheeGlocalGoalCreator::publishGoal() {
  double distance = getDistance();
  if(distance < goal_distance_) {
      goal_index_ += INDEX_STEP_;
      if(goal_index_ >= path_.poses.size()) goal_index_ = path_.poses.size() - 1;
  }
  goal_.header.stamp = ros::Time::now();
  goal_.point = path_.poses[goal_index_].pose.position;
  pub_goal_.publish(goal_);
}

double BrusheeGlocalGoalCreator::getDistance() {
  double dx = path_.poses[goal_index_].pose.position.x;
  double dy = path_.poses[goal_index_].pose.position.y;
  return hypot(dx, dy);
}