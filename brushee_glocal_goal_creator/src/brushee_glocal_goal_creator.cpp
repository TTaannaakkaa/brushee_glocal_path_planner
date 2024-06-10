#include "brushee_glocal_goal_creator/brushee_glocal_goal_creator.h"

BrusheeGlocalGoalCreator::BrusheeGlocalGoalCreator() : private_nh_("~") {
  private_nh_.param("hz", HZ_, {10});
  private_nh_.param("index_step", INDEX_STEP_, {1});
  private_nh_.param("goal_distance_weight", goal_distance_weight_, {0.3});
  private_nh_.param("pre_weight", pre_weight_, {10});
  private_nh_.param("tmp_weight", tmp_weight_, {1});
  is_path_ = false;
  goal_index_ = 0;

  goal_.header.frame_id = "base_link";
  tmp_goal_.header.frame_id = "base_link";

  sub_path_ = nh_.subscribe("/glocal_path", 1, &BrusheeGlocalGoalCreator::pathCallback, this);
  pub_goal_ = nh_.advertise<geometry_msgs::PoseStamped>("/glocal_goal", 1);
  pub_tmp_goal_ = nh_.advertise<geometry_msgs::PoseStamped>("/tmp_goal", 1);
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
  double goal_distance = goal_distance_weight_ * getGoalDistance();
  if(distance < goal_distance) {
      goal_index_ += INDEX_STEP_;
      if(goal_index_ >= path_.poses.size()) goal_index_ = path_.poses.size() - 1;
  }
  tmp_goal_.pose = path_.poses[goal_index_].pose;
  goal_.header.stamp = ros::Time::now();
  tmp_goal_.header.stamp = ros::Time::now();
 
  getAngle();
  setInertia(); 

  pub_goal_.publish(goal_);
  pub_tmp_goal_.publish(tmp_goal_);
  pre_goal_ = goal_;
}

void BrusheeGlocalGoalCreator::setInertia() {
  double pre_goal_yaw = tf2::getYaw(pre_goal_.pose.orientation);
  double tmp_goal_yaw = tf2::getYaw(tmp_goal_.pose.orientation);
  double goal_yaw = (pre_weight_ * pre_goal_yaw + tmp_weight_ * tmp_goal_yaw) / (pre_weight_ + tmp_weight_);

  goal_.pose.orientation = tf::createQuaternionMsgFromYaw(goal_yaw);
  goal_.pose.position.x = (pre_weight_ * pre_goal_.pose.position.x + tmp_weight_ * tmp_goal_.pose.position.x) / (pre_weight_ + tmp_weight_);
  goal_.pose.position.y = (pre_weight_ * pre_goal_.pose.position.y + tmp_weight_ * tmp_goal_.pose.position.y) / (pre_weight_ + tmp_weight_);
}

double BrusheeGlocalGoalCreator::getDistance() {
  double dx = path_.poses[goal_index_].pose.position.x;
  double dy = path_.poses[goal_index_].pose.position.y;
  return hypot(dx, dy);
}

void BrusheeGlocalGoalCreator::getAngle() {
  double dx = tmp_goal_.pose.position.x - path_.poses[path_.poses.size() - 1].pose.position.x;
  double dy = tmp_goal_.pose.position.y - path_.poses[path_.poses.size() - 1].pose.position.y;
  double angle = atan2(dy, dx) + M_PI;
  tmp_goal_.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
}

double BrusheeGlocalGoalCreator::getGoalDistance() {
  double dx = path_.poses[path_.poses.size() - 1].pose.position.x;
  double dy = path_.poses[path_.poses.size() - 1].pose.position.y;
  return hypot(dx, dy);
}