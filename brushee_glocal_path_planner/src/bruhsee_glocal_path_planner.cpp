#include "brushee_glocal_path_planner/brushee_glocal_path_planner.h"

BrusheeGlocalPathPlanner::BrusheeGlocalPathPlanner():private_nh_("~") {
  private_nh_.getParam("HZ", HZ_);

  sub_map_ = nh_.subscribe("/local_map", 1, &BrusheeGlocalPathPlanner::mapCallback, this);
  sub_goal_ = nh_.subscribe("/local_goal", 1, &BrusheeGlocalPathPlanner::goalCallback, this);
  
  pub_path_ = nh_.advertise<nav_msgs::Path>("/glocal_path", 1);
  pub_goal_ = nh_.advertise<geometry_msgs::PoseStamped>("/glocal_goal", 1);
}

void BrusheeGlocalPathPlanner::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  map_ = *msg;
}

void BrusheeGlocalPathPlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  goal_ = *msg;
}

void BrusheeGlocalPathPlanner::path_planning() {
  OPEN_SET_.clear();
  CLOSED_SET_.clear();

  START_NODE_ = {0, 0, 0, 0, 0};
  GOAL_NODE_ = get_goal_node();

  START_NODE_.cost = calc_heuristic(START_NODE_);
  OPEN_SET_.push_back(START_NODE_);

  while(ros::ok()) {
    if(OPEN_SET_.empty()) {
      ROS_WARN_STREAM("No path found.");
      exit(1):
    }

    Node current_node = selsct_current_node();

    if(is_goal(current_node)) {
      create_path(current_node);
      break;
    } else {
      transfar_node(current_node, OPEN_NODE_, CLOSED_SET_);
      update_set(current_node);
    }
  }
  path_pub_.publish(path_);
  exit(0);
}

Node BruheeGlocalPathPlanner::get_goal_node() {
  Node goal_node;
  goal_node.index_x = int(round((goal_.pose.position.x - map_info.origin.position.x) / map_.info.resolution));
  goal_node.index_y = int(round((goal_.pose.position.y - map_info.origin.position.y) / map_.info.resolution));

  return goal_node;
}

bool BrusheeGlocalPathPlanner::is_obs(const Node node) {
  const int grid_index = node.index_x + (node.index_y * map_.info.width);
  return map_.data[grid_index] == 100;
}

double BrusheeGlocalPathPlanner::calc_heuristic(const Node node) {
  const double w = 1.0;

  const double dx = double(node.index_x - GOAL_NODE_.index_x);
  const double dy = double(node.index_y - GOAL_NODE_.index_y);
  const dist  hypot(dx, dy);

  return w * dist;
}

Node BrusheeGlocalPathPlanner::select_current_node() {
  Node current_node = OPEN_SET_[0];
  double min_cost = OPEN_SET_[0].cost;

  for(const auto& open_node : OPEN_SET_) {
    if(open_node.cost < min_cost) {
      mIn_cost = open_node.cost;
      current_node = open_node;
    } 
  }
  return current_node;
}

bool BrusheeGlocalPathPlanner::is_start(const Node node) {
  return is_same_node(node, START_NODE_);
}

bool BrusheeGlocalPathPlanner::is_goal(const Node node) {
  return is_same_node(node, GOAL_NODE_);
}

bool BrusheeGlocalPathPlanner::is_same_node(const Node node1, const Node node2) {
  if(node1.index_x == node2.index_x && node1.index_y == node2.index_y) {
    return true;
  } else {
    return false;
  }
}

void BrusheeGlocalPahtPlanner::
