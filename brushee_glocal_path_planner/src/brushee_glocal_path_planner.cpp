#include "brushee_glocal_path_planner/brushee_glocal_path_planner.h"

BrusheeGlocalPathPlanner::BrusheeGlocalPathPlanner():private_nh_("~") {
  private_nh_.param("HZ", HZ_,{10});

  sub_map_ = nh_.subscribe("/local_map", 1, &BrusheeGlocalPathPlanner::mapCallback, this);
  sub_goal_ = nh_.subscribe("/local_goal", 1, &BrusheeGlocalPathPlanner::goalCallback, this);

  path_.header.frame_id = "base_link";
  path_.poses.reserve(1000);
  
  pub_path_ = nh_.advertise<nav_msgs::Path>("/glocal_path", 1);
  pub_goal_ = nh_.advertise<geometry_msgs::PoseStamped>("/glocal_goal", 1);

  printf("%d\n", HZ_);

}

void BrusheeGlocalPathPlanner::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  is_map_ = true;
  map_ = *msg;
}

void BrusheeGlocalPathPlanner::goalCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
  map_goal_ = *msg;
  tf::StampedTransform transform;

  try {
    tflistener_.lookupTransform("base_link", map_goal_.header.frame_id, ros::Time(0), transform);
  }
  catch (tf::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }
  tf::Vector3 point(map_goal_.point.x, map_goal_.point.y, map_goal_.point.z);
  tf::Vector3 transformed_point = transform * point;

  base_link_goal_.header.frame_id = "base_link";
  base_link_goal_.header.stamp = ros::Time::now();
  base_link_goal_.point.x = transformed_point.getX();
  base_link_goal_.point.y = transformed_point.getY();
  base_link_goal_.point.z = transformed_point.getZ();
}

void BrusheeGlocalPathPlanner::process() {
  ros::Rate loop_rate(HZ_);
  while(ros::ok()) {
    if(is_map_) path_planning();
    // else ROS_WARN_STREAM("map is not received.");
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void BrusheeGlocalPathPlanner::path_planning() {
  OPEN_SET_.clear();
  CLOSED_SET_.clear();

  START_NODE_ = {100, 100, 0, 0, 0};
  GOAL_NODE_ = get_goal_node();

  START_NODE_.cost = calc_heuristic(START_NODE_);
  OPEN_SET_.push_back(START_NODE_);

  while(ros::ok()) {
    if(OPEN_SET_.empty()) {
      ROS_WARN_STREAM("No path found.");
      // exit(1);
    }

    Node current_node = select_current_node();

    if(is_goal(current_node)) {
      create_path(current_node);
      break;
    } else {
      transfar_node(current_node, OPEN_SET_, CLOSED_SET_);
      update_set(current_node);
    }
  }
  pub_path_.publish(path_);
  path_.poses.clear();
  // exit(0);
}

Node BrusheeGlocalPathPlanner::get_goal_node() {
  Node goal_node;
  goal_node.index_x = int(round((base_link_goal_.point.x - map_.info.origin.position.x) / map_.info.resolution));
  goal_node.index_y = int(round((base_link_goal_.point.y - map_.info.origin.position.y) / map_.info.resolution));

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
  const double dist = hypot(dx, dy);

  return w * dist;
}

Node BrusheeGlocalPathPlanner::select_current_node() {
  Node current_node = OPEN_SET_[0];
  double min_cost = OPEN_SET_[0].cost;

  for(const auto& open_node : OPEN_SET_) {
    if(open_node.cost < min_cost) {
      min_cost = open_node.cost;
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

void BrusheeGlocalPathPlanner::create_path(Node node) {
  nav_msgs::Path path;
  path.poses.push_back(calc_pose(node));

  while(!is_start(node)) {
    for(int i=0; i<CLOSED_SET_.size(); i++) {
      if(is_parent(i, node)) {
        node = CLOSED_SET_[i];
        path.poses.push_back(calc_pose(node));
        break;
      }
      if(i == CLOSED_SET_.size() - 1) {
        ROS_ERROR_STREAM("parent node is not found.");
        // exit(4);
      }
    }
  }

  reverse(path.poses.begin(), path.poses.end());
  path_.poses.insert(path_.poses.end(), path.poses.begin(), path.poses.end());
}

geometry_msgs::PoseStamped BrusheeGlocalPathPlanner::calc_pose(const Node node) {
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = node.index_x * map_.info.resolution + map_.info.origin.position.x;
  pose.pose.position.y = node.index_y * map_.info.resolution + map_.info.origin.position.y;

  return pose;
}

bool BrusheeGlocalPathPlanner::is_parent(const int closed_node_index, const Node node) {
  bool is_same_x = CLOSED_SET_[closed_node_index].index_x == node.parent_index_x;
  bool is_same_y = CLOSED_SET_[closed_node_index].index_y == node.parent_index_y;

  return is_same_x && is_same_y;
}

void BrusheeGlocalPathPlanner::transfar_node(const Node node, std::vector<Node>& set1, std::vector<Node>& set2) {
  const int set1_node_index = search_node_from_set(node, set1);
  if(set1_node_index == -1) {
    ROS_ERROR_STREAM("node is not found.");
    // exit(2);
  }

  set1.erase(set1.begin() + set1_node_index);
  set2.push_back(node);
}

int BrusheeGlocalPathPlanner::search_node_from_set(const Node node, const std::vector<Node>& set) {
  for(int i=0; i<set.size(); i++) {
    if(is_same_node(node, set[i])) return i;
  }
  return -1;
}

void BrusheeGlocalPathPlanner::update_set(const Node node) {
  std::vector<Node> neighbor_nodes;
  create_neighbor_nodes(node, neighbor_nodes);

  for(const auto& neighbor_node : neighbor_nodes) {
    if(is_obs(neighbor_node)) continue;

    int flag = 0;
    int node_index = 0;
    std::tie(flag, node_index) = search_node(neighbor_node);

    if(flag == -1) OPEN_SET_.push_back(neighbor_node);
    else if(flag == 1) {
      if(neighbor_node.cost < OPEN_SET_[node_index].cost) {
        OPEN_SET_[node_index].cost = neighbor_node.cost;
        OPEN_SET_[node_index].parent_index_x = neighbor_node.parent_index_x;
        OPEN_SET_[node_index].parent_index_y = neighbor_node.parent_index_y;
      }
    }
    else if(flag == 2) {
      if(neighbor_node.cost < CLOSED_SET_[node_index].cost) {
        CLOSED_SET_.erase(CLOSED_SET_.begin() + node_index);
        OPEN_SET_.push_back(neighbor_node);
      }
    }
  }
}

void BrusheeGlocalPathPlanner::create_neighbor_nodes(const Node node, std::vector<Node>& neighbor_nodes) {
  std::vector<Motion> motion_model;
  create_motion_model(motion_model);
  const int motion_num = motion_model.size();

  for(int i=0; i<motion_num; i++) {
    Node neighbor_node = get_neighbor_node(node, motion_model[i]);
    neighbor_nodes.push_back(neighbor_node);
  }
}

void BrusheeGlocalPathPlanner::create_motion_model(std::vector<Motion>& motion_model) {
  motion_model.push_back(get_motion(1, 0, 1));
  motion_model.push_back(get_motion(0, 1, 1));
  motion_model.push_back(get_motion(-1, 0, 1));
  motion_model.push_back(get_motion(0, -1, 1));

  motion_model.push_back(get_motion(1, 1, sqrt(2)));
  motion_model.push_back(get_motion(-1, 1, sqrt(2)));
  motion_model.push_back(get_motion(-1, -1, sqrt(2)));
  motion_model.push_back(get_motion(1, -1, sqrt(2)));
}

Motion BrusheeGlocalPathPlanner::get_motion(const int dx, const int dy, const double cost) {
  if(1 < abs(dx) || 1 < abs(dy)) {
    ROS_ERROR_STREAM("dx or dy is invalid.");
    // exit(3);
  }

  Motion motion;
  motion.x = dx;
  motion.y = dy;
  motion.cost = cost;

  return motion;
}

Node BrusheeGlocalPathPlanner::get_neighbor_node(const Node node, const Motion motion) {
  Node neighbor_node;

  neighbor_node.index_x = node.index_x + motion.x;
  neighbor_node.index_y = node.index_y + motion.y;

  neighbor_node.cost = (node.cost - calc_heuristic(node)) + calc_heuristic(neighbor_node) + motion.cost;

  neighbor_node.parent_index_x = node.index_x;
  neighbor_node.parent_index_y = node.index_y;

  return neighbor_node;
}

std::tuple<int, int> BrusheeGlocalPathPlanner::search_node(const Node node) {
  const int open_node_index = search_node_from_set(node, OPEN_SET_);
  if(open_node_index != -1) return std::make_tuple(1, open_node_index);

  const int closed_node_index = search_node_from_set(node, CLOSED_SET_);
  if(closed_node_index != -1) return std::make_tuple(2, closed_node_index);

  return std::make_tuple(-1, -1);
}


