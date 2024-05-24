/**
 * @file local_goal_creator.cpp
 * @author Takuma Tanaka
 * @brief  This file is the source file for the local_goal_creator class.
 * @version 0.1
 * @date 2023-09-12
 *
 */

#include "brushee_local_map_creator/brushee_local_map_creator.h"

BrusheeLocalMapCreator::BrusheeLocalMapCreator() : private_nh_("~") {
  private_nh_.getParam("hz", hz_);
  private_nh_.getParam("map_size", local_map_size_);
  private_nh_.getParam("map_reso", local_map_resolution_);
  private_nh_.getParam("expand_size", expand_size_);
  private_nh_.getParam("is_expand", is_expand_);
  private_nh_.getParam("is_costmap", is_costmap_);

  sub_obs_poses_ = nh_.subscribe(
      "/obstacle_pose", 1, &BrusheeLocalMapCreator::obs_poses_callback, this);
  sub_costmap_ = nh_.subscribe(
      "/costmap_creator/costmap", 1,
      &BrusheeLocalMapCreator::costmap_callback, this);
  pub_local_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("/local_map", 1);

  local_map_raw_.header.frame_id = "base_link";
  local_map_.header.frame_id = "base_link";

  local_map_.info.resolution = local_map_resolution_;
  local_map_.info.width = int(round(local_map_size_ / local_map_resolution_));
  local_map_.info.height = int(round(local_map_size_ / local_map_resolution_));
  local_map_.info.origin.position.x = -local_map_size_ / 2.0;
  local_map_.info.origin.position.y = -local_map_size_ / 2.0;

  local_map_raw_.info.resolution = local_map_resolution_;
  local_map_raw_.info.width =
      int(round(local_map_size_ / local_map_resolution_));
  local_map_raw_.info.height =
      int(round(local_map_size_ / local_map_resolution_));
  local_map_raw_.info.origin.position.x = -local_map_size_ / 2.0;
  local_map_raw_.info.origin.position.y = -local_map_size_ / 2.0;

  local_map_.data.reserve(local_map_.info.width * local_map_.info.height);
  local_map_raw_.data.reserve(local_map_.info.width * local_map_.info.height);
}

void BrusheeLocalMapCreator::obs_poses_callback(
    const geometry_msgs::PoseArrayConstPtr &msg) {
  obs_poses_ = *msg;
  is_get_obs_poses_ = true;
}

void BrusheeLocalMapCreator::costmap_callback(
    const nav_msgs::OccupancyGridConstPtr &msg) {
  costmap_ = *msg;
  is_get_obs_poses_ = true;
}

void BrusheeLocalMapCreator::process() {
  ros::Rate loop_rate(hz_);

  while (ros::ok()) {
    if (is_get_obs_poses_) {
      init_local_map();
      update_local_map();
      pub_local_map_.publish(local_map_);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void BrusheeLocalMapCreator::init_local_map() {
  local_map_raw_.data.clear();
  if(is_costmap_) {
    local_map_raw_ = costmap_;
  } else {
    local_map_raw_.data.assign(local_map_.info.width * local_map_.info.height, 0);
  }
}

void BrusheeLocalMapCreator::update_local_map() {

  for (int i = 0; i < obs_poses_.poses.size(); i++) {
    const double x = obs_poses_.poses[i].position.x;
    const double y = obs_poses_.poses[i].position.y;
    const double dist = hypot(x, y);
    const double angle = atan2(y, x);

    if(!is_costmap_ && is_get_costmap_) {
      for (double dist_from_robot = 0.0;
          (dist_from_robot < dist && is_in_local_map(dist_from_robot, angle));
          dist_from_robot += local_map_resolution_) {
        const int grid_index = get_grid_index(dist_from_robot, angle);
        local_map_raw_.data[grid_index] = 0;
      }
    }

    if (is_in_local_map(dist, angle)) {
      const int grid_index = xy_to_index(x, y);
      local_map_raw_.data[grid_index] = 100;
    }
  }

  local_map_ = local_map_raw_;

  if(is_expand_) {
    for (int i = 0; i < local_map_raw_.data.size(); i++) {
      if (local_map_raw_.data[i] == 100) {
        int index_x = i % local_map_raw_.info.width;
        int index_y = i / local_map_raw_.info.width;
        for (int j = -expand_size_; j < expand_size_; j++) {
          for (int k = -expand_size_; k < expand_size_; k++) {
            int index =
                (index_x + k) + ((index_y + j) * local_map_raw_.info.width);
            local_map_.data[index] = 100;
          }
        }
      }
    }
  }
}

bool BrusheeLocalMapCreator::is_in_local_map(const double dist,
                                             const double angle) {
  const double x = dist * cos(angle);
  const double y = dist * sin(angle);
  const int index_x = int(round((x - local_map_.info.origin.position.x) /
                                local_map_.info.resolution));
  const int index_y = int(round((y - local_map_.info.origin.position.y) /
                                local_map_.info.resolution));

  if (index_x < local_map_.info.width && index_x >= 0 &&
      index_y < local_map_.info.height && index_y >= 0) {
    return true;
  } else {
    return false;
  }
}

int BrusheeLocalMapCreator::get_grid_index(const double dist,
                                           const double angle) {
  const double x = dist * cos(angle);
  const double y = dist * sin(angle);
  int index = xy_to_index(x, y);
  return index;
}

int BrusheeLocalMapCreator::xy_to_index(const double x, const double y) {
  const int index_x = int(round((x - local_map_.info.origin.position.x) /
                                local_map_.info.resolution));
  const int index_y = int(round((y - local_map_.info.origin.position.y) /
                                local_map_.info.resolution));

  int index = index_y * local_map_.info.width + index_x;
  return index;
}