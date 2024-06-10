#ifndef BRUSHEE_GLOCAL_PATH_PLANNER_H
#define BRUSHEE_GLOCAL_PATH_PLANNER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h> 
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

struct Node {
  int index_x;
  int index_y;
  int parent_index_x;
  int parent_index_y;
  double cost;
};

struct Motion {
  double x;
  double y;
  double cost;
};

class BrusheeGlocalPathPlanner {
  public:
    BrusheeGlocalPathPlanner();
    void process();
  private:
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void pointgoalCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
    void posegoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);


    void update_set(const Node node);
    void create_path(Node node);
    void create_motion_model(std::vector<Motion>& motion_model);
    void create_neighbor_nodes(const Node current_node, std::vector<Node>& neighbor_nodes);
    void transfar_node(const Node node, std::vector<Node>& set1, std::vector<Node>& set2);

    bool is_obs(const Node node);
    bool is_start(const Node node);
    bool is_goal(const Node node);
    bool is_same_node(const Node node1, const Node node2);
    bool is_parent(const int cloed_node_index, const Node node);
    int search_node_from_set(const Node node, const std::vector<Node>& set);
    double calc_heuristic(const Node node);
    Node get_neighbor_node(const Node node, const Motion motion);
    Motion get_motion(const int dx, const int dy, const double cost);
    std::tuple<int, int> search_node(const Node node);
    geometry_msgs::PoseStamped calc_pose(const Node node);

    void path_planning();
    void show_node_point(const Node node);
    Node get_goal_node();
    Node select_current_node();

    int HZ_;
    double GOAL_TOLEARANCE_;
    bool is_map_ = false;
    bool is_pose_ = true;
    bool is_visualize_ = false;
    bool is_goal_in_obs_ = false;
    Node START_NODE_;
    Node GOAL_NODE_;
    std::vector<Node> OPEN_SET_;
    std::vector<Node> CLOSED_SET_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber sub_map_;
    ros::Subscriber sub_goal_;

    ros::Publisher pub_path_;
    ros::Publisher pub_node_point_;

    nav_msgs::OccupancyGrid map_;
    nav_msgs::Path path_;
    geometry_msgs::PointStamped point_goal_;
    geometry_msgs::PoseStamped pose_goal_;
    geometry_msgs::PointStamped base_link_goal_;
    geometry_msgs::PointStamped node_point_;

    tf::TransformListener tflistener_;
};

#endif // BRUSHEE_GLOCAL_PATH_PLANNER_H

  
