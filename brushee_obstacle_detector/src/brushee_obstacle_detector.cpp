#include "brushee_obstacle_detector/brushee_obstacle_detector.h"

BrusheeObstacleDetector::BrusheeObstacleDetector():private_nh_("~")
{
    private_nh_.getParam("hz", hz_);
    private_nh_.getParam("laser_step", laser_step_);

    laser_scan_sub_ = nh_.subscribe("/scan", 1, &BrusheeObstacleDetector::laser_scan_callback, this);
    obstacle_pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/obstacle_pose", 1);

    obstacle_pose_array_.header.frame_id = "base_link";
}

void BrusheeObstacleDetector::process()
{
    ros::Rate loop_rate(hz_);
    while(ros::ok())
    {
        if(flag_laser_scan_)
        {
            scan_obstacle();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}


void BrusheeObstacleDetector::laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser_scan_ = *msg;
    flag_laser_scan_ = true;
}


void BrusheeObstacleDetector::scan_obstacle()
{
    obstacle_pose_array_.poses.clear();

    for(int i=0;i<laser_scan_.ranges.size();i+=laser_step_)
    {
        const double angle = laser_scan_.angle_min + laser_scan_.angle_increment * i;
        const double range = laser_scan_.ranges[i];
        geometry_msgs::Pose obs_pose;
        obs_pose.position.x = range * cos(angle + M_PI/4);
        obs_pose.position.y = range * sin(angle + M_PI/4);
        obstacle_pose_array_.poses.push_back(obs_pose);

    }
    obstacle_pose_pub_.publish(obstacle_pose_array_);
}