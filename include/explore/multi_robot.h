/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Zhi Yan.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Zhi Yan nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#ifndef EXPLORE_MULTI_ROBOT_H
#define EXPLORE_MULTI_ROBOT_H

/*
 * Multi-robot support for the exploration package.
 */

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <move_base_msgs/MoveBaseGoal.h>

namespace explore {
  class MultiRobot {
  public:
    MultiRobot(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
    ~MultiRobot();
    
    double inscribed_scale_;
    std::string tf_prefix_;
    
    bool external_map_received_;
    nav_msgs::OccupancyGrid external_map_;
    ros::Subscriber external_map_subsriber_;
    void externalMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void updateMap(costmap_2d::Costmap2DROS &costmap);
    
    bool external_goal_received_;
    move_base_msgs::MoveBaseGoal external_goal_;
    ros::Subscriber external_goal_subsriber_;
    void externalGoalCallback(const move_base_msgs::MoveBaseGoal::ConstPtr &msg);
    void updateGoal(move_base_msgs::MoveBaseGoal &goal);
    
    tf::TransformListener listener_;
    bool start_transform_known_;
    
    bool prev_pose_known_;
    double distance_traveled_;
    geometry_msgs::PoseStamped prev_pose_;
    ros::Publisher distance_traveled_publisher_;
    void distanceTraveled(const geometry_msgs::PoseStamped& pose, const std::string &mode);
    
    ros::Publisher estimatedcost_publisher_;
    void estimatedCost(const geometry_msgs::PoseStamped& goal, const geometry_msgs::PoseStamped& pose);
  };
}

#endif /* EXPLORE_MULTI_ROBOT_H */
