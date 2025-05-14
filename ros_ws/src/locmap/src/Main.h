//
// Created by jamestbest on 3/27/25.
//

#ifndef CATKIN_WS_MAIN_H
#define CATKIN_WS_MAIN_H

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "actionlib_msgs/GoalID.h"

#include "locmap/LocMapLocations.h"
#include "locmap/LocMapLocation.h"
#include "locmap/LocMapGoto.h"

static void create_subscribers(ros::NodeHandle node);
static void create_publishers(ros::NodeHandle node);

static void goal_callback(geometry_msgs::PoseStamped goal);
static void goto_callback(locmap::LocMapGoto goal_location);

static bool deserialize_all_files();
static bool serialize_location(const locmap::LocMapLocation location);

static void publish_locations();

static bool find_location(std::string name, locmap::LocMapLocation* loc_out);

static void print_cwd();

#endif //CATKIN_WS_MAIN_H
