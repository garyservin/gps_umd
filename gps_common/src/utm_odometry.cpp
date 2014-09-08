/*
 * Translates sensor_msgs/NavSat{Fix,Status} into nav_msgs/Odometry using UTM
 */

#include <gps_common/utm_odometry.h>

#include <gps_common/conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <boost/thread.hpp>

using namespace gps_common;

UTMOdometry::UTMOdometry() {
  ros::NodeHandle node;
  ros::NodeHandle priv_node("~");

  priv_node.param<std::string>("frame_id", frame_id, "");
  priv_node.param<std::string>("child_frame_id", child_frame_id, "");
  priv_node.param<double>("rot_covariance", rot_cov, 99999.0);

  odom_pub = node.advertise<nav_msgs::Odometry>("odom", 10);
  zone_pub = node.advertise<std_msgs::String>("zone", 10);

  fix_sub = node.subscribe<sensor_msgs::NavSatFix>("fix", 10, boost::bind(&UTMOdometry::callback, this, _1));
}


void UTMOdometry::callback(const sensor_msgs::NavSatFixConstPtr& fix) {
  if (fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
    ROS_INFO("No fix.");
    return;
  }

  if (fix->header.stamp == ros::Time(0)) {
    return;
  }

  double northing, easting;
  std_msgs::String zone;

  LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone.data);

  if (odom_pub) {
    nav_msgs::Odometry odom;
    odom.header.stamp = fix->header.stamp;

    if (frame_id.empty())
      odom.header.frame_id = "utm"; //fix->header.frame_id;
    else
      odom.header.frame_id = frame_id;

    odom.child_frame_id = child_frame_id;

    odom.pose.pose.position.x = easting;
    odom.pose.pose.position.y = northing;
    odom.pose.pose.position.z = fix->altitude;
    
    odom.pose.pose.orientation.x = 0;
    odom.pose.pose.orientation.y = 0;
    odom.pose.pose.orientation.z = 0;
    odom.pose.pose.orientation.w = 1;
    
    // Use ENU covariance to build XYZRPY covariance
    boost::array<double, 36> covariance = {{
      fix->position_covariance[0],
      fix->position_covariance[1],
      fix->position_covariance[2],
      0, 0, 0,
      fix->position_covariance[3],
      fix->position_covariance[4],
      fix->position_covariance[5],
      0, 0, 0,
      fix->position_covariance[6],
      fix->position_covariance[7],
      fix->position_covariance[8],
      0, 0, 0,
      0, 0, 0, rot_cov, 0, 0,
      0, 0, 0, 0, rot_cov, 0,
      0, 0, 0, 0, 0, rot_cov
    }};

    odom.pose.covariance = covariance;

    odom_pub.publish(odom);
    zone_pub.publish(zone);
  }
}
