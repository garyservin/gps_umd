/*
 * Translates sensor_msgs/NavSat{Fix,Status} into nav_msgs/Odometry using UTM
 */

#include <gps_common/utm_odometry.h>

int main (int argc, char **argv) {
  ros::init(argc, argv, "utm_odometry_node");

  gps_common::UTMOdometry odom;

  ros::spin();
}

