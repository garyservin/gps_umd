#include <ros/ros.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>

namespace gps_common {
  class UTMOdometry {
    public:
      UTMOdometry();
    private:
      void callback(const sensor_msgs::NavSatFixConstPtr& fix);

      // Publish GPS information as an Odometry message using UTM
      ros::Publisher odom_pub;

      // Publishes corresponding UTM zone information
      ros::Publisher zone_pub;

      // Subscribes to NavSatfix information
      ros::Subscriber fix_sub;

      std::string frame_id, child_frame_id;
      double rot_cov;
  };
};
