#ifndef AIRSKINCONTROL_NODELET_H
#define AIRSKINCONTROL_NODELET_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <tuw_airskin_msgs/AirskinColors.h>
#include <tuw_airskin_msgs/AirskinPressures.h>
#include <geometry_msgs/Twist.h>
#include <tuw_nav_msgs/JointsIWS.h>
#include <tf/transform_listener.h>
#include <tuw_airskin/AirSkinColorNodeletConfig.h>
#include <dynamic_reconfigure/server.h>

namespace tuw
{
class AirSkinColorNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle n_;
  ros::Subscriber sub_pressures_;
  ros::Publisher pub_colors_;

  tf::TransformListener tf_listener_;

  std::vector<std::string> pad_names_;
  std::map<std::string, int> pads_;
  bool publish_joint_;

  dynamic_reconfigure::Server<tuw_airskin::AirSkinColorNodeletConfig> reconfigureServer_;
  dynamic_reconfigure::Server<tuw_airskin::AirSkinColorNodeletConfig>::CallbackType reconfigureFnc_;

  tuw_airskin::AirSkinColorNodeletConfig config_;

  void pressuresCallback(const tuw_airskin_msgs::AirskinPressures::ConstPtr &pressures);
  void configCallback(tuw_airskin::AirSkinColorNodeletConfig &config, uint32_t level);
};
}

#endif  // AIRSKINCONTROL_NODELET_H
