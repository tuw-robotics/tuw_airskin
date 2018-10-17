#ifndef AIRSKINCONTROL_NODELET_H
#define AIRSKINCONTROL_NODELET_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <tuw_airskin_msgs/AirskinColors.h>
#include <tuw_airskin_msgs/AirskinPressures.h>
#include <tf/transform_listener.h>
#include <tuw_airskin/AirSkinControlNodeletConfig.h>
#include <dynamic_reconfigure/server.h>

namespace tuw
{
class AirSkinControlNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle n_;
  ros::Subscriber sub_pressures_;
  ros::Publisher pub_cmd_twist_;

  std::vector<std::string> cmd_pads_;

  dynamic_reconfigure::Server<tuw_airskin::AirSkinControlNodeletConfig> reconfigureServer_;
  dynamic_reconfigure::Server<tuw_airskin::AirSkinControlNodeletConfig>::CallbackType reconfigureFnc_;

  tuw_airskin::AirSkinControlNodeletConfig config_;

  void pressuresCallback(const tuw_airskin_msgs::AirskinPressures::ConstPtr &pressures);
  void configCallback(tuw_airskin::AirSkinControlNodeletConfig &config, uint32_t level);
  geometry_msgs::Twist cmd_twist_;
};
}

#endif  // AIRSKINCONTROL_NODELET_H