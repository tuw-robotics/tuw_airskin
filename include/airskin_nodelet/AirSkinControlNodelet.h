#ifndef AIRSKINCONTROL_NODELET_H
#define AIRSKINCONTROL_NODELET_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <tuw_airskin_msgs/AirskinColors.h>
#include <tuw_airskin_msgs/AirskinPressures.h>
#include <geometry_msgs/Twist.h>
#include <tuw_nav_msgs/JointsIWS.h>
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
  ros::Subscriber pressures_sub_;
  ros::Publisher colors_pub_;
  ros::Publisher twist_pub_;
  ros::Publisher joint_pub_;

  tf::TransformListener tf_listener_;

  std::vector<std::string> pad_names_;
  std::map<std::string, int> pads_;
  std::string pad_name_left_;
  std::string pad_name_right_;
  std::vector<unsigned int> pressures_min_;
  std::vector<unsigned int> pressures_max_;
  double w_max_;
  double wheel_radius_;
  double wheel_displacement_;
  bool publish_joint_;

  dynamic_reconfigure::Server<tuw_airskin::AirSkinControlNodeletConfig> reconfigureServer_;
  dynamic_reconfigure::Server<tuw_airskin::AirSkinControlNodeletConfig>::CallbackType reconfigureFnc_;

  tuw_airskin::AirSkinControlNodeletConfig config_;

  void pressuresCallback(const tuw_airskin_msgs::AirskinPressures::ConstPtr &pressures);
  void configCallback(tuw_airskin::AirSkinControlNodeletConfig &config, uint32_t level);
};
}

#endif  // AIRSKINCONTROL_NODELET_H
