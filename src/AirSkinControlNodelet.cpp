#include <pluginlib/class_list_macros.h>
#include <airskin_nodelet/AirSkinControlNodelet.h>

namespace tuw
{
void AirSkinControlNodelet::onInit()
{
  nh_ = getPrivateNodeHandle();
  n_ = getNodeHandle();
  NODELET_INFO("Initializing AirSkinControlNodelet...");

  reconfigureFnc_ = boost::bind(&AirSkinControlNodelet::configCallback, this, _1, _2);
  reconfigureServer_.setCallback(reconfigureFnc_);

  nh_.param("pad_names", pad_names_, std::vector<std::string>());

  for (int i = 0; i < pad_names_.size(); i++)
  {
    pads_.emplace(pad_names_[i], i);
  }

  pub_cmd_twist_ = n_.advertise<geometry_msgs::Twist>("airskin_cmd", 1);
  sub_pressures_ = n_.subscribe<tuw_airskin_msgs::AirskinPressures>("airskin_pressures", 1, &AirSkinControlNodelet::pressuresCallback, this);
}

void AirSkinControlNodelet::pressuresCallback(const tuw_airskin_msgs::AirskinPressures::ConstPtr &msg)
{
    
  NODELET_INFO("pressuresCallback");
}

void AirSkinControlNodelet::configCallback(tuw_airskin::AirSkinControlNodeletConfig &config, uint32_t level)
{
  config_ = config;
}
}

PLUGINLIB_EXPORT_CLASS(tuw::AirSkinControlNodelet, nodelet::Nodelet)
