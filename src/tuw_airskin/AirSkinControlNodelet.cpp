#include <pluginlib/class_list_macros.h>
#include <tuw_airskin/AirSkinControlNodelet.h>
#include <boost/algorithm/string.hpp>

namespace tuw
{
void AirSkinControlNodelet::onInit()
{
  nh_ = getPrivateNodeHandle();
  n_ = getNodeHandle();
  NODELET_INFO("Initializing AirSkinControlNodelet...");

  reconfigureFnc_ = boost::bind(&AirSkinControlNodelet::configCallback, this, _1, _2);
  reconfigureServer_.setCallback(reconfigureFnc_);

  
  std::string cmd_pads_tmp;
  boost::erase_all(cmd_pads_tmp, " []\"");
  boost::split ( cmd_pads_, cmd_pads_tmp, boost::is_any_of(","));
  if(cmd_pads_.size() != 2){
    NODELET_ERROR("You have to define two pad names to send control commands");
  } else {
    NODELET_INFO("Using pad: %s and %s to generate controls", cmd_pads_[0].c_str(), cmd_pads_[1].c_str());
  }

  pub_cmd_twist_ = n_.advertise<geometry_msgs::Twist>("airskin_cmd", 1);
  sub_pressures_ = n_.subscribe<tuw_airskin_msgs::AirskinPressures>("airskin_pressures", 1, &AirSkinControlNodelet::pressuresCallback, this);
}

void AirSkinControlNodelet::pressuresCallback(const tuw_airskin_msgs::AirskinPressures::ConstPtr &msg)
{
    
  tuw_airskin_msgs::AirskinColors colors;
  
  double pl = std::nan("1"), pr = std::nan("1");;
  for (size_t i = 0; i < msg->pressures.size(); i++)
  {  
    
  }
  
}

void AirSkinControlNodelet::configCallback(tuw_airskin::AirSkinControlNodeletConfig &config, uint32_t level)
{
  config_ = config;
}
}

PLUGINLIB_EXPORT_CLASS(tuw::AirSkinControlNodelet, nodelet::Nodelet)
