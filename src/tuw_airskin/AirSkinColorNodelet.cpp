#include <pluginlib/class_list_macros.h>
#include <tuw_airskin/AirSkinColorNodelet.h>

namespace tuw
{
void AirSkinColorNodelet::onInit()
{
  nh_ = getPrivateNodeHandle();
  n_ = getNodeHandle();
  NODELET_INFO("Initializing AirSkinColorNodelet...");

  reconfigureFnc_ = boost::bind(&AirSkinColorNodelet::configCallback, this, _1, _2);
  reconfigureServer_.setCallback(reconfigureFnc_);

  nh_.param("pad_names", pad_names_, std::vector<std::string>());
  nh_.param("info_timeout", info_timeout_, 1.0);

  for (int i = 0; i < pad_names_.size(); i++)
  {
    pads_.emplace(pad_names_[i], i);
  }

  airskin_info_.header.seq = 0;
  pub_colors_ = n_.advertise<tuw_airskin_msgs::AirskinColors>("airskin_colors", 1);
  sub_pressures_ = n_.subscribe<tuw_airskin_msgs::AirskinPressures>("airskin_pressures", 1, &AirSkinColorNodelet::pressuresCallback, this);
  sub_info_ = n_.subscribe<tuw_airskin_msgs::AirskinInfo>("airskin_info", 1, &AirSkinColorNodelet::infoCallback, this);
}

void AirSkinColorNodelet::infoCallback(const tuw_airskin_msgs::AirskinInfo::ConstPtr &msg){
  airskin_info_ = *msg;
}

void AirSkinColorNodelet::pressuresCallback(const tuw_airskin_msgs::AirskinPressures::ConstPtr &msg)
{
  ros::Duration d = msg->header.stamp - airskin_info_.header.stamp;
  if((airskin_info_.header.seq != 0) && (d.toSec() > info_timeout_)){
    NODELET_INFO("Airskin Info msg to old to compute colours");
    return;
  }
  tuw_airskin_msgs::AirskinColors colors;
  colors.header = msg->header;
  colors.idx.resize(msg->pressures.size());
  colors.colors.resize(msg->pressures.size());
  for (size_t i = 0; i < msg->pressures.size(); i++)
  {    
    colors.idx[i] = i;
    double v = (double)(msg->pressures[i] - airskin_info_.min[i]) / (double)( airskin_info_.max[i] - airskin_info_.min[i]);
    std_msgs::ColorRGBA &color = colors.colors[i];
    color.a = 1.;
    color.r = fabs(sin(v*M_PI/2.));
    color.g = fabs(cos(v*M_PI/2.));
    color.b = 0;
  }
  pub_colors_.publish(colors);
}

void AirSkinColorNodelet::configCallback(tuw_airskin::AirSkinColorNodeletConfig &config, uint32_t level)
{
  config_ = config;
}
}

PLUGINLIB_EXPORT_CLASS(tuw::AirSkinColorNodelet, nodelet::Nodelet)
