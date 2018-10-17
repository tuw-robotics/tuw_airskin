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

  for (int i = 0; i < pad_names_.size(); i++)
  {
    pads_.emplace(pad_names_[i], i);
  }

  pub_colors_ = n_.advertise<tuw_airskin_msgs::AirskinColors>("airskin_colors", 1);
  sub_pressures_ = n_.subscribe<tuw_airskin_msgs::AirskinPressures>("airskin_pressures", 1, &AirSkinColorNodelet::pressuresCallback, this);
}

void AirSkinColorNodelet::pressuresCallback(const tuw_airskin_msgs::AirskinPressures::ConstPtr &msg)
{
      
  tuw_airskin_msgs::AirskinColors colors;
  colors.header = msg->header;
  colors.idx.resize(msg->pressures.size());
  colors.colors.resize(msg->pressures.size());
  for (size_t i = 0; i < msg->pressures.size(); i++)
  {    
    colors.idx[i] = i;
    double v = (double)(msg->pressures[i] - msg->min[i]) / (double)( msg->max[i] -  msg->min[i]);
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
