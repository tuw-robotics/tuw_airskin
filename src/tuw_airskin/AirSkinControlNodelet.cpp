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
  nh_.param<std::string>("cmd_pads", cmd_pads_tmp, "airskin_left, airskin_right");
  boost::erase_all(cmd_pads_tmp, " ");
  nh_.param("max_velocity", max_velocity_, 0.3);
  nh_.param("wheel_displacement", wheel_displacement_, 0.3);
  nh_.param("wheel_radius", wheel_radius_, 0.1);
  boost::split ( cmd_pads_, cmd_pads_tmp, boost::is_any_of(","));
  if(cmd_pads_.size() != 2){
    NODELET_ERROR("You have to define two pad names to send control commands");
  } else {
    NODELET_INFO("Using pad: %s and %s to generate controls", cmd_pads_[0].c_str(), cmd_pads_[1].c_str());
  }

  airskin_info_.header.seq = 0;
  pub_cmd_twist_ = n_.advertise<geometry_msgs::Twist>("airskin_cmd", 1);
  sub_pressures_ = n_.subscribe<tuw_airskin_msgs::AirskinPressures>("airskin_pressures", 1, &AirSkinControlNodelet::pressuresCallback, this);
  sub_info_ = n_.subscribe<tuw_airskin_msgs::AirskinInfo>("airskin_info", 1, &AirSkinControlNodelet::infoCallback, this);
}

void AirSkinControlNodelet::infoCallback(const tuw_airskin_msgs::AirskinInfo::ConstPtr &msg){
  idx_left_ = -1;
  idx_right_ = -1;
  if(cmd_pads_.size() == 2){
    for(size_t i = 0; i < msg->names.size(); i++){
      if(cmd_pads_[0].compare(msg->names[i]) == 0){
        idx_left_ = i;
      }
      if(cmd_pads_[1].compare(msg->names[i]) == 0){
        idx_right_ = i;
      }
    }
    if((idx_left_ >= 0) && (idx_right_ >= 0) && (idx_right_ != idx_left_)){
      airskin_info_ = *msg;
    } else {
      NODELET_ERROR("%s or %s are not found!", cmd_pads_[0].c_str(), cmd_pads_[1].c_str());
    }
  } else {
    NODELET_ERROR("You have to define two pad names to send control commands");
  }
}

void AirSkinControlNodelet::pressuresCallback(const tuw_airskin_msgs::AirskinPressures::ConstPtr &msg)
{
      
  if ((airskin_info_.header.seq == 0) || (idx_left_ >= msg->pressures.size()) || (idx_right_ >= msg->pressures.size())){
    NODELET_ERROR("left %i or right %i control pad index is out of bound!", idx_left_, idx_right_);
    return;
  }
  double pl = (double)(msg->pressures[idx_left_]  - airskin_info_.min[idx_left_])  / (double)( airskin_info_.max[idx_left_]  - airskin_info_.min[idx_left_]);
  double pr = (double)(msg->pressures[idx_right_] - airskin_info_.min[idx_right_]) / (double)( airskin_info_.max[idx_right_] - airskin_info_.min[idx_right_]);
  double vl = pl * max_velocity_;
  double vr = pr * max_velocity_;
  double w = (vr-vl) / wheel_displacement_;
  double v = (vr+vl)/2.0;
  geometry_msgs::Twist cmd_twist_;
  cmd_twist_.linear.x = v;
  cmd_twist_.angular.z = w;
  pub_cmd_twist_.publish(cmd_twist_);
  
}

void AirSkinControlNodelet::configCallback(tuw_airskin::AirSkinControlNodeletConfig &config, uint32_t level)
{
  config_ = config;
}
}

PLUGINLIB_EXPORT_CLASS(tuw::AirSkinControlNodelet, nodelet::Nodelet)
