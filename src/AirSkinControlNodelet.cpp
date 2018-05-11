#include <pluginlib/class_list_macros.h>
#include <airskin_nodelet/AirSkinControlNodelet.h>

namespace tuw
{
void AirSkinControlNodelet::onInit()
{
  nh_ = getPrivateNodeHandle();
  NODELET_INFO("Initializing AirSkinControlNodelet...");

  reconfigureFnc_ = boost::bind(&AirSkinControlNodelet::configCallback, this, _1, _2);
  reconfigureServer_.setCallback(reconfigureFnc_);

  nh_.param("pad_names", pad_names_, std::vector<std::string>());
  nh_.param("pad_name_right", pad_name_right_, std::string("airskin_right"));
  nh_.param("pad_name_left", pad_name_left_, std::string("airskin_left"));
  nh_.param("w_max", w_max_, 1.5);
  nh_.param("wheel_radius", wheel_radius_, 0.097);
  nh_.param("wheel_displacement", wheel_displacement_, 0.33);
  nh_.param("publish_joint", publish_joint_, false);
  nh_.param("pressures_min", pressures_min_, std::vector<int>());
  nh_.param("pressures_max", pressures_max_, std::vector<int>());

  for (int i = 0; i < pad_names_.size(); i++)
  {
    pads_.emplace(pad_names_[i], i);
  }

  colors_pub_ = nh_.advertise<tuw_airskin_msgs::AirskinColors>("airskin_colors", 1);
  joint_pub_ = nh_.advertise<tuw_nav_msgs::JointsIWS>("/r0/joint_cmds", 1);
  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/r0/cmd_vel", 1);
  pressures_sub_ = nh_.subscribe<tuw_airskin_msgs::AirskinPressures>("airskin_pressures", 1,
                                                                     &AirSkinControlNodelet::pressuresCallback, this);

  //pressures_min_ = std::vector<unsigned int>(pad_names_.size(), std::numeric_limits<unsigned int>::max());
  //pressures_max_ = std::vector<unsigned int>(pad_names_.size(), std::numeric_limits<unsigned int>::min());
}

void AirSkinControlNodelet::pressuresCallback(const tuw_airskin_msgs::AirskinPressures::ConstPtr &pressures)
{
  // NODELET_INFO("pressuresCallback");
  std::vector<double> pressures_normalized = { 0, 0, 0, 0 };

  // update min / max pressure received
  for (size_t i = 0; i < pressures->pressures.size(); i++)
  {
    // std::cout << "pressures->pressures[" << i << "] = " << pressures->pressures[i] << std::endl;
    // std::cout << "pressures_min_[" << i << "] = " << pressures_min_[i] << std::endl;
    // std::cout << "pressures_max_[" << i << "] = " << pressures_max_[i] << std::endl;

    //if (pressures->pressures[i] < pressures_min_[i])
    //  pressures_min_[i] = pressures->pressures[i];

    //if (pressures->pressures[i] > pressures_max_[i])
    //  pressures_max_[i] = pressures->pressures[i];

    // scale pressure to 0 - 1

    int pressure_thres = 0;
    if((int)pressures->pressures[i] < pressures_min_[i])
      pressure_thres = pressures_min_[i];
    else if((int)pressures->pressures[i] > pressures_max_[i])
      pressure_thres = pressures_max_[i];
    else
      pressure_thres = (int)pressures->pressures[i];

    if (pressures_max_[i] - pressures_min_[i] > 0)
      pressures_normalized[i] =
          (double)(pressure_thres - pressures_min_[i]) / (double)(pressures_max_[i] - pressures_min_[i]);
  }

  // generate control output

  // for(size_t i = 0; i < pressures_normalized.size(); i++)
  //{
  //  std::cout << "pressures_normalized[" << i << "] = " << pressures_normalized[i] << std::endl;
  //}

  const double v = (pressures_normalized[pads_[pad_name_right_]] + pressures_normalized[pads_[pad_name_left_]]) *
                   w_max_ * wheel_radius_ / 2.;
  const double w = w_max_ * wheel_radius_ *
                   (pressures_normalized[pads_[pad_name_right_]] - pressures_normalized[pads_[pad_name_left_]]) /
                   wheel_displacement_;

  tuw_nav_msgs::JointsIWS joint;
  joint.header.stamp = ros::Time::now();
  joint.type_revolute = "cmd_velocity";
  joint.revolute.emplace_back(pressures_normalized[pads_[pad_name_right_]] *
                              w_max_);  // - pressures_normalized[pads_[pad_name_left_]] * w_max_);
  joint.revolute.emplace_back(pressures_normalized[pads_[pad_name_left_]] *
                              w_max_);  // - pressures_normalized[pads_[pad_name_right_]] * w_max_);

  geometry_msgs::Twist twist;
  twist.linear.x = v;
  twist.angular.z = w;

  if (publish_joint_)
    joint_pub_.publish(joint);

  twist_pub_.publish(twist);
}

void AirSkinControlNodelet::configCallback(tuw_airskin::AirSkinControlNodeletConfig &config, uint32_t level)
{
  config_ = config;
  w_max_ = config_.w_max;
  publish_joint_ = config.publish_joint;
}
}

PLUGINLIB_EXPORT_CLASS(tuw::AirSkinControlNodelet, nodelet::Nodelet)
