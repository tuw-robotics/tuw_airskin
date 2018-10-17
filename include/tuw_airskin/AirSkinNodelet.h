#ifndef TUW_AIRSKIN_NODELET_H
#define TUW_AIRSKIN_NODELET_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <tuw_i2c/I2C_Master.h>
#include <tuw_airskin/AirSkinPad.h>
#include <tuw_airskin_msgs/AirskinColors.h>
#include <tuw_airskin_msgs/AirskinPressures.h>
#include <tf/transform_listener.h>
namespace tuw
{
class AirSkinNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit();
  void run();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle n_;
  ros::Publisher pub_pressures_;
  ros::Publisher pub_contact_;
  ros::Subscriber sub_colors_;
  std::string device_file_name_;
  std::string frame_id_;
  double contact_treshold_; 
  bool auto_calibration_;
  std::vector<int> pressures_min_;
  std::vector<int> pressures_max_;
  std::shared_ptr<I2C_Master> i2c_master_;
  std::vector<std::shared_ptr<AirSkinPad> > pads_;
  ros::Timer timer_;
  std_msgs::Bool contact_;
  tuw_airskin_msgs::AirskinPressures airskin_pressures_;
  void timerCallback(const ros::TimerEvent& event);
  void colorsCallback(const tuw_airskin_msgs::AirskinColors::ConstPtr& colors);
  tf::TransformListener tf_listener_;
};
}

#endif  // TUW_AIRSKIN_NODELET_H
