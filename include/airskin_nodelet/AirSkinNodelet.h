#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <airskin_nodelet/I2C_Master.h>
#include <airskin_nodelet/AirSkinPad.h>
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
            ros::NodeHandle nh;
            ros::Publisher pressures_pub;
            ros::Subscriber colors_sub;
            std::string device_file_name;
            std::shared_ptr<I2C_Master> i2c_master;
            std::vector<AirSkinPad*> sensors;
            std::map<uint8_t,std::unique_ptr<AirSkinPad>> pads;
            bool airskin_ok;
            ros::Timer timer_;
            void timerCallback(const ros::TimerEvent& event);
            void colorsCallback(const tuw_airskin_msgs::AirskinColors::ConstPtr& colors);
            tf::TransformListener tf_listener_;
    };

}
