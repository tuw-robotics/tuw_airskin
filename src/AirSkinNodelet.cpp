#include <pluginlib/class_list_macros.h>
#include <airskin_nodelet/AirSkinNodelet.h>
#include <string>

namespace tuw
{
    void AirSkinNodelet::onInit() 
    {
        nh = getPrivateNodeHandle();
        nh.param("device",device_file_name,std::string("/dev/ttyUSB0"));
        NODELET_INFO("Initializing AirSkinNodelet...");
        NODELET_INFO("opening I2C device: '%s'", device_file_name.c_str());
        i2c_master = new I2C_Master_Devantech_ISS(device_file_name);
        NODELET_INFO("Devantech USB-ISS adapter, rev. %d",
            ((I2C_Master_Devantech_ISS*)i2c_master)->GetFirmwareVersion());
        
        
        
        std::vector<std::string> pad_names;
        std::vector<int> pad_i2c_ids;
        nh.param("pad_names",pad_names,std::vector<std::string>());
        nh.param("pad_i2c_ids",pad_i2c_ids,std::vector<int>());
        if(pad_names.size()<pad_i2c_ids.size()){
            for(int i=0;i<pad_i2c_ids.size()-pad_names.size();i++) {
                pad_names.emplace_back("no name specified");
            }
        }
        NODELET_INFO("Get pads from yaml file:");
        for(int i=0;i<pad_i2c_ids.size();i++) {
            NODELET_INFO("ID: %#4x = %s",pad_i2c_ids[i],pad_names[i].c_str());
            pads.emplace(pad_i2c_ids[i],new AirSkinPad(i2c_master,2*pad_i2c_ids[i], pad_names[i]));
        }
        size_t ok_cnt = 0;
        for(const auto& pad : pads)
        {
            if(pad.second->init())
            ok_cnt++;
        }
        airskin_ok = (ok_cnt == pads.size());        

        pressures_pub = nh.advertise<tuw_airskin_msgs::AirskinPressures>("airskin_pressures",1);
        colors_sub = nh.subscribe<tuw_airskin_msgs::AirskinColors>("airskin_colors", 1, &AirSkinNodelet::colorsCallback, this );
        timer_ = nh.createTimer(ros::Duration(0.01), boost::bind(& AirSkinNodelet::timerCallback, this, _1));
    }
    
    void AirSkinNodelet::colorsCallback(const tuw_airskin_msgs::AirskinColors::ConstPtr& colors) {
        for(int i = 0;i<colors->ids.size();i++) {
            const uint8_t id = colors->ids[i];
            try
            {
                pads.at(id)->setColor(colors->colors[i]);
            }
            catch (const std::out_of_range& oor)
            {
                NODELET_INFO("id not found");
            }
        }
    }
    
    void AirSkinNodelet::timerCallback(const ros::TimerEvent& event) {
        tuw_airskin_msgs::AirskinPressures pressures;

        pressures.header.stamp = ros::Time().now();
        for(const auto& pad: pads)
        {
            pad.second->update();
            pressures.ids.emplace_back(pad.second->getAddr());
            pressures.pressures.emplace_back(pad.second->getPressure());
            pressures.names.emplace_back(pad.second->getName());
        }
        pressures_pub.publish(pressures);
    }
}
PLUGINLIB_EXPORT_CLASS(tuw::AirSkinNodelet, nodelet::Nodelet) 
