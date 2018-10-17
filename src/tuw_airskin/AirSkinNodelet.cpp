#include <pluginlib/class_list_macros.h>
#include <tuw_airskin/AirSkinNodelet.h>
//#include <airskin_nodelet/I2C_Master_MPSSE.h>
#include <tuw_i2c/I2C_Master_Devantech_ISS.h>
#include <string>
#include <memory>

namespace tuw {
void AirSkinNodelet::onInit() {
    nh_ = getPrivateNodeHandle();
    n_ = getNodeHandle();
    nh_.param ( "i2c_device", device_file_name_, std::string ( "/dev/ttyUSB0" ) );
    NODELET_INFO ( "Initializing AirSkinNodelet..." );
    NODELET_INFO ( "Opening I2C device: '%s'", device_file_name_.c_str() );
    if ( device_file_name_ == "UM232H-B" ) {
        // i2c_master = std::make_shared<I2C_Master_MPSSE>();
    } else {
        i2c_master_ = std::make_shared<I2C_Master_Devantech_ISS>();
        NODELET_INFO ( "Devantech USB-ISS adapter, rev. %d", i2c_master_->GetFirmwareVersion() );
    }

    std::vector<std::string> pad_names;
    std::vector<int> pad_i2c_ids;
    nh_.param<std::string> ("frame_id", frame_id_, "airskin");
    nh_.param ( "pad_names", pad_names, std::vector<std::string>());
    nh_.param ( "pad_i2c_ids", pad_i2c_ids, std::vector<int>() );
    nh_.param ( "contact_treshold", contact_treshold_, 0.3 );
    nh_.param ( "pressures_min", pressures_min_, std::vector<int>() );
    nh_.param ( "pressures_max", pressures_max_, std::vector<int>() );
    nh_.param ( "auto_calibration", auto_calibration_, true );

    if ( (pad_names.size() != pad_i2c_ids.size() ) ||  (pad_names.size() !=  pressures_min_.size()) || (pad_names.size() != pressures_max_.size())) {
        NODELET_ERROR ( "Size of pad_names (%zu), pad_i2c_ids (%zu), pressures_min (%zu) and pressures_max (%zu) do not match!",
            pad_names.size(), pad_i2c_ids.size(), pressures_min_.size(), pressures_max_.size() );
    }
    
    NODELET_INFO ( "Get pads from yaml file:" );
    for ( int i = 0; i < pad_i2c_ids.size(); i++ ) {
        std::shared_ptr<AirSkinPad> pad = std::make_shared<AirSkinPad> ( i2c_master_, 2 * pad_i2c_ids[i], pad_names[i] ); 
        if ( pad->init() ) {
            NODELET_INFO ( "ID: %#4x = %s initialized", pad_i2c_ids[i], pad_names[i].c_str() );
            pads_.push_back(pad);
            pad->setColor ( 200, 255, 0 );
        } else {
            NODELET_WARN ( "Could not initialize ID: %#4x = %s", pad_i2c_ids[i], pad_names[i].c_str() );
        }
        
    }
    airskin_pressures_.header.frame_id = frame_id_;
    airskin_pressures_.pressures.resize ( pads_.size() );
    airskin_pressures_.names.resize ( pads_.size() );
    airskin_pressures_.ids.resize ( pads_.size() );
    airskin_pressures_.min.resize ( pads_.size() );
    airskin_pressures_.max.resize ( pads_.size() );

    pub_pressures_ = n_.advertise<tuw_airskin_msgs::AirskinPressures> ( "airskin_pressures", 1 );
    pub_contact_ = n_.advertise<std_msgs::Bool> ( "airskin_contact", 1 );
    
    sub_colors_ = n_.subscribe<tuw_airskin_msgs::AirskinColors> ( "airskin_colors", 1, &AirSkinNodelet::colorsCallback, this );
    timer_ = n_.createTimer ( ros::Duration ( 0.1 ), boost::bind ( &AirSkinNodelet::timerCallback, this, _1 ) );
}

void AirSkinNodelet::colorsCallback ( const tuw_airskin_msgs::AirskinColors::ConstPtr& colors ) {
    for ( size_t i = 0; i < colors->idx.size(); i++) {
        uint16_t idx = colors->idx[i];
        if( idx < pads_.size() ){
            std::shared_ptr<AirSkinPad> pad = pads_[idx];
            try {
                pad->setColor ( colors->colors[i] );
            } catch ( const std::out_of_range& oor ) {
                NODELET_ERROR ( "Error on set colour" );
            }
        } else {
            NODELET_WARN ( "could net set colour, idx=%i out of bound", idx );
        }
    }
}

void AirSkinNodelet::timerCallback ( const ros::TimerEvent& event ) {
    airskin_pressures_.header.stamp = ros::Time().now();
    contact_.data = false;
    for ( size_t i = 0; i < pads_.size(); i++) {
        std::shared_ptr<AirSkinPad> pad = pads_[i];
        pad->update();
        airskin_pressures_.ids[i] = pad->getAddr();
        airskin_pressures_.pressures[i] = pad->getPressure();
        if(auto_calibration_) {
          if ( airskin_pressures_.pressures[i] < pressures_min_[i] ) {
              pressures_min_[i] = airskin_pressures_.pressures[i];
          }
          if ( airskin_pressures_.pressures[i] > pressures_max_[i] ) {
              pressures_max_[i] = airskin_pressures_.pressures[i];
          }
        }
        airskin_pressures_.min[i] =  pressures_min_[i];
        airskin_pressures_.max[i] =  pressures_max_[i]; 
        double pressure_normalized = (double)(airskin_pressures_.pressures[i] - airskin_pressures_.min[i]) / (double)( airskin_pressures_.max[i] -  airskin_pressures_.min[i]);
        if(pressure_normalized > contact_treshold_) {
            contact_.data = true;
        }
        airskin_pressures_.names[i] = pad->getName();
    }
    pub_pressures_.publish ( airskin_pressures_ );
    pub_contact_.publish ( contact_ );
}
}
PLUGINLIB_EXPORT_CLASS ( tuw::AirSkinNodelet, nodelet::Nodelet )
