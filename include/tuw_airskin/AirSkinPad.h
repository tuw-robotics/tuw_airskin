#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tuw_i2c/Except.h>
#include <tuw_i2c/I2C_Master_Devantech_ISS.h>
#include <tuw_airskin/AirSkin_Sense.h>
#include <std_msgs/ColorRGBA.h>
#include <memory>

class RunningMean
{
    std::vector<int> history;  // ring buffer
    int pos;
    float mean;
    int numFilled;

public:
    RunningMean ( int size )
    {
        history.resize ( size, 0 );
        pos = 0;
        numFilled = 0;
        mean = 0.;
    }
    /**
     * Initial filling of the ringbuffer.
     * @return true if buffer is now completely filled, false otherwise
     */
    bool fill ( int val )
    {
        if ( numFilled < history.size() ) {
            history[pos] = val;
            mean += ( float ) history[pos] / ( float ) history.size();
            pos++;
            if ( pos >= history.size() ) {
                pos -= history.size();
            }
            numFilled++;
            return false;
        } else {
            return true;
        }
    }
    void updateMean ( int val )
    {
        // remove oldest
        mean -= ( float ) history[pos] / ( float ) history.size();
        // and add new
        history[pos] = val;
        mean += ( float ) history[pos] / ( float ) history.size();
        pos++;
        if ( pos >= history.size() ) {
            pos -= history.size();
        }
    }
    float getMean()
    {
        return mean;
    }
};

class AirSkinPad
{
private:
    static const int VALID_PRESSURE_MIN = 90000;
    static const int VALID_PRESSURE_MAX = 300000;
    static const int HISTORY_SIZE = 50;
    static const int ACTIVATION_THR = 50;
    static const int ACTIVATION_HYST = 25;
    static const int UNFREEZE_DLEAY = 5;

    AirSkin_Sense sensor;
    RunningMean mean;
    unsigned char addr;            // 8 Bit I2C address
    std::string name;              // readable name of respective pad
    int p;                         // current pressure
    bool is_activated;             // activation status
    bool ref_is_frozen;            // true if reference value update is frozen
    ros::Time ref_unfreeze_timer;  // we unfreeze the reference value a little bit after a release

    void updateReference()
    {
        mean.updateMean ( p );
    }

public:
    AirSkinPad ( std::shared_ptr<I2C_Master> &_master, unsigned char _addr, const std::string &_name );
    ~AirSkinPad();
    // NOTE: apparently we have I2C wiring issues, so somtimes -1 or some
    // crazy value is returned -> do sanity check
    bool isValidPressure ( int p )
    {
        return VALID_PRESSURE_MIN < p && p < VALID_PRESSURE_MAX;
    }
    std::string getName()
    {
        return name;
    }
    unsigned char getAddr()
    {
        return addr / 2;
    }
    bool isActivated()
    {
        return is_activated;
    }
    int getPressure()
    {
        return p;
    }
    int getReference()
    {
        return mean.getMean();
    }
    bool init();
    void update();
    void setColor ( std_msgs::ColorRGBA color );
    void setColor ( uint8_t red, uint8_t green, uint8_t blue );
};
