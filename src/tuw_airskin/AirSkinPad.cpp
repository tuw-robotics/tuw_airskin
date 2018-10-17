#include <tuw_airskin/AirSkinPad.h>
#include <nodelet/nodelet.h>

AirSkinPad::AirSkinPad(std::shared_ptr<I2C_Master> &_master, unsigned char _addr, const std::string &_name)
  : sensor(_master, _addr), mean(HISTORY_SIZE)
{
  addr = _addr;
  name = _name;
  p = 0;
  // set to true initially, to be on the safe side
  is_activated = true;
  ref_is_frozen = false;
  ref_unfreeze_timer = ros::Time(0.);
  ROS_INFO("using AirSkin sensor %s with I2C address (8 Bit) %02X", name.c_str(), addr);
}

AirSkinPad::~AirSkinPad()
{
  fprintf(stderr, "destroyed pad %s\n", name.c_str());
}

bool AirSkinPad::init()
{
  bool filled = false;
  int cnt = 0;
  int maxCnt = 200;

  // initially fill the sliding mean
  while (!filled && cnt < maxCnt)
  {
    int p = sensor.ReadRawPressure();
    if (isValidPressure(p))
      filled = mean.fill(p);
    // reading too fast apparently causes communication error
    usleep(5000);
    cnt++;
  }
  if (filled)
  {
    is_activated = false;
    ROS_INFO("pad %s (addr %02X) ready", name.c_str(), addr);
  }
  else
  {
    is_activated = true;
    ROS_ERROR("failed to get mean for pad %s (addr %02X)", name.c_str(), addr);
  }

  return filled;
}

void AirSkinPad::update()
{
  p = sensor.ReadRawPressure();
  if (isValidPressure(p))
  {
    if (!is_activated)
    {
      if (p >= getReference() + ACTIVATION_THR)
      {
        NODELET_DEBUG("pad %s (addr %02X) pressed", name.c_str(), addr);
        is_activated = true;
        // freeze reference value calculation
        ref_is_frozen = true;
      }
    }
    else
    {
      if (p <= getReference() + ACTIVATION_THR - ACTIVATION_HYST)
      {
        NODELET_DEBUG("pad %s (addr %02X) released", name.c_str(), addr);
        is_activated = false;
        // trigger unfreezing reference value calculation
        ref_unfreeze_timer = ros::Time::now();
      }
    }
    if (!ref_is_frozen)
    {
      updateReference();
    }
    else
    {
      ros::Duration d = ros::Time::now() - ref_unfreeze_timer;
      if (d.toSec() > UNFREEZE_DLEAY)
      {
        // NOTE: This check is necessary in case the pad is leaking. In that case there
        // will be under-pressurs after a longer/harder press, which will slowly grow to
        // 0 again. If we update the reference during that time, the pad will activate
        // itself. Therefore wait until the under-pressure is almost0 again.
        if (p > getReference() - ACTIVATION_HYST && p < getReference())
        {
          ref_is_frozen = false;
          NODELET_DEBUG("pad %s (addr %02X) resume reference", name.c_str(), addr);
        }
      }
    }
  }
}

void AirSkinPad::setColor(std_msgs::ColorRGBA color)
{
  uint8_t red = int(color.r * 255 * color.a);
  uint8_t green = int(color.g * 255 * color.a);
  uint8_t blue = int(color.b * 255 * color.a);
  setColor(red, green, blue);
}

void AirSkinPad::setColor(uint8_t red, uint8_t green, uint8_t blue)
{
  sensor.SetColor(red, green, blue);
}
