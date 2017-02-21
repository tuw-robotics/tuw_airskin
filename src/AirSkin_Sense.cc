/*
 * Driver for the AirSkin prototype, version Sept 2014.
 */

#include <cstdio>
#include <unistd.h>
#include <airskin_nodelet/AirSkin_Sense.h>
#include <airskin_nodelet/AirSkinPad.h>
#include <nodelet/nodelet.h>

AirSkin_Sense::AirSkin_Sense(I2C_Master *_master, unsigned char _addr)
  : I2C_Slave(_master, _addr)
{
}

/**
 * Read the raw atmospheric pressure value.
 */
int AirSkin_Sense::ReadRawPressure()
{
  unsigned char b[4];
  master->ReadRegister(addr, 0x12, 4, b);
  uint32_t p = (((uint32_t)b[3] << 24)) + (((uint32_t)b[2]) << 16) + (((uint32_t)b[1]) << 8) + (uint32_t)b[0];
  return (int)p;
}

/**
 * Read the filtered pressure value, with sliding mean as reference.
 */
int AirSkin_Sense::ReadFilteredPressure()
{
  unsigned char b[4];
  master->ReadRegister(addr, 0x02, 4, b);
  uint32_t p = (((uint32_t)b[3] << 24)) + (((uint32_t)b[2]) << 16) + (((uint32_t)b[1]) << 8) + (uint32_t)b[0];
  return (int)p;
}


void AirSkin_Sense::SetColor(unsigned char red, unsigned char green, unsigned char blue)
{
    unsigned char b[1];
    
    using namespace ros::this_node;
    b[0] = red;
    master->WriteRegister(addr,0x22,1,b);
    b[0] = green;
    master->WriteRegister(addr,0x23,1,b);
    b[0] = blue;
    master->WriteRegister(addr,0x24,1,b);
}

