#ifndef I2C_SLAVE_H
#define I2C_SLAVE_H

#include <tuw_i2c/I2C_Master.h>
#include <memory>

class I2C_Slave
{
protected:
  std::shared_ptr<I2C_Master> master;
  unsigned char addr;

public:
  I2C_Slave(std::shared_ptr<I2C_Master> &_master, unsigned char _addr);
  unsigned char GetAddress()
  {
    return addr;
  }
};

#endif
