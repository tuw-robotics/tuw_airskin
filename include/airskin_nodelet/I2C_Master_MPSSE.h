/**
 * FTDI UM232H Wrapper
 * (FT232H)
 *
 * @author Klaus Buchegger, klaus.buchegger@tuwien.ac.at
 */

#ifndef I2C_MASTER_MPSSE_H
#define I2C_MASTER_MPSSE_H

#include <string>
extern "C" {
#include <mpsse.h>
}
#include <airskin_nodelet/I2C_Master.h>

class I2C_Master_MPSSE : public I2C_Master
{
private:
  struct mpsse_context *context;

  /**
   * Read single ACK byte, used after sending a command.
   */
  bool ReadACK();

public:
  I2C_Master_MPSSE(void);

  virtual ~I2C_Master_MPSSE();

  virtual void Write(unsigned char addr, unsigned char nbytes, const unsigned char data[]);

  virtual void WriteRegister(unsigned char addr, unsigned char reg, unsigned char nbytes, const unsigned char data[]);

  virtual void Read(unsigned char addr, unsigned char nbytes, unsigned char data[]);

  virtual void ReadRegister(unsigned char addr, unsigned char reg, unsigned char nbytes, unsigned char data[]);
};

#endif
