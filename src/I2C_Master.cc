/**
 * Base class for I2C masters
 */

#include <stdio.h>
#include <unistd.h>
#include <airskin_nodelet/Except.h>
#include <airskin_nodelet/I2C_Master.h>

void I2C_Master::CheckedRead(int fd, unsigned char buf[], size_t len)
{
  ssize_t ret = read(fd, buf, len);
  if(ret != (ssize_t)len)
    throw Except(__HERE__, "failed to read from device");
}

void I2C_Master::CheckedWrite(int fd, const unsigned char buf[], size_t len)
{
  ssize_t written = 0;
  int retries = 10;
  while(written != (ssize_t)len && retries >= 0)
  {
    written += write(fd, buf + written, len - written);
    retries--;
  }
  if(written != (ssize_t)len)
    throw Except(__HERE__, "failed to write to device after %d retries", 10);
}

int I2C_Master::GetFirmwareVersion(void)
{
    throw Except(__HERE__, "not implemented");
    return 0;
}
