/**
 * FTDI UM232H Wrapper
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <airskin_nodelet/Except.h>
#include <airskin_nodelet/I2C_Master_MPSSE.h>
#include <ros/ros.h>
#include <string.h>

I2C_Master_MPSSE::I2C_Master_MPSSE(void)
{
    context = NULL;
    context = MPSSE(I2C,ONE_HUNDRED_KHZ,MSB);
    if(context == NULL || !context->open) {
        //throw exception
        ROS_INFO("MPSSE constructor failed: %s",::ErrorString(context));
    } else {
        ROS_INFO("MPSSE Adapter: %s",context->description);
    }
}

I2C_Master_MPSSE::~I2C_Master_MPSSE()
{
    ::Close(context);
    fprintf(stderr,"closed device\n");
}


void I2C_Master_MPSSE::Write(unsigned char addr, 
    unsigned char nbytes, const unsigned char data[])
{
    throw Except(__HERE__, "Write not implemented, use WriteRegister");
}

void I2C_Master_MPSSE::WriteRegister(unsigned char addr, unsigned char reg,
    unsigned char nbytes, const unsigned char data[])
{
    char buff[nbytes+2];
    buff[0] = addr;
    buff[1] = reg;
    memcpy(&buff[2],data,nbytes);
    ::Start(context);
    ::Write(context,buff,nbytes+2);
    ::Stop(context);
    if(::GetAck(context) != ::ACK)
        throw Except(__HERE__, "command failed");
}

void I2C_Master_MPSSE::Read(unsigned char addr,
    unsigned char nbytes, unsigned char data[])
{
    throw Except(__HERE__, "Read not implemented, use ReadRegister");
}

void I2C_Master_MPSSE::ReadRegister(unsigned char addr, unsigned char reg,
    unsigned char nbytes, unsigned char data[])
{
    char txbuff[2];
    char *rxbuff = 0;
    txbuff[1] = reg;
    for(int i=0;i<nbytes;i++) {
        txbuff[0] = addr;
        ::Start(context);
        ::Write(context,txbuff,2);
        ::Stop(context);
        txbuff[0] = addr+1;
        ::Start(context);
        ::Write(context,txbuff,1);
        ::SendNacks(context);
        rxbuff = ::Read(context,1);
        ::Stop(context);    
        data[i] = rxbuff[0];
        free(rxbuff);
        txbuff[1]++;
    }
}
