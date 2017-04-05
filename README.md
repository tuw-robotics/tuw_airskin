## UM232H-R Requirements:
Download, compile and install libmpsse
* https://github.com/devttys0/libmpsse.git


Create udev rule '/etc/udev/rules.d/99-ftdi.rules' with the following content:
```
ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", MODE="0666", GROUP="dialout"
```

## Configuration
The configuration is set up in tuw_airskin/cfg/airskin.yaml:
The used device (in case of multiple UM232H the first free device found is used)
The amount, addresses and names of AirSkin pads.
