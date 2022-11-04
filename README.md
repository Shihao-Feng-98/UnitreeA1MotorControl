# Unitree_motor_control
Some examples for Unitree motor real-time control. 

## Distinguish multiple identical device ports (ttyUSB*)
In this case, multiple serial ports haved to be used for high bandwide motor control, so distinction is required.

Plug in a USB device, and run the following line to get `devpath` infomation for distinction
```
udevadm info --attribute-walk /sys/class/tty/ttyUSB0 | grep devpath
```

`idVendor` and `idProduct` infomation also needed, run
```
udevadm info --attribute-walk /sys/class/tty/ttyUSB0 | grep idVendor
udevadm info --attribute-walk /sys/class/tty/ttyUSB0 | grep idProduct
```

Then build udev file
```
sudo vim /etc/udev/rules.d/unitree_usb.rules
```

and write
```
KERNEL=="ttyUSB*", ATTRS{devpath}=="1.1", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", MODE:="0777",SYMLINK+="unitree_usb0"
KERNEL=="ttyUSB*", ATTRS{devpath}=="1.2", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", MODE:="0777",SYMLINK+="unitree_usb1"
KERNEL=="ttyUSB*", ATTRS{devpath}=="1.3", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", MODE:="0777",SYMLINK+="unitree_usb2"
KERNEL=="ttyUSB*", ATTRS{devpath}=="1.4", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", MODE:="0777",SYMLINK+="unitree_usb3"
```

