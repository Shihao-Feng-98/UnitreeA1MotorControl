# Unitree_motor_control
Important notes: please run in ubuntu 18.04, ubuntu 20.04 and ubuntu 22.04 are abnormal in USB to RS485 communication， while causes standstill in motor control.

Some examples for Unitree motor real-time control. 

## Distinguish multiple identical device ports (ttyUSB*)
In this case, multiple serial ports haved to be used for high bandwide motor control, so distinction is required.

Plug in a USB device, and run the following line to get `devpath` infomation for distinction
```console
$ udevadm info --attribute-walk /sys/class/tty/ttyUSB0 | grep devpath
```

`idVendor` and `idProduct` infomation also needed, run
```console
$ udevadm info --attribute-walk /sys/class/tty/ttyUSB0 | grep idVendor
$ udevadm info --attribute-walk /sys/class/tty/ttyUSB0 | grep idProduct
```

Then build udev file
```console
$ sudo vim /etc/udev/rules.d/unitree_usb.rules
```

and write 
```
KERNEL=="ttyUSB*", ATTRS{devpath}=="1.1", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", MODE:="0777",SYMLINK+="unitree_usb0"
KERNEL=="ttyUSB*", ATTRS{devpath}=="1.2", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", MODE:="0777",SYMLINK+="unitree_usb1"
KERNEL=="ttyUSB*", ATTRS{devpath}=="1.3", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", MODE:="0777",SYMLINK+="unitree_usb2"
KERNEL=="ttyUSB*", ATTRS{devpath}=="1.4", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", MODE:="0777",SYMLINK+="unitree_usb3"
```

After rebbot, check the serial port name
```console
$ ls -l /dev | grep ttyUSB
crwxrwxrwx   1 root        dialout 188,     0 11月  4 15:04 ttyUSB0
lrwxrwxrwx   1 root        root             7 11月  4 15:04 unitree_usb0 -> ttyUSB0
```
