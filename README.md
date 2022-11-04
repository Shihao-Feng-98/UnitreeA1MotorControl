# Unitree_motor_control
Some examples for Unitree motor real-time control. 

## Distinguish multiple identical device ports (ttyUSB*)
In this case, multiple serial ports haved to be used for high bandwide motor control, so distinction is required.

Plug in a USB device, and run the following line to get `devpath` infomation for distinction
```bash
udevadm info --attribute-walk /sys/class/tty/ttyUSB0 | grep devpath
```
