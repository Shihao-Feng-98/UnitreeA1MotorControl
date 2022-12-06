# Introduction
This library is used for the communication between PC and the motor control board, so that the user can control the motors by PC. This library only includes Linux version, and we offer the usage example of C++.

# Dependencies
* [CMake](http://www.cmake.org) (version 2.8.3 or higher)
  
# Files
## /lib
Including the library files of Linux. Please modify the CMakeList manually to select the correst `.so` file.
## /include
Including the head files.
## /src
Including the source files of example and changeID tool. The example can control motors to run under desired command for desired time, and then stop.
## /build
Build directory. The final executable files are also in this directory.

# Usage
## Build
```bash
mkdir build
cd build
cmake ..
make
```
## Run
```bash
cd build
sudo ./test_motor
```
## Change ID
```bash
cd build
sudo ./changeID
```
