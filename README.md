# Q2 v1.0 Firmware  

This project contains the source code for Q2 v1.0 firmware.
(This firmware is modifed from a version of Crazyflie firmware) 

## Dependencies

To make BIN, you'll need to install some ARM toolchain.

### Install a toolchain


#### Debian/Ubuntu

Tested on Ubuntu 14.04 64b:

```bash
sudo add-apt-repository ppa:terry.guo/gcc-arm-embedded
sudo apt-get update
sudo apt-get install libnewlib-arm-none-eabi
```


### Cloning

This repository uses git submodules. Clone with the --recursive flag

```bash
git clone --recursive https://github.com/HovakimyanResearch/CF_firmware_pathfollowing.git
```

If you already have cloned the repo, use

```bash
git submodule init
git submodule update
```


## Compiling

This is the dafault build so just running "make" is enough or:
```bash
make PLATFORM=CF2
```

or with the toolbelt

```bash
tb build
```

After you make the 'bin', you can download the file using "crazyflie client"
The Crazyflie PYthon client can be downloaded from the below.
(https://www.bitcraze.io/download/)

### config.mk
To create custom build options create a file called config.mk in the root folder 
(same as Makefile) and fill it with options. E.g. 
```
PLATFORM=CF1
DEBUG=1
CLOAD=0
```
More information can be found on the 
[Bitcraze wiki](http://wiki.bitcraze.se/projects:crazyflie2:index)

## Folder description:
```
./              | Root, contains the Makefile
 + init         | Contains the main.c
 + config       | Configuration files
 + drivers      | Hardware driver layer
 |  + src       | Drivers source code
 |  + interface | Drivers header files. Interface to the HAL layer
 + hal          | Hardware abstaction layer
 |  + src       | HAL source code
 |  + interface | HAL header files. Interface with the other parts of the program
 + modules      | Firmware operating code and headers
 |  + src       | Firmware tasks source code and main.c
 |  + interface | Operating headers. Configure the firmware environement
 + utils        | Utils code. Implement utility block like the console.
 |  + src       | Utils source code
 |  + interface | Utils header files. Interface with the other parts of the program
 + platform     | Platform specific files. Not really used yet
 + tools        | Misc. scripts for LD, OpenOCD, make, version control, ...
 |              | *** The two following folders contains the unmodified files ***
 + lib          | Libraries
 |  + FreeRTOS  | Source FreeRTOS folder. Cleaned up from the useless files
 |  + STM32...  | Library folders of the ST STM32 peripheral libs
 |  + CMSIS     | Core abstraction layer
```
# Make targets:
```
all        : Shortcut for build
compile    : Compile cflie.hex. WARNING: Do NOT update version.c
build      : Update version.c and compile cflie.elf/hex
clean_o    : Clean only the Objects files, keep the executables (ie .elf, .hex)
clean      : Clean every compiled files
mrproper   : Clean every compiled files and the classical editors backup files

cload      : If the crazyflie-clients-python is placed on the same directory level and 
             the Crazyradio/Crazyradio PA is inserted it will try to flash the firmware 
             using the wireless bootloader.
flash      : Flash .elf using OpenOCD
halt       : Halt the target using OpenOCD
reset      : Reset the target using OpenOCD
openocd    : Launch OpenOCD
```

# Unit testing

## Dependencies

Frameworks for unit testing are pulled in as git submodules.

The testing framework uses ruby and rake to generate and run code. 

To minimize the need for installations and configuration, use the docker builder
image (bitcraze/builder) that contains all tools needed. All scripts in the 
tools/build directory are intended to be run in the image. The 
[toolbelt](https://wiki.bitcraze.io/projects:dockerbuilderimage:index) makes it
easy to run the tool scripts.

### Running unit tests
    
With the environment set up locally

        rake

with the docker builder image and the toolbelt

        tb test
