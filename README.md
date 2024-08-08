# Intel Vision Driver

This repository supports Intel Vision Driver on Intel Lunar Lake (LNL) CVS-enabled Platforms

## Dependencies
* Intel LNL platform BIOS and CVS device
* Linux kernel v6.7-rc8 or later
* Intel LJCA USB driver, adding LNL GPIO PID (INTC10B5) support


## Build instructions
Ways to build the USBIO drivers
  1. build out of kernel source tree 
  2. build with dkms
  3. build with kernel source tree aren't supported yet

Build was tested on Ubuntu 22.04 LNL CVS platform running kernel v6.7-rc8

### Build out of kernel source tree
* Prerequisite: 6.7-rc8 (or later) kernel with ```ljca``` and ```gpio_ljca``` module/driver loaded, header and build packages, to install required package
```
$ sudo apt-get install build-essential
```

To compile the Intel Vision driver:
```
$ cd vision-drivers
$ make clean
$ make
```

### Check LJCA drivers are loaded before vision driver can be installed
```
$ lsmod | grep -e ljca -e gpio_ljca     
```

### Installing the vision driver 
```
$ sudo insmod intel_cvs.ko 
```

### Uninstall the vision driver
```
$ sudo rmmod intel_cvs
```

### Build with dkms
a dkms.conf file is also provided as an example for building with dkms which can be used by ```dkms add, build and install```.



