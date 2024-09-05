# RFNM Hardware Driver ![CI](https://github.com/rfnm/librfnm/actions/workflows/build_all.yml/badge.svg)

This repository contains the code for the driver code in charge of communicating with the RFNM hardware.
It is important to note that **this driver is a work in progress.**

### Implemented Features

- Talk to RFNM hardware (the linux kernel drivers inside the RFNM device).
- Send/receive data and configure the device.

### Current Limitations

- Very early code, **THE API DEFINITION IS NOT YET STABLE**.
- Only works via USB.
- Can only do recieve properly, transmit is still a work in progress (DMA issues over PCIe, nothing to do with this code).

### Future Features

- Local transport (librfnm will run in userspace inside the RFNM device).

# Installation

The installation steps depend on the operating system.

## Windows

First, start by installing the dependencies using vcpkg. For this, make sure you have both visual studio, vcpkg and cmake installed.

```
vcpkg install libusb:x64-windows spdlog:x64-windows
```

Next, cd to a working directory of your chosing and grab the latest driver source code and cd into it. For this you can either download a zip of the code and extract it or use git:

```
git clone https://github.com/rfnm/librfnm
cd librfnm
```

Then, create a build directory and cd into it

```
mkdir build ; cd build
```

Next, prepare the cmake build files. Make sure to change the vcpkg path to that of vcpkg on your system.

```
cmake .. "-DCMAKE_TOOLCHAIN_FILE=C:/path/to/vcpkg/scripts/buildsystems/vcpkg.cmake"
```

Then, build the library

```
cmake --build . --config Release
```

Finally, the library can be installed using

```
cmake --install .
```

It should then be available at `C:/Program Files/RFNM`.

## Linux and MacOS

For linux, start by installing the dependencies using your package manager. You will need `libusb` and `spdlog`. The name of these packages will depend on your exact distribution.

Next, cd to a working directory of your chosing and grab the latest driver source code and cd into it. For this you can either download a zip of the code and extract it or use git:

```
git clone https://github.com/rfnm/librfnm
cd librfnm
```

Then, create a build directory and cd into it

```
mkdir build && cd build
```

Next, prepare the cmake build files.

```
cmake ..
```

Then, build the library

```
make
```

Finally, the library can be installed using

```
sudo make install
```
