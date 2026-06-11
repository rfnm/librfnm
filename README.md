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

# Ethernet streaming performance

The ethernet (TCP) transport works on any network with no special setup: the
device and host negotiate the usable packet size automatically (TCP MSS), so an
ordinary MTU-1500 host streams correctly out of the box, topping out near
**~77 Msps** on a 2.5 Gbps link.

For the full rate (up to **~90 Msps** sustained on 2.5 Gbps), enable jumbo
frames on the host NIC that connects to the device, e.g. on Linux:

```
sudo ip link set <interface> mtu 8000
```

(Windows: adapter properties -> Jumbo Packet; macOS: Hardware -> MTU.) Every
switch between the host and the device must also forward jumbo frames — if any
hop silently drops them, set the host NIC back to MTU 1500. The device side
already defaults to MTU 8000 on USB ethernet adapters.

librfnm logs the negotiated MSS when it connects — `jumbo frames active`
confirms the fast path; `jumbo frames inactive` means the host NIC or the path
is limiting the rate.
