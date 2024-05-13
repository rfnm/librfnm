# librfnm

- Very early code, kinda stable APIs.
- Can talk to RFNM hardware (the linux kernel drivers inside the RFNM device).
- Can send/receive data and configure the device.
- Only works via USB today, local transport will be enabled next (librfnm will run in userspace inside the RFNM device).
- Can only do recieve well today, transmit is still a work in progress (DMA issues over PCIe, nothing to do with this code).