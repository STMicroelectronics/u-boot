.. SPDX-License-Identifier: GPL-2.0-or-later
.. sectionauthor:: Patrice Chotard <patrice.chotard@foss.st.com>

STM32MP2xx boards
=================

This is a quick instruction for setup STMicroelectronics STM32MP2xx boards.

Further information can be found in STMicroelectronics STM32 WIKI_.

Supported devices
-----------------

U-Boot supports all the STMicroelectronics MPU with the associated boards

 - STMP32MP25x SoCs:

  - STM32MP257
  - STM32MP253
  - STM32MP251

Everything is supported in Linux but U-Boot is limited to the boot device:

 1. UART
 2. SD card/MMC controller (SDMMC)
 3. NAND controller (FMC)
 4. NOR controller (OSPI)
 5. HyperFlash controller (OSPI)
 6. USB controller (USB_STM32_USBH)
 7. Ethernet controller

And the necessary drivers

 1. I2C
 2. Clock, Reset, Sysreset
 3. Fuse (BSEC)
 4. OP-TEE
 5. ETH
 6. USB host
 7. WATCHDOG
 8. RNG
 9. RTC

STM32MP25x
``````````
The STM32MP25x is a Cortex-A35 MPU aimed at various applications.

It features:

 - Dual core Cortex-A35 application core (Single on STM32MP251)
 - 2D/3D image composition with GPU (only on STM32MP255 and STM32MP257)
 - Standard memories interface support
 - Standard connectivity, widely inherited from the STM32 MCU family
 - Comprehensive security support
 - Cortex M33 coprocessor

Each line comes with a security option (cryptography & secure boot) and
a Cortex-A frequency option:

 - A : Cortex-A35 @ 1.2 GHz
 - C : Secure Boot + HW Crypto + Cortex-A35 @ 1.2 GHz
 - D : Cortex-A35 @ 1.5 GHz
 - F : Secure Boot + HW Crypto + Cortex-A35 @ 1.5 GHz

Currently the following STMicroelectronics boards are supported:

 + stm32mp257f-dk.dts
 + stm32mp257f-ev1.dts

The access to reset and clock resources are provided by OP-TEE and the associated SCMI
services.

Boot Sequences
--------------

The boot chain is :

- FSBL = **TF-A BL2**
- Secure monitor = **TF-A BL31**
- Secure OS = **OP-TEE**
- SSBL = **U-Boot**

It is the only supported boot chain for STM32MP25x family.

defconfig_file :
   + **stm32mp25_defconfig**

TF-A_ and OP-TEE_ are 2 separate projects, with their git repository;
they are compiled separately.

TF-A_ (BL2) initialize the DDR and loads the next stage binaries from a FIP file:
   + BL32: a secure monitor BL32 = OP-TEE, performs a full initialization of
     Secure peripherals and provides service to normal world.
   + BL33: a non-trusted firmware = U-Boot, running in normal world and uses
     the secure monitor to access to secure resources.
   + HW_CONFIG: The hardware configuration file = the U-Boot device tree

Device Tree
-----------

All the STM32MP25x boards supported by U-Boot use the same generic board stm32mp2
which supports all the bootable devices.

Each STMicroelectronics board is only configured with the associated device tree.

STM32MP25x device Tree Selection
````````````````````````````````
The supported device trees for STM32MP25x are:

+ ev1: Evaluation board

   + stm32mp257f-ev1

+ dk: Discovery board

   + stm32mp257f-dk

Build Procedure
---------------

1. Install the required tools for U-Boot

  * install package needed in U-Boot makefile
    (libssl-dev, swig, libpython-dev...)

  * install ARMv8 toolchain for 64bit Cortex-A (from Linaro,
    from SDK for STM32MP25x, or any crosstoolchains from your distribution)
    (you can use any gcc cross compiler compatible with U-Boot)

2. Set the cross compiler::

   # export CROSS_COMPILE=/path/to/toolchain/arm-linux-gnueabi-

3. Select the output directory (optional)::

   # export KBUILD_OUTPUT=/path/to/output

   for example: use one output directory for each board::

   # export KBUILD_OUTPUT=stm32mp257f-ev1
   # export KBUILD_OUTPUT=stm32mp257f-dk

   you can build outside of code directory::

   # export KBUILD_OUTPUT=../build/stm32mp25

4. Configure U-Boot::

   # make <defconfig_file>

   with <defconfig_file>: stm32mp25_defconfig

5. Configure the device-tree and build the U-Boot image::

   # make DEVICE_TREE=<name> all

 6. U-Boot Output files

   In the output directory (selected by KBUILD_OUTPUT),
   you can found the needed U-Boot files:

     - stm32mp25_defconfig = **u-boot-nodtb.bin** and **u-boot.dtb**

7. TF-A_ compilation

   see OP-TEE_ and TF-A_ documentation for build commands.

   - compile OP-TEE to generate the binary included in FIP

   - after TF-A compilation, the used files are:

     - TF-A_ BL2 => FSBL = **tf-a.stm32**

     - FIP => **fip.bin**

       FIP file includes the 2 files given in arguments of TF-A_ compilation:

      - BL33=u-boot-nodtb.bin
      - BL33_CFG=u-boot.dtb

     You can also update an existing FIP after U-boot compilation with fiptool,
     a tool provided by TF-A_::

     # fiptool update --nt-fw u-boot-nodtb.bin --hw-config u-boot.dtb fip-stm32mp157c-ev1.bin

8. The bootloaders files

+ The **ROM code** expects FSBL binaries with STM32 image header = tf-a.stm32

According the FSBL / the boot mode:

+ **TF-A** expect a FIP binary = fip.bin, including the OS monitor (OP-TEE_) and the
  U-Boot binary + device tree

Switch Setting for Boot Mode
----------------------------

You can select the boot mode, on the board with one switch, to select
the boot pin values = BOOT0, BOOT1, BOOT2, BOOT3

  +-------------+---------+---------+---------+---------+
  |*Boot Mode*  | *BOOT3* | *BOOT2* | *BOOT1* | *BOOT0* |
  +=============+=========+=========+=========+=========+
  | Recovery    |    0    |    0    |    0    |    0    |
  +-------------+---------+---------+---------+---------+
  | SD-Card     |    0    |    0    |    0    |    1    |
  +-------------+---------+---------+---------+---------+
  | eMMC        |    0    |    0    |    1    |    0    |
  +-------------+---------+---------+---------+---------+
  | Reserved    |    0    |    0    |    1    |    1    |
  +-------------+---------+---------+---------+---------+
  | SPI-NOR     |    0    |    1    |    0    |    0    |
  +-------------+---------+---------+---------+---------+
  | Reserved    |    0    |    1    |    0    |    1    |
  +-------------+---------+---------+---------+---------+
  | Reserved    |    0    |    1    |    1    |    0    |
  +-------------+---------+---------+---------+---------+
  | Reserved    |    0    |    1    |    1    |    1    |
  +-------------+---------+---------+---------+---------+
  | Reserved    |    1    |    0    |    0    |    0    |
  +-------------+---------+---------+---------+---------+
  | Reserved    |    1    |    0    |    0    |    1    |
  +-------------+---------+---------+---------+---------+
  | Reserved    |    1    |    0    |    1    |    0    |
  +-------------+---------+---------+---------+---------+
  | Reserved    |    1    |    0    |    1    |    1    |
  +-------------+---------+---------+---------+---------+
  | Reserved    |    1    |    1    |    0    |    0    |
  +-------------+---------+---------+---------+---------+
  | Reserved    |    1    |    1    |    0    |    1    |
  +-------------+---------+---------+---------+---------+
  | Reserved    |    1    |    1    |    1    |    0    |
  +-------------+---------+---------+---------+---------+
  | Recovery    |    1    |    1    |    1    |    1    |
  +-------------+---------+---------+---------+---------+

- on the STM32MP25x **evaluation board ev1 = MB1936** with the switch SW1
  (BOOT0, BOOT1, BOOT2, BOOT3)
- on the STM32MP25x **discovery board dk = MB1605** with the switch SW1
  (BOOT0, BOOT1 only)

Recovery is a boot from serial link (UART/USB) and it is used with
STM32CubeProgrammer tool to load executable in RAM and to update the flash
devices available on the board (HyperFlash/NOR/NAND/eMMC/SD card).

The communication between HOST and board is based on

  - for UARTs : the uart protocol used with all MCU STM32
  - for USB : based on USB DFU 1.1 (without the ST extensions used on MCU STM32)

Prepare a SD card
-----------------

The minimal requirements for STMP32MP25x boot up to U-Boot are:

- GPT partitioning (with gdisk or with sgdisk)
- 2 fsbl partitions, named "fsbla1" and "fsbla2", size at least 256KiB (2 copies for
  redundancy)
- 2 metadata partitions for FIP update support, named "metadata1" and "metadata2",
  size at least 256KiB (2 copies for redundancy)
- 2 fip partitions named "fip-a" and "fip-b" for FIP binary
- 1 environment partition named "u-boot-env"

The 2 fsbl partitions have the same content and are present to guarantee a
fail-safe update of FSBL; fsbl2 can be omitted if this ROM code feature is
not required.

Then the minimal GPT partition is:

  +-------+------------+---------+------------------------+
  | *Num* |   *Name*   | *Size*  | *Content*              |
  +=======+============+=========+========================+
  | 1     |   fsbla1   | 256 KiB | TF-A_ BL2 (tf-a.stm32) |
  +-------+------------+---------+------------------------+
  | 2     |   fsbla2   | 256 KiB | TF-A_ BL2 (tf-a.stm32) |
  +-------+------------+---------+------------------------+
  | 3     | metadata1  | 256 KiB |                        |
  +-------+------------+---------+------------------------+
  | 4     | metadata1  | 256 KiB |                        |
  +-------+------------+---------+------------------------+
  | 5     |   fip-a    |   4 MiB |       fip.bin          |
  +-------+------------+---------+------------------------+
  | 6     |   fip-b    |   4 MiB |       fip.bin          |
  +-------+------------+---------+------------------------+
  | 7     | u-boot-env |  512 KiB|                        |
  +-------+------------+---------+------------------------+
  | 8     |   <any>    |  <any>  | Rootfs                 |
  +-------+------------+---------+------------------------+

And the 8th partition (Rootfs) is marked bootable with a file extlinux.conf
following the Generic Distribution feature (see :doc:`../../develop/distro` for
use).

The size of fip partition must be enough for the associated binary file,
4MB is the default value.

According the used card reader select the correct block device
(for example /dev/sdx or /dev/mmcblk0), in the next example, it is /dev/mmcblk0

For example:

a) remove previous formatting::

    # sgdisk -o /dev/<SD card dev>

b) create minimal image for FIP

   For FIP support in TF-A_::

    # sgdisk --resize-table=128 -a 1 \
    -n 1:34:545	                -c 1:fsbla1 \
    -n 2:546:1057		-c 2:fsbla2 \
    -n 3:1058:1569              -c 3:metadata1 \
    -n 4:1570:2081		-c 4:metadata2 \
    -n 5:2082:10273		-c 5:fip-a \
    -n 6:10274:18465		-c 6:fip-b \
    -n 7:18466:19489		-c 7:u-boot-env \
    -n 8:19490:			-c 8:rootfs -A 4:set:2 \
    -p /dev/<SD card dev>

   With gpt table with 128 entries an the partition 4 marked bootable (bit 2).

c) copy the FSBL (2 times) and SSBL file on the correct partition.
   in this example in partition 1 to 6::

    # dd if=tf-a.stm32 of=/dev/mmcblk0p1
    # dd if=tf-a.stm32 of=/dev/mmcblk0p2
    # dd if=fip.bin of=/dev/mmcblk0p5
    # dd if=fip.bin of=/dev/mmcblk0p6

To boot from SD card, select BootPinMode = 1 0 0 0 and reset.

Prepare eMMC
------------

You can use U-Boot to copy binary in eMMC.

In the next example, you need to boot from SD card and the images
(tf-a.stm32, metadata, fip.bin, u-boot.img are presents on SD card
(mmc 0) in ext4 partition 4 (bootfs)

To boot from SD card, select BootPinMode = 1 0 0 0 and reset.

Then you update the eMMC with the next U-Boot command :

a) prepare GPT on eMMC,
   example with 3 partitions, fip, bootfs and roots::

    # setenv emmc_part "name==metadata1,size=256KiB;name=metada2,size=256KiB;name=fip-a,size=4MiB;name=fip-b,size=4MiB;name=bootfs,type=linux,bootable,size=64MiB;name=rootfs,type=linux,size=512"
    # gpt write mmc 1 ${emmc_part}

b) copy FSBL ( TF-A_ ) in first eMMC boot partition::

    # ext4load mmc 0:4 0xC0000000 tf-a.stm32

    # mmc dev 1
    # mmc partconf 1 1 1 1
    # mmc write ${fileaddr} 0 200
    # mmc partconf 1 1 1 0

c) copy SSBL ( FIP ) in fip-a GPT partition of eMMC::

    # ext4load mmc 0:4 0xC0000000 fip.bin

    # mmc dev 1
    # part start mmc 1 fip-a partstart
    # mmc write ${fileaddr} ${partstart} ${filesize}

To boot from eMMC, select BootPinMode = 0 0 1 0 and reset.

Coprocessor firmware on STM32MP25x
----------------------------------

U-Boot can boot the coprocessor before the kernel (coprocessor early boot) by using rproc commands (update the bootcmd)

   Configurations::

	# env set name_copro "rproc-m33-fw_sign.bin"
	# env set dev_copro 0
	# env set loadaddr_copro ${kernel_addr_r}

   Load binary from bootfs partition on SD card (mmc 0)::

	# load mmc 0#bootfs ${loadaddr_copro} ${name_copro}

   => ${filesize} variable is updated with the size of the loaded file.

   Start M33 firmware with remote proc command::

	# rproc init
	# rproc load ${dev_copro} ${loadaddr_copro} ${filesize}
	# rproc start ${dev_copro}"0

DFU support
-----------

The DFU is supported on ST board.

The env variable dfu_alt_info is automatically build, and all
the memory present on the ST boards are exported.

The dfu mode is started by the command::

  STM32MP> dfu 0

On EV1 board, booting from SD card::

  STM32MP> dfu 0 list
  DFU alt settings list:

dev: RAM alt: 0 name: uImage layout: RAM_ADDR
dev: RAM alt: 1 name: devicetree.dtb layout: RAM_ADDR
dev: RAM alt: 2 name: uramdisk.image.gz layout: RAM_ADDR
dev: eMMC alt: 3 name: mmc0_fsbla1 layout: RAW_ADDR
dev: eMMC alt: 4 name: mmc0_fsbla2 layout: RAW_ADDR
dev: eMMC alt: 5 name: mmc0_metadata1 layout: RAW_ADDR
dev: eMMC alt: 6 name: mmc0_metadata2 layout: RAW_ADDR
dev: eMMC alt: 7 name: mmc0_fip-a layout: RAW_ADDR
dev: eMMC alt: 8 name: mmc0_fip-b layout: RAW_ADDR
dev: eMMC alt: 9 name: mmc0_u-boot-env layout: RAW_ADDR
dev: eMMC alt: 10 name: mmc0_bootfs layout: RAW_ADDR
dev: eMMC alt: 11 name: mmc0_vendorfs layout: RAW_ADDR
dev: eMMC alt: 12 name: mmc0_rootfs layout: RAW_ADDR
dev: eMMC alt: 13 name: mmc0_userfs layout: RAW_ADDR
dev: eMMC alt: 14 name: mmc1_boot1 layout: RAW_ADDR
dev: eMMC alt: 15 name: mmc1_boot2 layout: RAW_ADDR
dev: eMMC alt: 16 name: mmc1_fip layout: RAW_ADDR
dev: eMMC alt: 17 name: mmc1_bootfs layout: RAW_ADDR
dev: eMMC alt: 18 name: mmc1_vendorfs layout: RAW_ADDR
dev: eMMC alt: 19 name: mmc1_rootfs layout: RAW_ADDR
dev: eMMC alt: 20 name: mmc1_test_report layout: RAW_ADDR
dev: eMMC alt: 21 name: mmc1_backup layout: RAW_ADDR
dev: MTD alt: 22 name: nor1 layout: RAW_ADDR
dev: MTD alt: 23 name: nor1_fsbla1 layout: RAW_ADDR
dev: MTD alt: 24 name: nor1_fsbla2 layout: RAW_ADDR
dev: MTD alt: 25 name: nor1_metadata1 layout: RAW_ADDR
dev: MTD alt: 26 name: nor1_metadata2 layout: RAW_ADDR
dev: MTD alt: 27 name: nor1_fip-a layout: RAW_ADDR
dev: MTD alt: 28 name: nor1_fip-b layout: RAW_ADDR
dev: MTD alt: 29 name: nor1_u-boot-env layout: RAW_ADDR
dev: MTD alt: 30 name: nor1_nor-user layout: RAW_ADDR
dev: VIRT alt: 31 name: OTP layout: RAW_ADDR

All the supported device are exported for dfu-util tool::

  $> dfu-util -l

  Found DFU: [0483:df11] ver=0200, devnum=21, cfg=1, intf=0, path="3-6.1", alt=31, name="OTP", serial="004A00253836500B00343046"
  Found DFU: [0483:df11] ver=0200, devnum=21, cfg=1, intf=0, path="3-6.1", alt=30, name="nor1_nor-user", serial="004A00253836500B00343046"
  Found DFU: [0483:df11] ver=0200, devnum=21, cfg=1, intf=0, path="3-6.1", alt=29, name="nor1_u-boot-env", serial="004A00253836500B00343046"
  Found DFU: [0483:df11] ver=0200, devnum=21, cfg=1, intf=0, path="3-6.1", alt=28, name="nor1_fip-b", serial="004A00253836500B00343046"
  Found DFU: [0483:df11] ver=0200, devnum=21, cfg=1, intf=0, path="3-6.1", alt=27, name="nor1_fip-a", serial="004A00253836500B00343046"
  Found DFU: [0483:df11] ver=0200, devnum=21, cfg=1, intf=0, path="3-6.1", alt=26, name="nor1_metadata2", serial="004A00253836500B00343046"
  Found DFU: [0483:df11] ver=0200, devnum=21, cfg=1, intf=0, path="3-6.1", alt=25, name="nor1_metadata1", serial="004A00253836500B00343046"
  Found DFU: [0483:df11] ver=0200, devnum=21, cfg=1, intf=0, path="3-6.1", alt=24, name="nor1_fsbla2", serial="004A00253836500B00343046"
  Found DFU: [0483:df11] ver=0200, devnum=21, cfg=1, intf=0, path="3-6.1", alt=23, name="nor1_fsbla1", serial="004A00253836500B00343046"
  Found DFU: [0483:df11] ver=0200, devnum=21, cfg=1, intf=0, path="3-6.1", alt=22, name="nor1", serial="004A00253836500B00343046"
  Found DFU: [0483:df11] ver=0200, devnum=21, cfg=1, intf=0, path="3-6.1", alt=21, name="mmc1_backup", serial="004A00253836500B00343046"
  Found DFU: [0483:df11] ver=0200, devnum=21, cfg=1, intf=0, path="3-6.1", alt=20, name="mmc1_test_report", serial="004A00253836500B00343046"
  Found DFU: [0483:df11] ver=0200, devnum=21, cfg=1, intf=0, path="3-6.1", alt=19, name="mmc1_rootfs", serial="004A00253836500B00343046"
  Found DFU: [0483:df11] ver=0200, devnum=21, cfg=1, intf=0, path="3-6.1", alt=18, name="mmc1_vendorfs", serial="004A00253836500B00343046"
  Found DFU: [0483:df11] ver=0200, devnum=21, cfg=1, intf=0, path="3-6.1", alt=17, name="mmc1_bootfs", serial="004A00253836500B00343046"
  Found DFU: [0483:df11] ver=0200, devnum=21, cfg=1, intf=0, path="3-6.1", alt=16, name="mmc1_fip", serial="004A00253836500B00343046"
  Found DFU: [0483:df11] ver=0200, devnum=21, cfg=1, intf=0, path="3-6.1", alt=15, name="mmc1_boot2", serial="004A00253836500B00343046"
  Found DFU: [0483:df11] ver=0200, devnum=21, cfg=1, intf=0, path="3-6.1", alt=14, name="mmc1_boot1", serial="004A00253836500B00343046"
  Found DFU: [0483:df11] ver=0200, devnum=21, cfg=1, intf=0, path="3-6.1", alt=13, name="mmc0_userfs", serial="004A00253836500B00343046"
  Found DFU: [0483:df11] ver=0200, devnum=21, cfg=1, intf=0, path="3-6.1", alt=12, name="mmc0_rootfs", serial="004A00253836500B00343046"
  Found DFU: [0483:df11] ver=0200, devnum=21, cfg=1, intf=0, path="3-6.1", alt=11, name="mmc0_vendorfs", serial="004A00253836500B00343046"
  Found DFU: [0483:df11] ver=0200, devnum=21, cfg=1, intf=0, path="3-6.1", alt=10, name="mmc0_bootfs", serial="004A00253836500B00343046"
  Found DFU: [0483:df11] ver=0200, devnum=21, cfg=1, intf=0, path="3-6.1", alt=9, name="mmc0_u-boot-env", serial="004A00253836500B00343046"
  Found DFU: [0483:df11] ver=0200, devnum=21, cfg=1, intf=0, path="3-6.1", alt=8, name="mmc0_fip-b", serial="004A00253836500B00343046"
  Found DFU: [0483:df11] ver=0200, devnum=21, cfg=1, intf=0, path="3-6.1", alt=7, name="mmc0_fip-a", serial="004A00253836500B00343046"
  Found DFU: [0483:df11] ver=0200, devnum=21, cfg=1, intf=0, path="3-6.1", alt=6, name="mmc0_metadata2", serial="004A00253836500B00343046"
  Found DFU: [0483:df11] ver=0200, devnum=21, cfg=1, intf=0, path="3-6.1", alt=5, name="mmc0_metadata1", serial="004A00253836500B00343046"
  Found DFU: [0483:df11] ver=0200, devnum=21, cfg=1, intf=0, path="3-6.1", alt=4, name="mmc0_fsbla2", serial="004A00253836500B00343046"
  Found DFU: [0483:df11] ver=0200, devnum=21, cfg=1, intf=0, path="3-6.1", alt=3, name="mmc0_fsbla1", serial="004A00253836500B00343046"
  Found DFU: [0483:df11] ver=0200, devnum=21, cfg=1, intf=0, path="3-6.1", alt=2, name="uramdisk.image.gz", serial="004A00253836500B00343046"
  Found DFU: [0483:df11] ver=0200, devnum=21, cfg=1, intf=0, path="3-6.1", alt=1, name="devicetree.dtb", serial="004A00253836500B00343046"
  Found DFU: [0483:df11] ver=0200, devnum=21, cfg=1, intf=0, path="3-6.1", alt=0, name="uImage", serial="004A00253836500B00343046"

You can update the boot device:

- SD card (mmc0)::

  $> dfu-util -d 0483:df11 -a 3 -D tf-a-stm32mp257f-ev1.stm32
  $> dfu-util -d 0483:df11 -a 4 -D tf-a-stm32mp257f-ev1.stm32
  $> dfu-util -d 0483:df11 -a 5 -D fip-stm32mp257f-ev1.bin
  $> dfu-util -d 0483:df11 -a 10 -D st-image-bootfs-openstlinux-weston-stm32mp2.ext4
  $> dfu-util -d 0483:df11 -a 11 -D st-image-vendorfs-openstlinux-weston-stm32mp2.ext4
  $> dfu-util -d 0483:df11 -a 12 -D st-image-weston-openstlinux-weston-stm32mp2.ext4
  $> dfu-util -d 0483:df11 -a 13 -D st-image-userfs-openstlinux-weston-stm32mp2.ext4

- EMMC (mmc1)::

  $> dfu-util -d 0483:df11 -a 14 -D tf-a-stm32mp257f-ev1.stm32
  $> dfu-util -d 0483:df11 -a 15 -D tf-a-stm32mp257f-ev1.stm32
  $> dfu-util -d 0483:df11 -a 16 -D fip-stm32mp257f-ev1.bin
  $> dfu-util -d 0483:df11 -a 17 -D st-image-bootfs-openstlinux-weston-stm32mp2.ext4
  $> dfu-util -d 0483:df11 -a 18 -D st-image-vendorfs-openstlinux-weston-stm32mp2.ext4
  $> dfu-util -d 0483:df11 -a 19 -D st-image-weston-openstlinux-weston-stm32mp2.ext4
  $> dfu-util -d 0483:df11 -a 20 -D st-image-userfs-openstlinux-weston-stm32mp2.ext4

- you can also dump the OTP and the PMIC NVM with::

  $> dfu-util -d 0483:df11 -a 31 -U otp.bin

When the board is booting for nor1, only the MTD partition on the boot devices are available, for example:

- NOR (nor1 = alt 15) :

  $> dfu-util -d 0483:df11 -a 16 -D tf-a-stm32mp157c-ev1.stm32
  $> dfu-util -d 0483:df11 -a 17 -D tf-a-stm32mp157c-ev1.stm32
  $> dfu-util -d 0483:df11 -a 20 -D fip-stm32mp157c-ev1.bin
  $> dfu-util -d 0483:df11 -a 23 -D st-image-weston-openstlinux-weston-stm32mp1_nand_4_256_multivolume.ubi

References
----------

.. _WIKI:

STM32 Arm® Cortex®-based MPUs user guide

  + https://wiki.st.com/
  + https://wiki.st.com/stm32mpu/wiki/Main_Page

.. _TF-A:

TF-A = The Trusted Firmware-A project provides a reference implementation of
secure world software for Armv7-A and Armv8-A class processors

  + https://www.trustedfirmware.org/projects/tf-a/
  + https://trustedfirmware-a.readthedocs.io/en/latest/
  + https://trustedfirmware-a.readthedocs.io/en/latest/plat/stm32mp2.html
  + https://git.trustedfirmware.org/TF-A/trusted-firmware-a.git/

.. _OP-TEE:

OP-TEE = an open source Trusted Execution Environment (TEE) implementing the
Arm TrustZone technology

  + https://www.op-tee.org/
  + https://optee.readthedocs.io/en/latest/
  + https://optee.readthedocs.io/en/latest/building/devices/stm32mp2.html
  + https://github.com/OP-TEE/optee_os
