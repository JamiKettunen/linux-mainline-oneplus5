# REPO DEPRECATION
This repository is deprecated in favor of [msm8998-mainline/linux on GitLab](https://gitlab.com/msm8998-mainline/linux) where development has continued where this tree left off :)

# linux-mainline-oneplus5
Linux mainline kernel fork focused on OnePlus 5 (cheeseburger) / 5T (dumpling) devices.

## Table of Contents
* [linux-mainline-oneplus5](#linux-mainline-oneplus5)
	* [Building](#building)
		* [Dependencies](#dependencies)
		* [Kernel and modules](#kernel-and-modules)
		* [Boot image](#boot-image)
		* [Updating an existing boot.img](#updating-an-existing-bootimg)
	* [Manual deployment](#manual-deployment)
		* [Firmware](#firmware)
		* [Kernel modules](#kernel-modules)
	* [What works?](#what-works)
	* [What doesn't work?](#what-doesnt-work)
	* [Other issues](#other-issues)
	* [OnePlus 5T specific quirks](#oneplus-5t-specific-quirks)
	* [Links](#links)

## Building
### Dependencies
Ubuntu 20.04 LTS
```
# apt install build-essential flex bison libssl-dev bc kmod gcc-aarch64-linux-gnu mkbootimg
```

### Kernel and modules
```sh
export ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu-
make oneplus5_defconfig # see arch/arm64/configs/

make -j$(nproc)
make INSTALL_MOD_PATH=rootfs modules_install -j$(nproc)
tar -czf modules.tar.gz -C rootfs/lib/modules . --owner=0 --group=0 --numeric-owner
```
**NOTE:** In case you want to boot without a ramdisk you need to `make menuconfig` after generating a `.config` and tweak the following:
```sh
# change sde21 (system) below to match your desired rootfs partition
CONFIG_CMDLINE="clk_ignore_unused loglevel=3 quiet root=/dev/sde21"
CONFIG_CMDLINE_FORCE=y
```
Otherwise see e.g. [my initramfs-tools](https://github.com/JamiKettunen/initramfs-tools) after building; the cmdline below will contain `rd.*` option examples that are specific to my ramdisk, such as booting a loopback-based `rootfs.img` on the `userdata` partition root :)

### Boot image
Next install [osm0sis/mkbootimg](https://github.com/osm0sis/mkbootimg) to create an Android bootloader compatible boot image:
```sh
DEVICE="cheeseburger" # change depending on which device you're targeting
cat arch/arm64/boot/{Image.gz,dts/qcom/msm8998-oneplus-$DEVICE.dtb} > Image.gz-dtb
mkbootimg --kernel Image.gz-dtb --ramdisk /path/to/initramfs.cpio* \
	--cmdline "clk_ignore_unused loglevel=3 quiet rd.rootfs=/dev/sda13/rootfs.img rd.extra_hooks=configfs,rndis" \
	--base 0x0 --pagesize 4096 -o boot.img
```
### Updating an existing boot.img
Same steps as above apply, except instead of `mkbootimg` run:
```sh
abootimg -u /path/to/boot.img -k Image.gz-dtb
```
**NOTE:** Kernel cmdline can also be easily tweaked using `abootimg -u ... -c "cmdline=..."` (assuming you're not using `CMDLINE_FORCE=y`).

## Manual deployment
Now you can use e.g. `fastboot boot boot.img` or `fastboot flash boot boot.img && fastboot reboot` to boot the newly compiled kernel.

### Firmware
Currently probing `ipa` (built as a module) without firmware in place causes a kernel panic (as of v5.12). If you don't wish to deal with this you can:
1. Set `CONFIG_QCOM_IPA=n` in `make menuconfig` and rebuild the kernel
2. Make a new boot image with `modprobe.blacklist=ipa` added to cmdline

Otherwise follow these steps to enable features such as Bluetooth and Wi-Fi:
```sh
wget https://github.com/JamiKettunen/firmware-mainline-oneplus5/archive/10.0.1.tar.gz
scp 10.0.1.tar.gz user@172.16.42.1:~
ssh -t user@172.16.42.1 "sudo sh -c 'mkdir -p /lib/firmware && tar -xmf 10.0.1.tar.gz --strip-components=1 -C /lib/firmware && find /lib/firmware/* -maxdepth 0 ! -name qca ! -name ath10k ! -name qcom -delete && rm 10.0.1.tar.gz && sync; echo Firmware deployed.'"
```

### Kernel modules
Once booted you should have an SSH connection via USB RNDIS to the device, which means you can just `scp` over the previously made `modules.tar.gz` and extract it:
```sh
scp modules.tar.gz user@172.16.42.1:~
ssh -t user@172.16.42.1 "sudo sh -c 'mkdir -p /lib/modules && tar -xmf modules.tar.gz -C /lib/modules && rm modules.tar.gz && sync && reboot'"
```
**NOTE:** `172.16.42.1` is the default IP address my ramdisk assigns to the device's USB interface from the RNDIS hook.

## What works?
* UFS internal storage
* Samsung S6E3FA5/S6E3FC1 display panels
	* There are some issues with refreshing, but the drivers are mostly usable
* Synaptics touchscreen (& back/recents capacitive buttons on OP5) via RMI4
* Capacitive button backlight on OP5 (see `/sys/class/leds/white:kbd_backlight`)
* Adreno 540 GPU
	* At least partially; there are various visual glitches currently on more complicated scenes
* Bluetooth (requires firmware blobs)
* Wi-Fi via ath10k SNOC (requires firmware blobs & [5 userspace daemons](https://wiki.postmarketos.org/wiki/Qualcomm_Snapdragon_835_(MSM8998)#WLAN))
	* Somewhat unstable; FW keeps crashing without [diag-router](https://github.com/andersson/diag) and on any type of network disconnect
	* With `ath10k_snoc` driver probed the shutdown is additionally hung by some amount due to the remoteproc refusing to shutdown cleanly
* Real-time clock (RTC)
	* Reported timestamp is always somewhere around 1972
	* Stored timestamp cannot be updated, but [alarms do fire when written relative to the incorrectly reported timestamp](https://wiki.postmarketos.org/wiki/Qualcomm_Snapdragon_835_(MSM8998)#Real-time_clock_.28RTC.29)
* USB in peripheral mode
	* No power delivery from the device's USB port -> host mode practically useless for now
* GPS via GPSD [with Qualcomm PDS patch](https://gitlab.com/gpsd/gpsd/-/merge_requests/139)
* Power and volume buttons
* Haptics/vibration motor
* Battery details query via fuel gauge
* Hall effect sensor (for magnetic flip cases and whatnot)
* Some cellular modem functions (tested via ModemManager/libqmi)
	* PIN unlock
	* Carrier info query
	* Sending SMS
* Notification light
	* [Patches for Qualcomm Light Pulse Generator from bamse's WIP tree](https://github.com/andersson/kernel/commits/wip/lpg)

## What doesn't work?
* Any kind of audio input/output (except via Bluetooth)
* Plenty of cellular modem functions (tested via ModemManager/libqmi)
	* Receiving SMS
	* Calls
	* Data connection
* Fingerprint sensors (& home button on OP5)
* Any cameras (sensors: IMX{398,350,376K,371})
* Camera flashlight
	* [Downstream driver here](https://github.com/LineageOS/android_kernel_oneplus_msm8998/blob/lineage-18.1/drivers/media/platform/msm/camera_v2/sensor/flash/msm_flash.c)
* Hardware video acceleration via [Venus](https://cateee.net/lkddb/web-lkddb/VIDEO_QCOM_VENUS.html)
	* [Pending SDM660 patches exist on LKML](https://patchwork.kernel.org/project/linux-arm-msm/cover/20210115185252.333562-1-angelogioacchino.delregno@somainline.org/); could be adapted somewhat easily for MSM8998?
* Practically all sensors
	* Needs "low power island" bringup + drivers as most of the sensors are controlled by this DSP
* Fast charging
	* This could perhaps be implemented when the proprietary `dashd` Android blob is reverse engineered
* NFC
	* While the PN544 I2C chip has a driver and [can be hooked up in device tree, there appears to be an SPI "secure element" chip handling some of the tasks](https://github.com/JamiKettunen/linux-mainline-oneplus5/commit/a7fdc7374a91feeee270d07207d3d8d25475b4e7) (and potentially even an Android blob? check on downstream!)
* CPUfreq scaling
	* This worked back on [v5.11](../tree/v5.11) but causes a kernel panic when probes since [v5.12](../tree/v5.12), waiting for updated patchset from Angelo that can hopefully get merged upstream
* Probably everything else I forgot to mention

## Other issues
* Performance
	* A stock postmarketOS install for example takes 40 seconds to boot; about half of the time is spent decompressing a 40 MB archive which should take only a few seconds when compared with downstream and SDM845 mainline
* Battery life
	* Primary sources of power consumption should be measured (how?)
* Might be a nitpick but there's some amount of ugly spam about clocks getting stuck and such in `dmesg`

## OnePlus 5T specific quirks
* Android 10 bootloader ([OxygenOS 10.0.1 FW](https://sourceforge.net/projects/crdroid/files/dumpling/6.x/firmware/firmware_10.0.1_oneplus5T.zip)) is the only one where I've experienced `fastboot boot` working so far
	* Particularly it doesn't work with [OxygenOS 9.0.11 FW](https://sourceforge.net/projects/crdroid/files/dumpling/6.x/firmware/firmware_9.0.11_oneplus5T.zip) flashed
* If appending more than **one** DTB (e.g. for cheeseburger + dumpling) the device won't boot anymore
	* Perhaps downstream (DTBs) can be investigated as downstream kernel boots fine with many unordered DTBs appended?

## Links
* [Original README](README)
* [initramfs-tools](https://github.com/JamiKettunen/initramfs-tools)
* [firmware-mainline-oneplus5](https://github.com/JamiKettunen/firmware-mainline-oneplus5)
* Defconfigs
	* [oneplus5_defconfig](arch/arm64/configs/oneplus5_defconfig)
	* [msm8998_defconfig](arch/arm64/configs/msm8998_defconfig)
* Device tree (DTS)
	* [msm8998-oneplus-common.dtsi](arch/arm64/boot/dts/qcom/msm8998-oneplus-common.dtsi)
	* [msm8998-oneplus-cheeseburger.dts](arch/arm64/boot/dts/qcom/msm8998-oneplus-cheeseburger.dts)
	* [msm8998-oneplus-dumpling.dts](arch/arm64/boot/dts/qcom/msm8998-oneplus-dumpling.dts)
* postmarketOS wiki pages
	* [OnePlus 5/5T devices](https://wiki.postmarketos.org/wiki/OnePlus_5_(oneplus-cheeseburger))
	* [Snapdragon 835 (MSM8998) SoC](https://wiki.postmarketos.org/wiki/Qualcomm_Snapdragon_835_(MSM8998))
* Other relevant mainline forks
	* [msm8998-mainline/linux](https://gitlab.com/msm8998-mainline/linux)
	* [sdm845-mainline/linux](https://gitlab.com/sdm845-mainline/linux)
	* [SoMainline/linux](https://github.com/SoMainline/linux)
	* [jhugo/linux](https://github.com/jhugo/linux)
