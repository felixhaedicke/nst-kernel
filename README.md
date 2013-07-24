nst-kernel
==========

Linux Kernel for the Nook Simple Touch with fixed USB host and fast display mode support.


Currently, this contains a kernel based on the sources released by Barnes & Noble: http://images.barnesandnoble.com/PResources/download/Nook/source-code/nook2_1-1-5.tgz
+ and the changes created / collected by staylo: https://github.com/staylo/nook2
+ and this patch: ftp://ftp.cs.huji.ac.il/mirror/linux/kernel/linux/kernel/people/gregkh/usb/2.6/2.6.30-rc2/usb-musb_host-fix-ep0-fifo-flushing.patch


Installation:
See http://forum.xda-developers.com/showpost.php?p=24168019&postcount=21


Building the kernel:
Assuming a linux-x86_64 Android NDK is installed in /opt/android-ndk:

```
cp ../build/uImage.config src/.config
make ARCH=arm oldconfig
make -j6 ARCH=arm CROSS_COMPILE=/opt/android-ndk/toolchains/arm-linux-androideabi-4.7/prebuilt/linux-x86_64/bin/arm-linux-androideabi- uImage
cp arch/arm/boot/uImage ../build
```

