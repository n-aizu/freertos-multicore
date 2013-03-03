#!/bin/sh
if [ $# -lt 1 ]; then
   echo "example usage: $0 /dev/sdc"
   exit 1
fi

DRIVE=$1

dd if=/dev/zero of=$DRIVE bs=1024 count=1024

SIZE=`fdisk -l $DRIVE | grep Disk | awk '{print $5}'`

echo DISK SIZE - $SIZE bytes

CYLINDERS=`echo $SIZE/255/63/512 | bc`

echo CYLINDERS - $CYLINDERS

{
echo ,11,0x0C,*
echo ,168,,-
} | sfdisk -D -H 255 -S 63 -C $CYLINDERS $DRIVE

mkfs.vfat -F 32 -n "boot" ${DRIVE}1
mkfs.ext4 -j -L "rootfs" ${DRIVE}2

