#!/bin/bash
TMP=$PWD
cd /usr/realtime/modules/

KERNELVERSION=`uname -r | awk '{print substr($1,1,3)}'`

if [ "$KERNELVERSION" = "2.6" ]
then

insmod ./rtai_hal.ko
insmod ./rtai_lxrt.ko
#insmod ./rtai_usi.ko
insmod ./rtai_fifos.ko
insmod ./rtai_shm.ko
cd $TMP

else

insmod ./rtai_hal.o
insmod ./rtai_lxrt.o
insmod ./rtai_usi.o
insmod ./rtai_fifos.o
cd $TMP

fi
