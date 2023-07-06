#!/bin/bash -x
export LD_LIBRARY_PATH=.:$LD_LIBRARY_PATH
echo 293 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio293/direction
echo 1 > /sys/class/gpio/gpio293/value
echo 293 > /sys/class/gpio/unexport


echo 290 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio290/direction
echo 0 > /sys/class/gpio/gpio290/value
sleep 0.5
echo 1 > /sys/class/gpio/gpio290/value
echo 290 > /sys/class/gpio/unexport

i2cdetect -y -r 5
lsof -i :10086 | grep LISTEN | awk '{print $2}' | xargs kill -9
export LOGLEVEL=4
echo 1 >/sys/class/vps/mipi_host0/param/stop_check_instart
echo 1 >/sys/class/vps/mipi_host1/param/stop_check_instart

/app/bin/vps/vpm/vcs_test -m 4 -v ./vpm_config.json -p 0 -c hb_j5dev.json  -i 0 -h /system/etc/vio_tool/dump_raw.json -s 2147483647