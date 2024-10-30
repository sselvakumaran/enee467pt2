#!/bin/bash

set -e

if [ ! -f /tmp/calibrationdata.tar.gz ]
then
  echo "Calibration output does not exist, save the calibration data and try again."
  exit 1
fi

if [ ! -d $1 ]
then
  echo "Save directory for the calibration file does not exist."
  exit 1
fi

cd /tmp

tar -C $1 -xf calibrationdata.tar.gz $(tar tf calibrationdata.tar.gz | grep ost.yaml) --overwrite

cd $1

if [ ! -f ost.yaml ]
then
  echo "Extraction of the calibration file failed. Exiting."
fi

if [ -f camera_info.yaml ]
then
  mv camera_info.yaml old_camera_info.yaml
fi

mv ost.yaml camera_info.yaml

printf "\nUpdated camera calibration file!\n"
