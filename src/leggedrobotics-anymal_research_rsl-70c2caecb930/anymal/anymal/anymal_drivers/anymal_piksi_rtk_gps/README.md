# Package for Piksi RTK GPS Receiver

## Overview 

Launch file and configurations for Piksi RTK GPS receiver installed in ANYmal.

**Keywords:** anymal, navigation, piksi multi, rtk gps.

## License

This software is licensed under a [proprietary license](../LICENSE).

**Author(s):** Marco Tranzatto.  
**Maintainer:** Marco Tranzatto, marcot@ethz.ch.

## Documentation

An overview of Piksi RTK GPS is available on [ethz_piksi_ros](https://github.com/ethz-asl/ethz_piksi_ros).

## Usage

Launch the ROS driver with: 

	roslaunch anymal_piksi_rtk_gps piksi_rtk_gps.launch


### Config files

* **enu_origin.yaml:** parameters for the origin of the ENU (East-North-Up) frame that is used to convert WGS-84 coordinates (latitude, longitude, altitude) in local Cartesian ENU coordinates (east, north, up).

* **piksi_rtk_gps.yaml:** parameters for the ROS driver.

### Udev rule
Make sure the following file exists on ANYmal: `/etc/udev/rules.d/95-piksi-rtk-gps-pcb-serial.rules`. Its content should look like


```
# Piksi Multi RTK GPS Receiver - FTDI chip embedded in PCB created at RSL - ETHZ
KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="A5063CCR", SYMLINK+="anymal/piksi_rtk_gps"
```

You may want to adapt the fileds "idVendor", "idProduct", and "serial" in case you use a different FTDI chip than the one integrated in the PCB that is used at the moment (25/04/2018).
