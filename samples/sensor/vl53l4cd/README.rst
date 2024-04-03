.. _vl53l4cd:

VL53L4CD: Time Of Flight sensor
##############################

Overview
********
This sample periodically measures distance between vl53l4cd sensor and target.
The result is displayed on the console.

Requirements
************
This sample uses the VL53L4CD sensor which is controlled using the I2C interface.

References
**********
 - VL53L4CD: https://www.st.com/en/imaging-and-photonics-solutions/vl53l4cd.html

Building and Running
********************
This project outputs sensor data to the console. It requires a VL53L4CD sensor.

 .. zephyr-app-commands::
    :app: samples/sensor/vl53l4cd/
    :goals: build flash

Sample Output
=============
 .. code-block:: console

    proximity = 1
    distance = 0.037 m
    proximity = 0
    distance = 0.104 m
    proximity = 0
    distance = 0.470 m
    proximity = 0
    distance = 0.146 m
    proximity = 1
    distance = 0.040 m
    proximity = 1
    distance = 0.071 m
    proximity = 0
    distance = 0.164 m
    proximity = 1
    distance = 0.034 m
    proximity = 0
    distance = 0.409 m
    proximity = 1
    distance = 0.061 m

    <repeats endlessly every second>
