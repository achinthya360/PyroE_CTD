# PyroE_CTD
A CTD system developed for PyroE, Inc. under the mentorship of Dr. Kevin Lu, Ph.D. The system is being built with care taken to maintain low power consumption (ideally under 1 mW) along with a small overall physical design architecture. 

## Acknowledgements:
This project is largely thanks to the OpenCTD platform which can be found at https://github.com/OceanographyforEveryone/OpenCTD. All included libraries can be found at that repository.

## Navigating the Repo:
The code files are split into 3 sections: TailAndMotor, SensorTesting, and MCUCommunication. 

### TailAndMotor
This folder contains miscellaneous sketches for running a hybrid step-servo motor to create sinusoidal oscillations along an anguilliform tail while simultaneously collecting IMU data at multiple nodes along the tail. More work is required to meet our goal of continuously visualizing and analyzing the tail's waveform from the data acquired.

### SensorTesting
This folder contains numerous sketches pertaining to breadboarding, calibrating, and plotting data from temperature, conductivity, and pressure sensors, as well as using and collecting data with Invensense MPUs. 

### MCUCommunication
This folder contains 2 main files (feather.ino and nano.ino) along with a few peripheral testing sketches used for testing Bluetooth or Wifi connectivity and SD card data retrieval capability. The feather.ino file contains all the code required for the CTD itself, and a large portion is attributed to the researchers at OpenCTD. The nano.ino file provides basic examples of code for turning on and off the CTD and communicating with it from a central MCU. In our case, we used an Arduino Nano Every microcontroller on our EEL robot's head. 

### Extra info:
Bluetooth Commands:
https://docs.google.com/document/d/15tzf24LUGOdAkbHmA78yEyWiHE7d_mF4LLaTbkWwPSY/edit?usp=sharing
Manual:
https://docs.google.com/document/d/1CsEsWTur9dHu2vPHQrDwllO8IHEek2RX2XKdxYSUBIw/edit?usp=sharing
