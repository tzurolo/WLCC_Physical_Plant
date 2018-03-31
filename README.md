# WLCC_Physical_Plant

This system monitors the boiler and oil tank using computer vision. Status of the system is displayed on a web page, and alerts can be generated based on various conditions (oil low, pressure too high, etc.)


The system consists of two camera boxes: one for the boiler, one for the oil tank.
The boiler camera system watches the pressure gauge and the boiler control panel.
The oil tank camera system watches the oil level gauge on the top of the tank.

## System Details

Each camera box has a Raspberry Pi B+ running Debian Jessie.
A C++ program is launched by the node.js server.
The C++ program uses the RaspiCam library to capture camera images and OpenCV to analyze the images. 
