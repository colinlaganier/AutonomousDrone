# Drone Client

## Description 

This is the source code of the online control loop, positioning and communication. This code should be run on the drone mounted Raspberry Pi. 

To use on the Raspberry Pi, ensure the following packages are installed:
* Mavlink_v2
* WiringPi
* Pthread

To run the code, ensure the server is up and running and enter the following command in the command line from the DroneClient directory: 

```
mkdir build && cd build
cmake ../CMakeLists.txt
make 
./DroneClient
```

