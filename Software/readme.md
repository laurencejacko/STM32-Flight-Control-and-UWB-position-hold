- "MATLAB PLOTTING"
  Produces figures from the recorded flight data
- "UWB FLIGHT CONTROL"
  Software that executes on the flight controller. The important files are found at:

 "core -> src -> main.c/.h"
 
 "core -> src -> freertos.c" (runs the RTOS scheduler)
 
 "tasks -> tasks_uwb.c/.h" (important functions for solving the multilateration position)

 LittleFS updates the flash memory

 The folder "Radio" refers to the driver for the onboard LoRa module, "CRSF_LIB" refers to the driver to use the CRSF protocol that lets the operator use an ELRS transmitter to control the UAV (ELRS device connects via a 4 pin picoblade connector and is not the onboard LoRa module). 

 - "UWB_CAN_REVB"

   Software that executes on the UWB enabled PCB. Configures between a static mode when mounted on the wall and a mobile mode that updates the flight controller with the distance data (ie the distance from the UAV to node A:F). In mobile mode, this data is updated as soon as is available with a CAN bus message. 

 
