Introduction
This project demonstrates how to use ESP32 to interface with BME680 and BH1750 sensors to collect environmental data and transmit it using the Zigbee protocol. 
The code initializes the sensors, reads the sensor data, and sends the data to a Zigbee network coordinator.

Hardware Requirements
ESP32: The main microcontroller for interfacing with sensors and handling Zigbee communication.
BME680 Sensor: Used to measure temperature, humidity, pressure, and gas resistance.
BH1750 Sensor: Used to measure light intensity (lux).
I2C Connection: Ensure proper wiring between the ESP32 and the sensors using I2C protocol.
Software Requirements
ESP-IDF: The official development framework for the ESP32. Ensure it's installed and configured properly.
ESP-Zigbee Library: Required for Zigbee communication. Available through ESP-IDF components.
FreeRTOS: For task management and multitasking.

Sources: 
https://github.com/espressif/esp-zigbee-sdk/issues/233
https://github.com/espressif/esp-zigbee-sdk/issues/74
https://github.com/kylemwagner83
