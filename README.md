# ESPHome-LD2410
ESPHome LD2410 mmWave Radar Sensor Custom Component

## Installation
Set wifi_ssid and wifi_password in your esphome's secrets.yaml first
1. Place ld2410_uart.h into your esphome configuration folder
2. Create new device with the yaml in this repository

## Wiring
ESP8266  |  LD2410  
5V      <-> VCC  
GND     <-> GND  
TX      <-> RX  
RX      <-> TX  
