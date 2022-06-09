#pragma once

/*
Pinout:
https://randomnerdtutorials.com/esp32-pinout-reference-gpios/
*/

//output inverted SBUS? 
//inverted SBUS = true ===> normal UART
#define SBUS_INVERTED  false

#define SBUS_PIN 22

//#define TELEMETRY_BAUDRATE 115200
#define TELEMETRY_BAUDRATE 57600

//=============================================================================
//Receiver binding
#define USE_WIFI_CHANNEL 3
#define USE_KEY 0 

#define USE_LR_MODE true

//if there is not transmitter connection after powerup to the specified time,
//receiver will switch from LR to nomal mode to show AP and allow OTA updates
//set to 0 to disable
#define NORMAL_MODE_DELAY_MS 60*1000
