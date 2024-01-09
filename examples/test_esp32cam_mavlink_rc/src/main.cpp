#include <Arduino.h>
#include "HX_ESPNOW_RC_Slave.h"
#include "rx_config.h"
#include "hx_mavlink_rc_encoder.h"
#include "hx_mavlink_parser.h"
#include "hx_mavlink_frame_sender.h"
#include "HX_ESPNOW_RC_SerialBuffer.h"
#include "frame.h"





HXMavlinkParser outgoingMavlinkParser(true);
HXMavlinkFrameSender mavlinkFrameSender;

unsigned long lastStats = millis();

//=====================================================================
//=====================================================================
typedef enum
{
    RCV_GOT_CONNECTION_ONCE  = 0,
    RCV_NORMAL_MODE          = 1,
    RVC_WAITING_CONNECTION   = 2,
} RSState;

uint8_t state = RVC_WAITING_CONNECTION;
unsigned long startTime = millis();

HardwareSerial mavlinkSerial(2);

//=====================================================================

//=====================================================================
void trySendFrame()
{

  mavlinkFrameSender.send( mavlinkSerial );

  if (mavlinkFrameSender.isEmpty())
  {
    mavlinkFrameSender.addFrame(frame, FRAME_DATA_SIZE);
  }
}


//=====================================================================
void setup()
{

  Serial.begin(115200);
  pinMode( 17, OUTPUT);
  mavlinkSerial.begin(57600, SERIAL_8N1, 17, 18 );

  size_t freeHeap = ESP.getFreeHeap();

}
//=====================================================================

void loop()
{  
      trySendFrame();
   if (millis() - lastStats > 1000)
  {
    lastStats = millis();
    mavlinkFrameSender.dumpState();

  
}
}
 
