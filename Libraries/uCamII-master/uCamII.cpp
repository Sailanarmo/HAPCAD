/*
 *    uCam.cpp
 *
 *    http://www.4dsystems.com.au/productpages/uCAM-II/downloads/uCAM-II_datasheet_R_1_4.pdf
 */

#include "uCamII.h"
#define cameraDebugSerial




// From http://pastebin.com/ZuMu48w7

byte _SYNC_COMMAND[6] = {0xAA, 0x0D, 0x00, 0x00, 0x00, 0x00};
byte _RESET[6] = {0xAA, 0x08, 0x00, 0x00, 0x00, 0x00};
byte _SYNC_ACK_REPLY[6] = {0xAA, 0x0E, 0x0D, 0x00, 0x00, 0x00};
byte _SYNC_ACK_REPLY_EXT[6] = {0xAA, 0x0D, 0x00, 0x00, 0x00, 0x00};
byte _SYNC_FINAL_COMMAND[6] = {0xAA, 0x0E, 0x00, 0x00, 0xF5, 0x00};

byte _INITIAL_COMMAND[6] = {0xAA, 0x01, 0x00, 0x07, 0x07, 0x07};
byte _GENERIC_ACK_REPLY[6] = {0xAA, 0x0E, 0x00, 0x00, 0x00, 0x00};
byte _PACK_SIZE[6] = {0xAA, 0x06, 0x08, UCAMII_BUF_SIZE + 6, 0x00, 0x00};
byte _SNAPSHOT[6] = {0xAA, 0x05, 0x00, 0x00, 0x00, 0x00};
byte _GET_PICTURE[6] = {0xAA, 0x04, 0x01, 0x00, 0x00, 0x00};



UCAMII::UCAMII() {
  this->image_pos = 0;
  this->package_no = 0;
}

boolean UCAMII::init() 
{
#ifdef cameraDebugSerial
  Serial.println("Intitial is starting to be sent");
#endif
  if (this->attempt_sync()) 
  {
#ifdef cameraDebugSerial
	Serial.println("\n\rCam has ACKED the SYNC");
#endif
    return true;
  }
  return false;
}

int UCAMII::attempt_sync() 
{
  int attempts = 0;
  byte cam_reply;
  int ack_success = 0;
  int last_reply = 0;
  int incrementTime = 0;

  while (attempts < 60 && ack_success == 0) {
    // Flush
    while (Serial1.available());
    delay(200);
#ifdef cameraDebugSerial
    Serial3.println("Trying...");
    Serial.println("Sending SYNC...");
#endif
    for (int i = 0; i < 6; i++) {
      Serial1.write(_SYNC_COMMAND[i]);
    }

    if (this->wait_for_bytes(_SYNC_ACK_REPLY) ) {
      if (this->wait_for_bytes(_SYNC_ACK_REPLY_EXT)) {
        delay(5 + incrementTime);
#ifdef cameraDebugSerial
        Serial.println("\r\nSending FINAL SYNC...");
#endif
        for (int i = 0; i < 6; i++) {
          Serial1.write(_SYNC_FINAL_COMMAND[i]);
        }
        return 1;
      }
    }
      attempts++;
      incrementTime++;
  }
  attempts = 0;
  incrementTime = 0;
  Serial3.println("Failed!");
  return 0;
}


// Return number of packages ready
int UCAMII::numberOfPackages() {
  return this->imageSize / UCAMII_BUF_SIZE;
}


int UCAMII::send_initial() {

  // flush
  while (Serial1.available() > 0) {
    Serial1.read();
  }

  delay(100);

#ifdef cameraDebugSerial
  Serial.println("Sending INITIALISE...");
#endif
  for (int i = 0; i < 6; i++) {
    Serial1.write(_INITIAL_COMMAND[i]);
  }
  // @todo why 500 delay?
  delay(500);
  if (this->wait_for_bytes(_GENERIC_ACK_REPLY)) {
#ifdef cameraDebugSerial
    Serial.println("INITIALISE success");
#endif
    return 1;
  }

#ifdef cameraDebugSerial
  Serial.println("INITIALISE fail");
#endif

  return 0;
}

boolean UCAMII::wait_for_bytes(byte command[6]) {
  byte cam_reply;
  int i = 0;
  int received;
  short found_bytes;
  found_bytes = 0;
  // @todo millis() wait, millis() wrap around watch out
  // unsigned long start = millis();

#ifdef cameraDebugSerial
  Serial.print("\r\nWAIT: ");
  for (i = 0; i < 6; i++) {
    Serial.print("0x");
    Serial.print(command[i], HEX);
    Serial.print(" ");
  }
  Serial.print("\r\nGOT : ");
  i = 0;
#endif

  while (Serial1.available()) {
    cam_reply = Serial1.read();
    if (i < 6 ) {
      if ((cam_reply == command[i]) || command[i] == 0x00) {
        found_bytes++;
        i++;
      }
    }

#ifdef cameraDebugSerial
    Serial.print("0x");
    Serial.print(cam_reply, HEX);
    Serial.println(" ");
#endif
    received++;
    if (found_bytes == 6) {
      return true;
    }
  }
  return false;
}



bool UCAMII::takePicture() {
  if (send_initial()) {
    if (this->set_package_size()) {
      if (this->do_snapshot()) {
        if (get_picture()) {
          return 1;
        }
      }
    }
  }
}


int UCAMII::set_package_size() {

  byte ack[6] = {0xAA, 0x0E, 0x06, 0x00, 0x00, 0x00};

  delay(100);

#ifdef cameraDebugSerial
  Serial.println("Sending packet size...");
#endif

  for (int i = 0; i < 6; i++) {
    Serial1.write(_PACK_SIZE[i]);
#ifdef cameraDebugSerial
    Serial.print(_PACK_SIZE[i], HEX);
    Serial.print(" ");
#endif

  }
  // @todo why 500 delay?
  delay(500);
  if (this->wait_for_bytes(ack)) {
#ifdef cameraDebugSerial
    Serial.println("\r\npacket size success");
#endif
    return 1;
  }

#ifdef cameraDebugSerial
  Serial.println("packet size fail");
#endif

  return 0;
}

int UCAMII::do_snapshot() {

  byte ack[6] = {0xAA, 0x0E, 0x05, 0x00, 0x00, 0x00};

  delay(100);

#ifdef cameraDebugSerial
  Serial.println("Sending snapshot...");
#endif


  for (int i = 0; i < 6; i++) 
  {
    Serial1.write(_SNAPSHOT[i]);
  }
  // @todo why 500 delay?
  delay(500);
  if (this->wait_for_bytes(ack)) {
#ifdef cameraDebugSerial
    Serial.println("snapshot success");
#endif
    return 1;
  }

#ifdef cameraDebugSerial
  Serial.println("snapshot fail");
#endif

  return 0;

}

int UCAMII::get_picture() {

  byte ack[6] = {0xAA, 0x0E, 0x04, 0x00, 0x00, 0x00};
  unsigned long imageSize;
  short i;

  delay(100);

#ifdef cameraDebugSerial
  Serial.println("Sending get picture...");
#endif

  for (int i = 0; i < 6; i++) 
  {
    Serial1.write(_GET_PICTURE[i]);
  }
  // @todo why 500 delay?
  delay(500);
  if (this->wait_for_bytes(ack)) {
#ifdef cameraDebugSerial
    Serial.println("picture success");
#endif
    // get the 6 bytes ACK
    for (i = 0; i <= 5; i++) {
      ack[i] = 0;
      while (!Serial1.available());

      ack[i] = Serial1.read();
      // last 3 bytes are the image size
#ifdef cameraDebugSerial
      Serial.print(i, DEC);
      Serial.print(" value: ");
      Serial.println(ack[i], HEX);
#endif
    }

    imageSize = 0;
    imageSize = (imageSize << 8) | ack[5];
    imageSize = (imageSize << 8) | ack[4];
    imageSize = (imageSize << 8) | ack[3];

    this->imageSize = imageSize;
    this->image_pos = this->imageSize;
    if (imageSize > 0) {
      return 1;
    }
  }

#ifdef cameraDebugSerial
  Serial.println("picture fail");
#endif

  return 0;
}

// @todo return false if time exceeded
int UCAMII::getData() {

  unsigned char high = (unsigned char)(this->package_no >> 8);
  unsigned char low  = this->package_no & 0xff;
  byte my_ack[6] = {0xAA, 0x0E, 0x00, 0x00, low, high};

  int i = 0;
  byte s;

  int bytes;

  // request bytes
  for (int i = 0; i < 6; i++) {
    Serial1.write(my_ack[i]);
  }
  

  // Set number of bytes we should wait for
  if (this->image_pos < UCAMII_BUF_SIZE) {
    bytes = this->image_pos + 6;
  } else {
    bytes = UCAMII_BUF_SIZE + 6;
  }
#ifdef cameraDebugSerial
//    Serial.print("REMAINING: ");
//    Serial.print(this->image_pos, DEC);
//    Serial.print(" BYTES PER CHUNK: ");
//    Serial.println(bytes, DEC);


#endif

  for (i = 0; i < bytes; i++) {
      while (!Serial1.available());
      // wait for bytes
          s = Serial1.read();

          // Skip first 4 and last 2, Page 10 of the datasheet
          if (i >= 4 && i < bytes - 2) {
#ifdef cameraDebugSerial
              //Serial.print("*");
#endif
              this->imgBuffer[i - 4] = s;
              this->image_pos--; //hitting 0 before it needs to hit 0? add print statements
          }
#ifdef cameraDebugSerial
          //Serial.print(this->imgBuffer[s], HEX);
          //Serial.print(HEX);
          //Serial.print(" ");


#endif
      }


#ifdef cameraDebugSerial
      Serial.println("");
#endif

  this->package_no++;
  if(this->image_pos <= 0) {
    // send the final thank you goodbye package
    my_ack[4] = 0xF0;
    my_ack[5] = 0xF0;
    for (int i = 0; i < 6; i++) {
      Serial1.write(my_ack[i]);
    }
      if(this->image_pos == 0) {
          for(int i = 0;i<6;i++){
              Serial1.write(_RESET[i]);
          }
          //Serial.println("RESET SUCCESSFUL");
          this->image_pos = 0;
          this->package_no = 0;
      }
  }
  return bytes-6;

}



