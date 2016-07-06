#include <uCamII.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <TimeLib.h>
#include <DS1307RTC.h>
#include "Adafruit_MCP9808.h"
#include "I2Cdev.h"
#include <CoolSatBaro.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <TinyGPS++.h>

#define BaroAddress 0x76 //defining the address for the Barometor?

#define BNO055_SAMPLERATE_DELAY_MS (100) //Delay for the Gyro


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE //Not sure why this is needed
#include "Wire.h"
#endif

//=========== Creating Instances of our sensors ==========//
UCAMII camera;  //instance for the camera
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808(); //instance for the outer temperature sensor
Adafruit_MCP9808 tempsensor2 = Adafruit_MCP9808(); //instance for the inner temperature sensor
Adafruit_BNO055 bno = Adafruit_BNO055(); //instance for the Gyro
CoolSatBaro myBaro; //instance for the barometer 
TinyGPSPlus gps;
//========================================================//

//#define OUTPUT_READABLE_ACCELGYRO

//================ Text Files ==================//
File myGyro;
File myAccel;
File myFile;
File myTemp;
File myTemp2;
File myLight;
File myBarotxt;
File dummy;
//=============================================//

tmElements_t tm; //in the time library somewhere this is called.

//===================== Global Variables =======================//
const int wireCutter = 12; //pin for our wire cutter
const int powerOff = 10; //pin for removing battery power when voltage is to low
const int boomSwitch = 30;
const long OneSec = 995;  //time for 1 second
const long FiveMin = 299995;  //time for 5 minutes
unsigned long previousMillis = 0; //reset the time 
unsigned long previousStamp = 0;  //reset the time

byte serialReadCommand; //needed so the Arduino can read commands sent through the Radio.

bool commandSent = false;
bool commandSent2 = false;
bool commandSent3 = false;
bool takePicture = false; //setting to false so it will only activate for the fail safe.
bool powerCheck = false;
bool printheader = false; // bool so header in file only prints one time
bool printTemp1Header = false; // bool so header in file only prints one time
bool printTemp2Header = false; // bool so header in file only prints one time
bool printGyroHeader = false; // bool so header in file only prints one time
bool printAccelHeader = false; // bool so header in file only prints one time
bool cut = false;

const int lightPin = 15; //Light sensor pin, should change the name
const int batteryPin = 0;  //Battery reading pin
const int uvPin = 1;  //UV sensor pin

const int chipSelect = 46; //enable for SD card

float val = 0.0;  //Voltage
float RLDR = 0.0; //Resistance in Ohms
float Lux = 0.0;  //Lux reading
float battery = 0.0;  //Battery Voltage
float uv = 0.0;  // uv sensor
//==============================================================//

void setup() {
  //Serial.begin(9600);
  Serial2.begin(9600);
  Serial3.begin(9600);  //Initiating the Serial port to push out the the Radio
  Wire.begin(); //Begining everying on our I2C Bus

  Serial3.print(F("Testing TinyGPS++ library v. ")); Serial3.println(TinyGPSPlus::libraryVersion());

  myBaro.initial(BaroAddress);  //Passing the address to initialize the Barometer
  // Disable internal pullups, 10Kohms are on the breakout
  PORTC |= (1 << 4);
  PORTC |= (1 << 5);

  pinMode(wireCutter, OUTPUT);  //Initializing our wirecutter
  pinMode(boomSwitch, INPUT);   //Initializing our Boom Switch
  pinMode(powerOff, OUTPUT);   //Initializing power cut off
   
  //Serial.print("Initializing SD card..."); // Initiallizes SD Card in Port 22.
  Serial3.print("Initializing SD card...");

  if (!SD.begin(chipSelect)) 
  {
    //Serial.println("initialization failed!"); //if initialization of SD Card fails, should stop here.
    Serial3.println("initialization failed!");
    return;
  }
  //Serial.println("initialization done."); //Done initializing the SD Card.
  Serial3.println("initialization done.");
  //Serial.println("");
  Serial3.println("");
  
  //initializing our temperature sensors
if (!tempsensor.begin()) {
    //Serial.println("Couldn't find MCP9808!");
    while (1);
  }
if (!tempsensor2.begin2()) {
    //Serial.println("Couldn't find MCP9808!");
    while (1);
  }

//initializing the Barometer
if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  bno.setExtCrystalUse(true);
  
}


void loop() {
  if(powerCheck == false){
    digitalWrite(powerOff, LOW);
    delay(1000);
    powerCheck = true;
  }
  unsigned long currentMillis = millis(); //millis() is a function for the arduino
  unsigned long timeMillis = millis();
  int bytes = 0;
  bool cameraInit = false;  //we have the camera not initialized so it will only respond once we send it a command
  unsigned long counter = 0; 

  runGyroSensor();  //Runs the Gyro in the "background" it's not really "tasking" but it runs whenever the other sensors are not running.

  //Enabling the arduino to read whatever commands is sent through the Radio.
  if(Serial3.available()){
    serialReadCommand = Serial3.read();
    Serial3.println("");
    Serial3.println("");
    Serial3.write(serialReadCommand);
    Serial3.println("");

  //If 1 is pressed, taked a picture
  if (serialReadCommand == 49 && !commandSent){
    commandSent = true;
    Serial3.println("Attempting to take Picture...");
    }

  //if 9 is pressed, activate the Wire Cutters
  if (serialReadCommand == 57 && !commandSent2){
      Serial3.println("**************");
      Serial3.println("ABOUT TO ACTIVATE THE WIRE CUTTERS, ARE YOU SURE YOU WANT TO DO THIS?!?! Press y for Confirmation");
      Serial3.println("**************");
      while(counter < 60000){
        serialReadCommand = Serial3.read();
        if(serialReadCommand == 121){
          commandSent2 = true;
          Serial3.println("WIRE CUTTERS ACTIVATED!!!");
          break;
        }
        Serial.println(counter++);
      }  
    }

  //If 3 is pressed, send us a text file of our data.
//  if (serialReadCommand == 51 && !commandSent3){
//    commandSent3 = true;
//    Serial3.println("Sending you a text file, standby...");  
//    }
  }
  
  //If you hit 1 second, purge yourself and then do the following. 
  if (currentMillis - previousMillis >= OneSec){
    previousMillis = currentMillis;
    
    //Checking to see if a command has been sent through the radio.
    if (commandSent){
      runCamera();
      commandSent = false;
      Serial3.println("");
      Serial3.println("");
    }
    if (commandSent2){
      cutWire();
      commandSent2 = false;
      Serial3.println("");
      Serial3.println("");
    }
//    if (commandSent3){
//      sendText();
//      commandSent3 = false;
//      Serial3.println("");
//      Serial3.println("");      
//    }

    myBaro.readBaro(); //Give me a reading of the pressure
    //gps.encode(Serial2.read());
    
    myBarotxt = SD.open("Baro.txt", FILE_WRITE);
    myLight = SD.open("Light.txt", FILE_WRITE);

    //=============== Math ==================//
    val = analogRead(lightPin); //Read the value of the light sensor pin
    val = val * .0048; //Convert to Voltage
    battery = analogRead(batteryPin); //Read the value of the batter level sensor pin
    battery = (battery * .00475) * 2; //Convert to voltage while taking the voltage divider circuit into the equation.
    if(battery <= 6.3){
      digitalWrite(powerOff, LOW);
      delay(1000);
    }
    uv = analogRead(uvPin); //Read the value of the light sensor pin
    RLDR = (1000.0 * (5 - val))/val; //convert to Resistance in Ohms
    Lux = (776897.0 * (pow(RLDR, -1.206))); //Convert to Lux
    //=======================================//

    //Print to the Radio and write to the SD Card

    if(printheader == false){
      //print header for Serial3(radio)
      Serial3.println("");
      Serial3.print("\tBatt\t");
      //Serial3.print("Baro\t");
      Serial3.print("Actual\t\t");
      //Serial3.print("Corrected\t");
      Serial3.println("");
      Serial3.print("Lux\t");
      Serial3.print("Level\t");
      //Serial3.print("Temp\t");
      Serial3.print("Press(mb)\t");
      //Serial3.print("Press(mb)\t");
      Serial3.print("Altitude\t");
      Serial3.print("Temp Inner\t");
      Serial3.print("Temp outer\t");
      Serial3.print("Time\t\t");
      Serial3.print("Date\t");
      Serial3.println("");
      Serial3.print("-----------------------------------------------------------------------------------------------------------------------------------------");
      Serial3.println("");
      
      //print header for myLight text file on SD card
      myLight.print("Lux\t");
      myLight.print("Volts\t");
      myLight.print("Ohms\t\t");
      myLight.print("Switch\t\t");
      myLight.print("Battery\t\t");
      myLight.print("UV\t\t");
      myLight.print("Time\t\t");
      myLight.print("Date\t");
      myLight.println("");
      
      //print header for myBarotxt text file on SD card
      myBarotxt.print("Barometor Temp\t\t");
      myBarotxt.print("Actual Pressure(mb)\t");
      myBarotxt.print("Corrected Pressure(mb)\t");
      myBarotxt.print("Altitude\t");
      myBarotxt.print("Time\t\t");
      myBarotxt.print("Date\t");
      myBarotxt.println("");
      printheader = true;
    }

    //print values for Serial3(radio)
    Serial3.print(Lux);
    Serial3.print("\t");
    Serial3.print(battery);
    //Serial3.print("\t");
    //Serial3.print(myBaro.getTemp());
    Serial3.print("\t");
    Serial3.print(myBaro.getPressure());
    Serial3.print("\t\t");
    //Serial3.print(myBaro.getCorrectedPressure());
    //Serial3.print("\t\t");
    Serial3.print(myBaro.getAltitude());
    Serial3.print("\t\t");

    //print values for myLight text file on SD card
    myLight.print(Lux);
    myLight.print("\t");
    myLight.print(val);
    myLight.print("\t");
    myLight.print(RLDR);
    myLight.print("\t\t");
    myLight.print(digitalRead(boomSwitch));
    myLight.print("\t\t");
    myLight.print(battery);
    myLight.print("\t\t");
    myLight.print(uv);
    myLight.print("\t\t");
    timeStamp(myLight);
    myLight.close();

    //print values for myBarotxt text file on SD card
    myBarotxt.print(myBaro.getTemp());
    myBarotxt.print("\t\t\t");
    myBarotxt.print(myBaro.getPressure());
    myBarotxt.print("\t\t\t");
    myBarotxt.print(myBaro.getCorrectedPressure());
    myBarotxt.print("\t\t\t");
    myBarotxt.print(myBaro.getAltitude());
    myBarotxt.print("\t\t");
    timeStamp(myBarotxt);
    myBarotxt.close();

    //call temp sensor functions to read/print to radio and SD card
    tempSens1();
    tempSens2();

    //time stamp with dummy file name so time stamp will only print once to radio each pass
    timeStamp(dummy);
    

    //if statement for automated wire cut and picture take and send on radio
    if(myBaro.getPressure() <= 44 && cut == false){
      cutWire();
      cut = true;
    }


  }
  if(takePicture == true){
    if (timeMillis - previousStamp >= FiveMin){
      previousStamp = timeMillis;
      runCamera();
    }
  }
}

//function to take a picture
void runCamera() {
  int bytes = 0; 
  Serial1.begin(115200); // uCAM-II Default Baud Rate
 
  myFile = SD.open("Picture.txt", FILE_WRITE);
  myFile.println("Picture Begin...");
  timeStamp(myFile);  
  

  bool cameraInit = 0;

  cameraInit = camera.init(); // Initializing the camera, this must be done every time or the camera will go to sleep forever. 

  if(cameraInit == true){
  
  camera.takePicture(); // Taking a picture.

  Serial3.print("Image Size: ");
  
  Serial3.println(camera.imageSize, DEC);

  Serial3.print("Number of Packages: ");
  Serial3.println(camera.numberOfPackages(), DEC);

  while(bytes = camera.getData())
  {
    // while the bytes are getting the data? loop through.
    for (short x = 0; x < bytes; x++)
    {
//      Serial3.print("0x");
//      Serial3.print(camera.imgBuffer[x], HEX);  //uncomment if you want to see camera data over the radio when picture is taken
//      Serial3.print(" ");                       //this was commented out because it take longer to get a picture if sent over the radio
      myFile.print("0x"); // printing out to the text file.
      myFile.print(camera.imgBuffer[x], HEX);
      myFile.print(" ");
    }
  }
  Serial3.println("");
  myFile.println("");
  myFile.println("End of picture.");
  timeStamp(myFile);
  myFile.println("");
  Serial3.println("Done Downloading");
  }
  myFile.close(); //closing the text file.  
}

//function call for the wire cutter. Stays on for 3 seconds which is just enough to cut the wires.
void cutWire() {
  Serial3.println("");
  Serial3.println("");
  Serial3.println("Wire cutter:");
  // turn Cutter on:
  digitalWrite(wireCutter, HIGH);
  Serial3.println("On");
  delay(3000);
    
  // turn LED off:
  digitalWrite(wireCutter, LOW);
  Serial3.println("Off");
  delay(1000);

  Serial3.println("Done cutting!!");
  Serial3.println("");
  Serial3.println(""); 

  delay(20000);
  runCamera();
  runCamera();
  runCamera();
  runCamera();
  runCamera();
 // sendText();
  takePicture = true;
}

void runGyroSensor(){

  myGyro = SD.open("Gyro.txt", FILE_WRITE);
  myAccel = SD.open("Accel.txt", FILE_WRITE);

  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> acceleration = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);


  if(printGyroHeader == false){
    myAccel.print(" m/s^2 "); myAccel.println(""); myAccel.print("X:\t\t"); myAccel.print(" Y:\t\t"); myAccel.print(" Z:\t\t"); myAccel.print("Time\t\t"); myAccel.print("Date"); myAccel.println("");
    printGyroHeader = true;
  }
  /* Display the floating point data */
  
  myAccel.print(acceleration.x());
  myAccel.print("\t\t");
  myAccel.print(acceleration.y());
  myAccel.print("\t\t");
  myAccel.print(acceleration.z());
  myAccel.print("\t\t");
  timeStamp(myAccel);

  

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  

  if(printAccelHeader == false){
    myGyro.print(" Degrees. "); myGyro.println(""); myGyro.print("X:\t\t"); myGyro.print(" Y:\t\t"); myGyro.print(" Z:\t\t"); myGyro.print("Time\t\t"); myGyro.print("Date"); myGyro.println(""); 
    printAccelHeader = true;
  }
    /* Display the floating point data */
  
  myGyro.print(euler.x());
  myGyro.print("\t\t");
  myGyro.print(euler.y());
  myGyro.print("\t\t");
  myGyro.print(euler.z());
  myGyro.print("\t\t");
  timeStamp(myGyro);
   
  myGyro.close();
  myAccel.close();

  delay(BNO055_SAMPLERATE_DELAY_MS);
}

void tempSens1(){
    tempsensor.shutdown_wake(0);   // Don't remove this line! required before reading temp
    float c = tempsensor.readTempC();
    float f = c * 9.0 / 5.0 + 32;  // conversion to Farenheight
    myTemp = SD.open("Temp1.txt", FILE_WRITE);

    if(printTemp1Header == false){
      myTemp.print("Temp Inner:\t"); myTemp.println(""); myTemp.print("*C\t"); myTemp.print("*F\t"); myTemp.print("Time\t\t"); myTemp.print("Date\t"); myTemp.println("");// Printing to the console 
      printTemp1Header = true;
    }
    myTemp.print(c); myTemp.print("\t"); myTemp.print(f); myTemp.print("\t");   // Printing to the text file.
    Serial3.print(c); Serial3.print("*C\t\t"); 
    //Serial3.print(f); Serial3.print("*F   ");
      
    delay(250);
    timeStamp(myTemp);
    myTemp.close();
    tempsensor.shutdown_wake(1);
}


void tempSens2(){
    tempsensor2.shutdown_wake(0);   // Don't remove this line! required before reading temp
    float c = tempsensor2.readTempC();
    float f = c * 9.0 / 5.0 + 32;  // conversion to Farenheight
    myTemp2 = SD.open("Temp2.txt", FILE_WRITE);

    if(printTemp2Header == false){
      myTemp2.print("Temp Outer:\t"); myTemp2.println(""); myTemp2.print("*C\t"); myTemp2.print("*F\t"); myTemp2.print("Time\t\t"); myTemp2.print("Date\t"); myTemp2.println("");  
      printTemp2Header = true;
    }
    myTemp2.print(c); myTemp2.print("\t"); myTemp2.print(f); myTemp2.print("\t");   // Printing to the text file.
    Serial3.print(c); Serial3.print("*C\t\t"); 
    //Serial3.print(f); Serial3.print("*F   "); 
    
    delay(300);
    timeStamp(myTemp2);
    myTemp2.close();
    tempsensor2.shutdown_wake(1);
}

//void sendText(){
//
//  char readData;
//  byte terminate = 72;
//  
//  myFile = SD.open("Picture.txt", FILE_READ);
//    while (myFile.available()){
//    readData = myFile.read();
//    if (readData){
//      Serial3.print(readData);
//    }
//  }
//  myFile.close();
//  delay(200);
//
//  Serial3.println("Done"); 
//}

void timeStamp(File file){
  if (RTC.read(tm)){
    if(file == dummy){
      ////Serial3.print("Time: ");
      print2digits(file, tm.Hour);
      Serial3.write(':');
      print2digits(file, tm.Minute);
      Serial3.write(':');
      print2digits(file, tm.Second);
      Serial3.print("\t");
      Serial3.print(tm.Day);
      Serial3.write('/');
      Serial3.print(tm.Month);
      Serial3.write('/');
      Serial3.print(tmYearToCalendar(tm.Year));
      Serial3.print("   ");
      Serial3.print(digitalRead(boomSwitch));
      Serial3.print("   ");
      Serial3.print(uv);
      Serial3.print("\t");
      printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
      Serial3.print("\t");
      printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
      smartDelay(1000);
//      Serial3.print("\t");
//      printFloat(gps.location.lat(), 11, 6);
//      Serial3.print("\t");
//      printFloat(gps.location.lng(), 12, 6);
      Serial3.println();
    }else{
      ////file.print("Time: ");
      print2digits(file, tm.Hour);
      file.write(':');
      print2digits(file, tm.Minute);
      file.write(':');
      print2digits(file, tm.Second);
      file.print("\t");
      file.print(tm.Day);
      file.write('/');
      file.print(tm.Month);
      file.write('/');
      file.print(tmYearToCalendar(tm.Year));
      file.println();
    }
  }
}

void print2digits(File file, int number){
  if (number >= 0 && number < 10){
    if(file == dummy){
      Serial3.write('0');
      Serial3.print(number);
    }else{
      file.write('0');
      file.print(number);
    }
  }else{
    if(file == dummy){
      Serial3.print(number);
    }else{  
      file.print(number);
    }
  }
}

void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (Serial2.available()){
      gps.encode(Serial2.read());
    }
  } while (millis() - start < ms);
}

void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial3.print('*');
    Serial3.print(' ');
  }
  else
  {
    Serial3.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial3.print(' ');
  }
  smartDelay(0);
}
