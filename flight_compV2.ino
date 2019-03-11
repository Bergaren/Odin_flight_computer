// Noé Bolin & Nils Lindberg Odhner 

#include <Wire.h>
#include <SPI.h>
#include <SdFat.h>
#include "FaBo9Axis_MPU9250.h"
SdFat SD;

#define DEBUG 1   //comment this line to suppress all serial.print

#ifdef DEBUG
 #define DEBUG_PRINT(x)  Serial.println (x)
 #define DEBUG_PRINTLN(x)  Serial.println (x)
#else
 #define DEBUG_PRINT(x)
 #define DEBUG_PRINTLN(x)
#endif

unsigned int windowBegin = 7000, windowEnd = 9000;    //specifying the time window in which the parachute shall open
unsigned int timeOpenRelay = 16000;                   //specify when to open the relay to stop high current from the battery (timeOpenRelay > windowEnd) !

unsigned int C[8];
unsigned long D1,D2;

double p_sea = 101325.0;
double altitude, lastMeanAltitude;
double altitudeVector[5];
byte indexAltitudeVector = 0;
double temperature;
unsigned long time_value, time_value_ms, timerInit = 0;

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;

byte address = 0x77; //Address of the MS5611 sensor.
byte counter=0, counterCloseSD=0, counterLed=0;

const byte chipSelect = 10;
const byte pinRelay = 8, pinLed = 3, pinLiftOff = 5;

bool onGround = true, apogee = false, testTimeWindow = false, FlagSignalSent = false;
bool setupOK = true;

FaBo9Axis fabo_9axis;

File myFile;


void setup(){
  pinMode(pinLed, OUTPUT);
  pinMode(pinLiftOff,INPUT_PULLUP);
  pinMode(pinRelay, OUTPUT);

  digitalWrite(pinRelay,HIGH);
  
  #ifdef DEBUG
    Serial.begin(115200);  //initialise serial communication
  #endif

  Wire.begin();  //initialize I2C
  Wire.setClock(400000);  //set SCL clock to 400kHz (100kHz standart)

  digitalWrite(pinLed, HIGH);

  ms5611_init(address);

  if (fabo_9axis.begin()) {                                         //initialise MPU9250
    DEBUG_PRINTLN("configured mpu9250");
  } else {
    DEBUG_PRINTLN("device error, mpu9250");
    setupOK = false;
  }

  fabo_9axis.configMPU9250(MPU9250_GFS_1000, MPU9250_AFS_16G);      //configure scale gyro & accel
  // look for MPU9250_CONFIG and MPU9250_ACCEL_CONFIG_2 in library to change the value of LPF (by default : 41Hz)

  if (SD.begin(chipSelect)){
    DEBUG_PRINTLN("configured SD writer");
  } else {
    DEBUG_PRINTLN("device error, SD writer");
    setupOK = false;
  }

  if (myFile = SD.open("DATA.txt", FILE_WRITE)){
    myFile.close();
  } else {
    DEBUG_PRINTLN("SD file problem!");
    myFile.close();
    setupOK = false;
  }

  while(digitalRead(pinLiftOff)){ //if lift-off wire is not connected, wait.
    delay(100);
  }

  if(setupOK == false) while(1);  //if an error occured in the setup, don't continue (LED is HIGH)
}

void loop(){
  time_value = micros();      
  time_value_ms = millis();
  counter++;
  counterLed++;


  //---------- HERE WE READ THE DATA FROM THE SENSORS AND WRITE THEM INTO THE SD CARD ----------
  fabo_9axis.readAccelXYZ(&ax,&ay,&az);
  fabo_9axis.readGyroXYZ(&gx,&gy,&gz);
  fabo_9axis.readMagnetXYZ(&mx,&my,&mz);

  //don't need to wait 9ms till the conversion is finished in a function
  //counter goes from 1 to 8
  if(counter==1){
    ms5611_readTemperature(address), ms5611_askPressure(address);
  }
  else if(counter==4){   //is executed only if the above if isn't executed, change the value (=5) depending of the loop speed
    ms5611_readPressure(address), ms5611_askTemperature(address);
    
    altitude = calcHeight(&temperature); //calculate altitude and update of the temperature
    
    if(indexAltitudeVector == 5) indexAltitudeVector = 0;
    altitudeVector[indexAltitudeVector] = altitude;
    indexAltitudeVector++;   
  }
  else if(counter==6) counter = 0, counterCloseSD++;
  
  writeToSD();
  //--------------------------------------------------------------------------------


  //--------------------  HERE WE DETECT THE LIFT-OFF --------------------
  if(digitalRead(pinLiftOff) && onGround){   //detection of lift-off and initialisation of timer, it happens just once.
    timerInit = millis();
    onGround = false;
  }
  //--------------------------------------------------------------------------------


  //---------- HERE WE TEST THE APOGEE, THE TIME WINDOW, AND SEE IF WE OPEN THE PARACHUTE ----------
  if(indexAltitudeVector == 5){
    apogee = detectionApogee();         //test if we are at the apogee
  
    if((time_value_ms-timerInit>windowBegin) && ((time_value_ms-timerInit)<windowEnd) && (timerInit>0)) testTimeWindow = true; //test if we are in the time Window
    else if(((time_value_ms-timerInit)>=windowEnd) && (timerInit>0)){                                         //if the window is over, force the opening of the parachute
      apogee = true;
    }
    else testTimeWindow = false;
    
    if(!onGround && testTimeWindow && apogee && !FlagSignalSent){  //open the parachute
      digitalWrite(pinRelay,LOW);                                       //open the parachute
      myFile.print("apogee;"), myFile.println(time_value_ms-timerInit); //write into the SD card the moment of opening of parachute
      FlagSignalSent = true;                                           //the relay is closed, no necessity to send again the ejection signal
    }
  }
  //--------------------------------------------------------------------------------


  //--- AFTER THE EJECTION HAS BEEN DONE, OPEN THE RELAY AGAIN
  if((time_value_ms-timerInit>timeOpenRelay) && (timerInit>0)) digitalWrite(pinRelay,HIGH);   //stop the high current from the battery
  //---

  
  //------------------------------HERE THE LED IS BLINKING ------------------------------
  if((counterLed == 15) && !onGround){                          //blinking the fast after lift-off
    counterLed = 0, digitalWrite(pinLed, !digitalRead(pinLed));
  } else if((counterLed == 40) && onGround){                    //blinking the led slow before lift-off
    counterLed = 0, digitalWrite(pinLed, !digitalRead(pinLed));
  }
  //------------------------------------------------------------------------------------------

}


bool detectionApogee(){
  unsigned long meanAltitude=0;
  static unsigned long meanLastAltitude;
  bool detection;
  
  for(byte i=0; i<5; i++){
    meanAltitude = meanAltitude + altitudeVector[i];
  }
  meanAltitude = meanAltitude / (float)5.0;
  
  if(meanAltitude <= meanLastAltitude) detection=1;
  else detection=0;

  meanLastAltitude = meanAltitude;
  return detection;
}

void writeToSD(){
  //try difference between myFile.print and myFile.write
  //try write buffer;
  //try writing without character ";"
  if(counterCloseSD==7){   //close and open the file each time countercloseSD reach 7, ie 5 pressure measurment
    counterCloseSD = 0;
    myFile.close();
    if(myFile = SD.open("DATA.txt", FILE_WRITE));
    else DEBUG_PRINTLN("SD file problem 2!");
  }
  //myFile.write(buf,sizeof(buf));
  
  myFile.print(time_value); myFile.print(";");
  myFile.print(ax); myFile.print(";"); myFile.print(ay); myFile.print(";"); myFile.print(az); myFile.print(";");
  myFile.print(gx); myFile.print(";"); myFile.print(gy); myFile.print(";"); myFile.print(gz); myFile.print(";");
  myFile.print(mx); myFile.print(";"); myFile.print(my); myFile.print(";");
  if(counter == 5){
    myFile.print(mz); myFile.print(";");
    myFile.print(altitude); myFile.print(";"); myFile.println(temperature);  //myFile.write("\r\n");
  } else{
    myFile.println(mz);
  }
}

double calcHeight(double *temperature){
  //Calculating temperature and pressure as the datasheet says.
  int64_t dT = D2 - C[5] * 256LL;
  signed long TEMP = 2000L + dT * C[6] / 8388608L;           
  int64_t OFF = C[2] * 65536LL + (C[4] * dT) / 128LL;
  int64_t SENS = C[1] * 32768LL + (C[3] * dT) / 256LL;
  unsigned long P = (D1 * SENS / 2097152UL - OFF) / 32768UL;

  *temperature = (float)TEMP / 100.0;
  //TODO maybe change the equation so we dont depend on temp.
  double height = (pow((p_sea / P),1 / 5.257) - 1)*(*temperature + float(273.15)) / 0.0065;     // http://keisan.casio.com/exec/system/1224585971
  return height;
}

void ms5611_init(byte address){
  Wire.beginTransmission(address);
  Wire.write(0x1E); //Restart command.
  Wire.endTransmission();
  
  delay(3); //Waiting until MS5611 restarts.
 
  for(int i=1; i<8; i++){ //Reading C1 to C7 values from PROM.
    Wire.beginTransmission(address);
    Wire.write(0xA0+(i*2));
    Wire.endTransmission();
   
    Wire.requestFrom(address, 2);
    while(Wire.available() < 2);
    C[i] = Wire.read()<<8 | Wire.read();
  }
 
}

void ms5611_askPressure(byte address){
  /* D1 VALUE PRESSURE */
  Wire.beginTransmission(address);
  Wire.write(0x48); //Convert D1 command.
  Wire.endTransmission();
}

void ms5611_readPressure(byte address){
  /* D1 VALUE PRESSURE */ // wait 9ms after the ask
  Wire.beginTransmission(address);
  Wire.write(0x00); //ADC read command (read D1 value).
  Wire.endTransmission();

  Wire.requestFrom(address, 3);
  while(Wire.available() < 3);
  D1 = 0;
  D1 |= (unsigned long)Wire.read()<<16;
  D1 |= (unsigned long)Wire.read()<<8;
  D1 |= (unsigned long)Wire.read();
}

void ms5611_askTemperature(byte address){
  /* D2 VALUE TEMPERATURE */
  Wire.beginTransmission(address);
  Wire.write(0x58); //Convert D2 command. It's possible to reduce OSR, especially for temperature (oversampling rate for a faster conversion time)
  Wire.endTransmission();
}

void ms5611_readTemperature(byte address){
  /* D2 VALUE TEMPERATURE */
  Wire.beginTransmission(address);
  Wire.write(0x00); //ADC read command (read D2 value);
  Wire.endTransmission();
 
  Wire.requestFrom(address, 3);
  while(Wire.available() < 3);
  D2 = 0;
  D2 |= (unsigned long)Wire.read()<<16;
  D2 |= (unsigned long)Wire.read()<<8;
  D2 |= (unsigned long)Wire.read();
}

/*
Quick explanation of the code

As soon as the lift-off detection cable is take out a timer is started.
When the timer enters in the time window specified in the beginning of the code, the ejection of the parachute is enabled. It will happen when apogee occurs.
The apogee is detected by the pressure acquisition through the altitude. The apogee is detected when the altitude start to decrease.
At the end of the time Window, if the apogee isn't still detected, the ejection of the parachute is forced !

All the data (coming from the accelerometer, gyroscope, magnetometer, temperature and pressure) are stored into the SD card at a period approximatively of 8ms (except for temperature and pressure).
The storage begins when the mother card is powered and ends when the power is turn off.
*/
