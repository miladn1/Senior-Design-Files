// Teensy example/practice for FC data logger 
                    // so far on startup it plays the GOT theme song, then initializes the SD card and BMP388, then goes to mode 0, press the button low to 
                    // initiate ground mode where led light goes cray blue while data is being spit out onto the serial monitor & bluetooth device
                    // by the BMP388 and BNO08x (calibrated), can switch modes from the bluetooth device by typing a command or by pressing the button low.
#include <pitches.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <Servo.h>
#include <math.h>

Adafruit_BMP3XX bmp;  // create bmp
#define SEALEVELPRESSURE_HPA (1013.25)  // set ground pressure
#define BLUE 6  // set LED pins 
#define GREEN 4
#define RED 2
#define REST 0

// delta t variables 
float milliold = 0;
float millinew = 0;
float dt = 0;
SoftwareSerial hc06(28,29);   // Rx,Tx set pins for recieving and transmission 
int Bluetoothdata; // data sent from master device 
int tempo = 400;   // default 85 
int buzzer = 10;   // set buzzer pin 
int buttonpin = 9;
const int chipSelect = BUILTIN_SDCARD;
int redValue = 0;
int greenValue = 0;
int blueValue = 0;
/// Quaternion variables 
float SEq_1 = 1, SEq_2 = 0, SEq_3 = 0, SEq_4 = 0; // estimated orientation quanternion elements with initial conditions

// For SPI mode, we also need a RESET 
//#define BNO08X_RESET 5
// but not for I2C or UART
#define BNO08X_RESET -1

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

void setReports() {
  Serial.println("Setting desired reports");
  hc06.println("Setting desired reports");
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED)) {  // going to be used for orientation in flight 
    Serial.println("Could not enable gyroscope");
    hc06.println("Could not enable gyroscope");
  }
}
  // notes of the moledy followed by the duration.
  // a 4 means a quarter note, 8 an eighteenth , 16 sixteenth, so on
  // !!negative numbers are used to represent dotted notes,
  // so -4 means a dotted quarter note, that is, a quarter plus an eighteenth!!
int melody[] = {
  // Game of Thrones
  NOTE_G4,8, NOTE_C4,8, NOTE_DS4,16, NOTE_F4,16, NOTE_G4,8, NOTE_C4,8, NOTE_DS4,16, NOTE_F4,16,                                     //1 da da dada da da dada x4
  NOTE_G4,8, NOTE_C4,8, NOTE_DS4,16, NOTE_F4,16, NOTE_G4,8, NOTE_C4,8, NOTE_DS4,16, NOTE_F4,16, 
  NOTE_G4,8, NOTE_C4,8, NOTE_E4,16, NOTE_F4,16, NOTE_G4,8, NOTE_C4,8, NOTE_E4,16, NOTE_F4,16,
  NOTE_G4,8, NOTE_C4,8, NOTE_E4,16, NOTE_F4,16, NOTE_G4,8, NOTE_C4,8, NOTE_E4,16, NOTE_F4,16,
  NOTE_G4,-4, NOTE_C4,-4,                                                                                                           //5 daa daa

  NOTE_DS4,16, NOTE_F4,16, NOTE_G4,4, NOTE_C4,4, NOTE_DS4,16, NOTE_F4,16,                                                           //6 // da da da da da da daaa
  NOTE_D4,-1,
  NOTE_F4,-4, NOTE_AS3,-4,
  NOTE_DS4,16, NOTE_D4,16, NOTE_F4,4, NOTE_AS3,-4,
  NOTE_DS4,16, NOTE_D4,16, NOTE_C4,-1, 


  NOTE_G4,-4, NOTE_C4,-4,

  NOTE_DS4,16, NOTE_F4,16, NOTE_G4,4, NOTE_C4,4, NOTE_DS4,16, NOTE_F4,16, 
  NOTE_D4,-1, 
  NOTE_F4,-4, NOTE_AS3,-4,
  NOTE_DS4,16, NOTE_D4,16, NOTE_F4,4, NOTE_AS3,-4,
  NOTE_DS4,16, NOTE_D4,16, NOTE_C4,-1, 
  NOTE_G4,-4, NOTE_C4,-4,
  NOTE_DS4,16, NOTE_F4,16, NOTE_G4,4,  NOTE_C4,4, NOTE_DS4,16, NOTE_F4,16,

  NOTE_D4,-2,
  NOTE_F4,-4, NOTE_AS3,-4,
  NOTE_D4,-8, NOTE_DS4,-8, NOTE_D4,-8, NOTE_AS3,-8,
  NOTE_C4,-1,
  NOTE_C5,-2,
  NOTE_AS4,-2,
  NOTE_C4,-2,
  NOTE_G4,-2,
  NOTE_DS4,-2,
  NOTE_DS4,-4, NOTE_F4,-4, 
  NOTE_G4,-1,
  
  NOTE_C5,-2,
  NOTE_AS4,-2,
  NOTE_C4,-2,
  NOTE_G4,-2, 
  NOTE_DS4,-2,
  NOTE_DS4,-4, NOTE_D4,-4,
  NOTE_C5,8, NOTE_G4,8, NOTE_GS4,16, NOTE_AS4,16, NOTE_C5,8, NOTE_G4,8, NOTE_GS4,16, NOTE_AS4,16,
  NOTE_C5,8, NOTE_G4,8, NOTE_GS4,16, NOTE_AS4,16, NOTE_C5,8, NOTE_G4,8, NOTE_GS4,16, NOTE_AS4,16,
  
  REST,4, NOTE_GS5,16, NOTE_AS5,16, NOTE_C6,8, NOTE_G5,8, NOTE_GS5,16, NOTE_AS5,16,
  NOTE_C6,8, NOTE_G5,16, NOTE_GS5,16, NOTE_AS5,16, NOTE_C6,8, NOTE_G5,8, NOTE_GS5,16, NOTE_AS5,16,  
};

// notes in the melody:
int melody1[] = {
  NOTE_C5, NOTE_D5, NOTE_E5, NOTE_F5, NOTE_G5, NOTE_A5, NOTE_B5, NOTE_C6};
int duration = 500;  // 500 miliseconds

// sizeof gives the number of bytes, each int value is composed of two bytes (16 bits)
// there are two values per note (pitch and duration), so for each note there are four bytes
int notes = sizeof(melody) / sizeof(melody[0]) / 2;
// this calculates the duration of a whole note in ms
int wholenote = (60000 * 4) / tempo;
int divider = 0, noteDuration = 0;

void setup() {
  //configure the serial monitor & bluetooth module
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  hc06.begin(9600);
  //configure button
  pinMode(buttonpin,INPUT_PULLUP);
  // configure led buttons
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);
  digitalWrite(RED, LOW);
  digitalWrite(GREEN, HIGH);
  digitalWrite(BLUE, LOW);
  // initialiaze the SD card, BMP388, BNO08x, 
  Serial.print("Initializing SD card...");
  hc06.println("Initializing SD card...");
  
   // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    hc06.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("Card initialized.");
  hc06.println("Card initialized.");
  Serial.print("Initializing BMP388...");
  hc06.print("Initializing BMP388...");

  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
  //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
  //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    hc06.print("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  Serial.println("BMP388 Initialized.");
  hc06.print("BMP388 Initialized.");

  Serial.println("Initializing BNO08x...");
  hc06.println("Initializing BNO08x...");
  
  // Try to initialize!
  if (!bno08x.begin_I2C()) {
  //if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer!
  //if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip.");
    hc06.println("Failed to find BNO08x chip.");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Initialized.");
  hc06.println("BNO08x Initialized.");
  setReports();
  delay(100);

  // iterate over the notes of the melody.
  // Remember, the array is twice the number of notes (notes + durations)
  for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) {

    // calculates the duration of each note
    divider = melody[thisNote + 1];
    if (divider > 0) {
      // regular note, just proceed
      noteDuration = (wholenote) / divider;
    } else if (divider < 0) {
      // dotted notes are represented with negative durations!!
      noteDuration = (wholenote) / abs(divider);
      noteDuration *= 1.5; // increases the duration in half for dotted notes
    }

    // we only play the note for 90% of the duration, leaving 10% as a pause
    tone(buzzer, melody[thisNote], noteDuration * 0.9);

    // Wait for the specief duration before playing the next note.
    delay(noteDuration);

    // stop the waveform generation before the next note.
    noTone(buzzer);
  }
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}


void loop() {
  #define delayTime 10
  int noisecount = 0;
  // make a string for assembling the data to log:
  int data = 0;
  String dataString = "";
  if (hc06.available()){
    Bluetoothdata = hc06.read();
    Serial.write(Bluetoothdata);
  }
  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    hc06.print("sensor was reset ");
    setReports();
  }
  if (bno08x.getSensorEvent(&sensorValue)) {
  // in this demo only one report type will be received depending on FAST_MODE define (above)
  switch (sensorValue.sensorId) {
    case SH2_GYROSCOPE_CALIBRATED:
      millinew = millis();
      dt = millinew - milliold; 
      dt = dt/1000; // convert miliseconds to seconds
      milliold = millinew;
      // calculate preliminary values 
      float SEqDot[3];
      float halfSEq_1 = 0.5 * SEq_1;
      float halfSEq_2 = 0.5 * SEq_2;
      float halfSEq_3 = 0.5 * SEq_3;
      float halfSEq_4 = 0.5 * SEq_4;
      float twoSEq_1 = 2 * SEq_1;
      float twoSEq_2 = 2 * SEq_2;
      float twoSEq_3 = 2 * SEq_3;
      float twoSEq_4 = 2 * SEq_4;
      float x = sensorValue.un.gyroscope.x;
      float y = sensorValue.un.gyroscope.y;
      float z = sensorValue.un.gyroscope.z;
  
      //compute quaternion derivative from measured gyroscope or rate of change of quaternion from gyroscope. 
      SEqDot[0] = -halfSEq_2 * x - halfSEq_3 * y - halfSEq_4 * z;
      SEqDot[1] = halfSEq_1 * x + halfSEq_3 * z - halfSEq_4 * y;
      SEqDot[2] = halfSEq_1 * y - halfSEq_2 * z + halfSEq_4 * x;
      SEqDot[3] = halfSEq_1 * z + halfSEq_2 * y - halfSEq_3 * x;

      // Compute then integrate the estimated quanternion derrivative 
      SEq_1 += SEqDot[0] * dt;
      SEq_2 += SEqDot[1] * dt;
      SEq_3 += SEqDot[2] * dt;
      SEq_4 += SEqDot[3] * dt;

      // Normalize quaternions
      float norm = sqrt(SEq_1*SEq_1 + SEq_2*SEq_2 + SEq_3*SEq_3 + SEq_4*SEq_4);
      SEq_1 /= norm;
      SEq_2 /= norm;
      SEq_3 /= norm;
      SEq_4 /= norm;

      // Convert quaternion to euler 
      quaternionToEuler(SEq_1, SEq_2, SEq_3, SEq_4, &ypr, true); 
//        Serial.print(ypr.yaw);                Serial.print("\t");
//        Serial.print(ypr.pitch);              Serial.print("\t");
//        Serial.println(ypr.roll);
      break;
      }
  }
  switch(Bluetoothdata) {
    case 'A':
      analogWrite(RED, 0);
      analogWrite(GREEN, 0);
      analogWrite(BLUE, 255);
      delay(delayTime);
      //output data from IMU to serial port and blutooth module store data in file on sd card. 
      static long last = 0;
      long now = micros();
      Serial.print(now - last);             Serial.print("\t");
      hc06.print(now - last);             hc06.print("\t");
      last = now;
      Serial.print(sensorValue.status);     Serial.print("\t");  // This is accuracy in the range of 0 to 3
      hc06.print(sensorValue.status);     hc06.print("\t");
      Serial.print(ypr.yaw);                Serial.print("\t");
      hc06.print(ypr.yaw);                hc06.print("\t");
      dataString += String(ypr.yaw); 
      dataString += ","; 
      Serial.print(ypr.pitch);              Serial.print("\t");
      hc06.print(ypr.pitch);              hc06.print("\t");
      dataString += String(ypr.pitch); 
      dataString += ","; 
      Serial.println(ypr.roll);
      hc06.println(ypr.roll);
      dataString += String(ypr.roll); 
      dataString += ","; 

      
      //output data from bmp388 to serial port and blutooth module and store data in file on sd card
      if (! bmp.performReading()) {
        Serial.println("Failed to perform reading :(");
        hc06.println("Failed to perform reading :(");
        return;
      }
      Serial.print("Temperature = ");
      Serial.print((bmp.temperature*9/5)+32);
      Serial.print(" *F, ");
      data = ((bmp.temperature*9/5)+32);
      dataString += String(data); 
      dataString += ","; 
      Serial.print("Pressure = ");
      Serial.print(bmp.pressure / 100.0);
      Serial.print(" hPa, ");
      data = (bmp.pressure/100);
      dataString += String(data); 
      dataString += ","; 
      Serial.print("Approx. Altitude = ");
      Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
      Serial.println(" m");
      data = (bmp.readAltitude(SEALEVELPRESSURE_HPA));
      dataString += String(data); 
      dataString += "\n"; 
      Serial.println();
    
      hc06.print("Temperature = ");
      hc06.print((bmp.temperature*9/5)+32);
      hc06.print(" *F, ");
      hc06.print("Pressure = ");
      hc06.print(bmp.pressure / 100.0);
      hc06.print(" hPa, ");
      hc06.print("Approx. Altitude = ");
      hc06.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
      hc06.println(" m");
      hc06.println();
      analogWrite(RED, 0);
      analogWrite(GREEN, 0);
      analogWrite(BLUE, 0);

      // open the file.
      File dataFile = SD.open("datalog.txt", FILE_WRITE);
    
      // if the file is available, write to it:
      if (dataFile) {
        dataFile.println(dataString);
        dataFile.close();
      }  
      // if the file isn't open, pop up an error:
      else {
        Serial.println("error opening datalog.txt");
      }   
      break;
  default: 
    analogWrite(RED, 0);
    analogWrite(GREEN, 255);
    analogWrite(BLUE, 0);
    break;
  }
}
