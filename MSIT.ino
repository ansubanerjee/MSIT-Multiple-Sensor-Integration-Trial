// SUBMISSION OF ANSU BANERJEE, VIT UNIVERSITY, Email: workansubanerjee@gmail.com
// Utilizes ESP32 Microcontroller, SIM900A, Integrates SHT85, SEN0244, and DFRobot Gravity Analog pH Sensor for Arduino

// Packages Included:
#include <SoftwareSerial.h>
#include <Wire.h>
#include "SHTSensor.h"

// Constants:
SoftwareSerial gprsSerial(12, 13);  // RX, TX pins for SIM900A
SHTSensor sht; //Instance of SHT sensor
unsigned long delayTime;  //delay
int TDSPin = 4;   // GPIO4 for TDS sensor
int tds = 0;      // Initial value for tds
int PhPin = 5;    // GPIO5 for pH sensor
float ph = 0;     // Initial value for ph

//ph value derivation variable declaration
unsigned long int avgphval;
int buffer_pharr[10];
float phCalibration_value = 21.34 - 0.7; // Needs to be adjusted with calibration.

void setup() {
  Serial.begin(9600);   //BAUD RATE
  pinMode(TDSPin, INPUT);   //Analog pin declaration
  pinMode(PhPin, INPUT);    //Analog pin declaration
  Wire.begin();             //Initiates I2C

  //I2C Initialization Test
  Serial.println("Systems Test");
  bool statussht; 
  
  statussht = sht.init();
  if (!statussht) {
    Serial.println("SHT Initialization failure");
    while (1);
  }
  Serial.println("-- Default Test --");
  delayTime = 300000;

  
  sht.setAccuracy(SHTSensor::SHT_ACCURACY_HIGH); //Setting SHT Sensor Accuracy
  Serial.println();
}

//Function which reads data from SIM900A and writes in serial monitor.
void ShowSerialData() {
  while (gprsSerial.available() != 0) {
    Serial.write(gprsSerial.read());
  }
  delay(5000);
}

//Main Loop
void loop() {
  // Declaration of all Sensor Data to be fed into the SIM900A
  float temperature = sht.getTemperature();
  delay(100);
  float humidity = sht.getHumidity();
  delay(100);
  tds = analogRead(TDSPin);
  delay(100);
  readPH(PhPin, phCalibration_value);
  delay(100);
  delay(1000);

  //Print data in serial monitor
  Serial.println(temperature);
  Serial.println(humidity);
  Serial.println(tds);
  Serial.println(ph);
  delay(1000);

  // Setting up the GPRS Module
  if (gprsSerial.available()) {
    Serial.write(gprsSerial.read());
  }
  gprsSerial.println("AT");           //Response Check
  delay(1000);
  gprsSerial.println("AT+CPIN?");     //Readyness Check
  delay(1000);
  gprsSerial.println("AT+CREG?");     //Checks Network Registration status
  delay(1000);
  gprsSerial.println("AT+CGATT?");    //Checks if GPRS attached
  delay(1000);
  gprsSerial.println("AT+CIPSHUT");   //Resets IP Session
  delay(1000);
  gprsSerial.println("AT+CIPSTATUS"); //Checks current IP status
  delay(2000);
  gprsSerial.println("AT+CIPMUX=0");  //Single mode connection 
  delay(2000);
  
  ShowSerialData();
  
  gprsSerial.println("AT+CSTT=\"airtelgprs.com\"");   //Sets APN
  delay(1000);
  ShowSerialData();
  
  gprsSerial.println("AT+CIICR");                     //Brings up wireless connection
  delay(3000);
  ShowSerialData();
  
  gprsSerial.println("AT+CIFSR");                     //Gets Local IP Address
  delay(2000);
  ShowSerialData();
  
  gprsSerial.println("AT+CIPSPRT=0");                 //Data sending = Normal
  delay(3000);
  ShowSerialData();
  
  gprsSerial.println("AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",\"80\"");    //TCP connection on port 80
  delay(6000);
  ShowSerialData();
  
  gprsSerial.println("AT+CIPSEND");     //Prepares to send data
  delay(4000);
  ShowSerialData();

  String str = "GET https://api.thingspeak.com/update?api_key=FK49DTJ4TSU1PA1I&field1=" + String(temperature) + "&field2=" + String(humidity) + "&field3=" + String(tds) + "&field4=" + String(ph);     //Sends Data
  Serial.println(str);
  gprsSerial.println(str);    //Sends Data

  delay(4000);
  ShowSerialData();
  gprsSerial.println((char)26);   //End of text char
  delay(5000);
  gprsSerial.println();
  ShowSerialData();
 
  gprsSerial.println("AT+CIPSHUT");
  delay(100);
  ShowSerialData();
}


//Special function to read PH values
void readPH(int PhPin, float phCalibration_value) {
  unsigned long int avgphval = 0;

  // Read sensor values
  for (int i = 0; i < 10; i++) {
    buffer_pharr[i] = analogRead(PhPin);
    delay(30);
  }

  // Sort sensor values
  for (int i = 0; i < 9; i++) {
    for (int j = i + 1; j < 10; j++) {
      if (buffer_pharr[i] > buffer_pharr[j]) {
        float temp = buffer_pharr[i];
        buffer_pharr[i] = buffer_pharr[j];
        buffer_pharr[j] = temp;
      }
    }
  }

  // Average the middle 6 values
  for (int i = 2; i < 8; i++) {
    avgphval += buffer_pharr[i];
  }

  float volt = (float)avgphval * 5.0 / 1024 / 6; 
  ph = -5.70 * volt + phCalibration_value;
}
