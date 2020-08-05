/*
Kovuru Sai Charan
Cold chain management
to be deployed on Helium Developer kit 
*/


#include "LoRaWAN.h"
#include "TimerMillis.h"
#include <CayenneLPP.h>
#include <MicroNMEA.h>
#include <LSM6DSOSensor.h>
#include <LIS2DW12Sensor.h>
#include <LIS2MDLSensor.h>
#include <LPS22HHSensor.h>
#include <STTS751Sensor.h>
#include <HTS221Sensor.h>


#define thresh_idle_time 8 //threshold for idle time above which an alert is sent 

const char *devEui = "FILL_ME_IN";
const char *appEui = "FILL_ME_IN";
const char *appKey = "FILL_ME_IN";

#define RESET_PIN 7

const uint32_t TX_INTERVAL = 60000; // 60 Seconds
TimerMillis timer_send;

float longitude_mdeg;
float latitude_mdeg;
long alt;
float prev_longitude_mdeg;
float prev_latitude_mdeg;
long _speed;
long _course;

uint8_t door_state = 0; 
/*
door_state == 0 door closed
           == 1 door open
*/

uint8_t alert_temp = 0; // set to one if the temperature is not in the desired window
uint8_t alert_idle = 0; // set to one if the vehicle is idle for more than the defined thershold

// Refer to serial devices by use
HardwareSerial &console = Serial;
HardwareSerial &gps = Serial1;

// Sensors
LSM6DSOSensor *AccGyr;
LPS22HHSensor *PressTemp;
HTS221Sensor *HumTemp;

CayenneLPP lpp(51);
static volatile bool uplink_attempted;

// MicroNMEA library structures
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

bool ledState = LOW;
volatile bool ppsTriggered = false;

void ppsHandler(void);

void ppsHandler(void) { ppsTriggered = true; }

void gpsHardwareReset() {
  // Empty input buffer
  while (gps.available())
    gps.read();

  // reset the device
  digitalWrite(RESET_PIN, LOW);
  delay(50);
  digitalWrite(RESET_PIN, HIGH);

  // wait for reset to apply
  delay(2000);
}

void setupGPS() {
  delay(3000);
  console.begin(115200); // console
  Serial.println("Starting GPS Example...");

  gps.begin(9600); // gps

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, ledState);

  // Start the module
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, HIGH);
  Serial.println();
  Serial.println("Resetting GPS module ...");
  gpsHardwareReset();
  Serial.println("done.");

  // Change the echoing messages to the ones recognized by the MicroNMEA library
  MicroNMEA::sendSentence(gps, "$PSTMSETPAR,1201,0x00000042");
  MicroNMEA::sendSentence(gps, "$PSTMSAVEPAR");

  // Reset the device so that the changes could take plaace
  MicroNMEA::sendSentence(gps, "$PSTMSRR");

  delay(4000);

  // clear serial buffer
  while (gps.available())
    gps.read();

  pinMode(6, INPUT);
  attachInterrupt(digitalPinToInterrupt(6), ppsHandler, RISING);
}

void readGPS() {
  // If a message is received
  if (ppsTriggered) {
    ppsTriggered = false;
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);

    // Clear Payload
    lpp.reset();

    latitude_mdeg = nmea.getLatitude();
    longitude_mdeg = nmea.getLongitude();
    nmea.getAltitude(alt);
    _speed = nmea.getSpeed();
    _course = nmea.getCourse();

    lpp.addAnalogInput(7, (float)_speed / 1000);    //add speed to payload
    lpp.addAnalogInput(8, (float)_course / 1000);   //add course to payload
    lpp.addGPS(6, latitude_mdeg / 1000000, longitude_mdeg / 1000000,   // add lat, long, and alt to payload
               alt / 1000);

    nmea.clear();
  }

  // While the message isn't complete
  while (!ppsTriggered && gps.available()) {
    // Fetch the character one by one
    char c = gps.read();
    // Serial.print(c);
    // Pass the character to the library
    nmea.process(c);
  }
}

void async_timer_send() {
  if (LoRaWAN.joined() && !LoRaWAN.busy()) {
    // Send Packet
    Send_Sensor_data();
    //if(alert_temp == 1)
    //lpp.addDigitalInput(9, alert_temp);
    //if(alert_idle == 1)
    //lpp.addDigitalInput(10, alert_idle);

    lpp.addDigitalOutput(11, door_state);
    LoRaWAN.sendPacket(1, lpp.getBuffer(), lpp.getSize());
    alert_temp = 0;  // clear alert after sending the packet
    alert_idle = 0;
    uplink_attempted = true;
  }
}

void async_temp_alert_send() {
  if (LoRaWAN.joined() && !LoRaWAN.busy()) {
    // Send Packet
    Send_Sensor_data(); // add temperature, pressure, humidity to payload
    
    lpp.addDigitalInput(9, alert_temp);  // add temperature alert to payload

    lpp.addDigitalOutput(11, door_state);  // add door state to payload for downlink
    LoRaWAN.sendPacket(1, lpp.getBuffer(), lpp.getSize());   
    alert_temp = 0;  // clear alert after sending the packet
    alert_idle = 0;
    uplink_attempted = true;
  }
}


//Sensor variables
float humidity = 0;
float temperature = 0;
float pressure = 0;
int32_t accelerometer[3];
int32_t gyroscope[3];


void readSensors() {
  // Read humidity and temperature.

  HumTemp->GetHumidity(&humidity);
  HumTemp->GetTemperature(&temperature);

  // Read pressure and temperature.
  
  PressTemp->GetPressure(&pressure);

  // Read accelerometer and gyroscope.
  
  AccGyr->Get_X_Axes(accelerometer);
  AccGyr->Get_G_Axes(gyroscope);

}


void Send_Sensor_data(){

  // Clear Payload
  //lpp.reset();
  // Pack Packload
  lpp.addTemperature(1, temperature);
  lpp.addRelativeHumidity(2, humidity);
  lpp.addBarometricPressure(3, pressure); 
  //lpp.addAccelerometer(4, accelerometer[0], accelerometer[1], accelerometer[2]);
  //lpp.addGyrometer(5, gyroscope[0], gyroscope[1], gyroscope[2]);
  
  // Debug Print Data 
  Serial.print("| Hum[%]: ");
  Serial.print(humidity, 2);
  Serial.print(" | Temp[C]: ");
  Serial.print(temperature, 2);
  Serial.print(" | Pres[hPa]: ");
  Serial.print(pressure, 2);
  Serial.print(" | Acc[mg]: ");
  Serial.print(accelerometer[0]);
  Serial.print(" ");
  Serial.print(accelerometer[1]);
  Serial.print(" ");
  Serial.print(accelerometer[2]);
  Serial.print(" | Gyr[mdps]: ");
  Serial.print(gyroscope[0]);
  Serial.print(" ");
  Serial.print(gyroscope[1]);
  Serial.print(" ");
  Serial.println(gyroscope[2]);
  
}



void setup(void) {
  setupGPS();

  Serial.begin(115200);

  while (!Serial) {
  }

  // Initialize I2C bus.
  Wire.begin();

  // Enable Sensors
  AccGyr = new LSM6DSOSensor (&Wire);
  AccGyr->Enable_X();
  AccGyr->Enable_G();
  PressTemp = new LPS22HHSensor(&Wire);
  PressTemp->Enable();
  HumTemp = new HTS221Sensor (&Wire);
  HumTemp->Enable();

  
  // INDIA Region
  LoRaWAN.begin(IN865);
  // Helium SubBand
  LoRaWAN.setSubBand(2);
  // Disable Adaptive Data Rate
  LoRaWAN.setADR(false);
  // Set Data Rate 1 - Max Payload 53 Bytes
  LoRaWAN.setDataRate(1);
  // Device IDs and Key
  LoRaWAN.joinOTAA(appEui, appKey, devEui);

  Serial.println("JOIN( )");

  while (!LoRaWAN.joined() && LoRaWAN.busy()) {
    Serial.println("JOINING( )");
    delay(5000);
  }
  Serial.println("JOINED( )");

  // Start Continuous Uplink Timer
  timer_send.start(async_timer_send, 0, TX_INTERVAL);

  
}

unsigned long current_time;
unsigned long previous_time = 0;
int i = 0; /*counter to remove the temp spike noise*/
unsigned long current_t_GPS;
unsigned long prev_t_GPS = 0;
int idle_time = 0; /*idle time of vehicle in hours*/
unsigned long curr_alert_t;
unsigned long last_alert_t = 0;


void loop(void) {
  if (uplink_attempted) {
    Serial.print("TRANSMIT( ");
    Serial.print("TimeOnAir: ");
    Serial.print(LoRaWAN.getTimeOnAir());
    Serial.print(", NextTxTime: ");
    Serial.print(LoRaWAN.getNextTxTime());
    Serial.print(", MaxPayloadSize: ");
    Serial.print(LoRaWAN.getMaxPayloadSize());
    Serial.print(", DR: ");
    Serial.print(LoRaWAN.getDataRate());
    Serial.print(", TxPower: ");
    Serial.print(LoRaWAN.getTxPower(), 1);
    Serial.print("dbm, UpLinkCounter: ");
    Serial.print(LoRaWAN.getUpLinkCounter());
    Serial.print(", DownLinkCounter: ");
    Serial.print(LoRaWAN.getDownLinkCounter());
    Serial.println(" )");
    Serial.print("Latitude (deg): ");
    Serial.print(latitude_mdeg / 1000000., 6);
    Serial.print(" Longitude (deg): ");
    Serial.print(longitude_mdeg / 1000000., 6);
    Serial.print("  Altitude (m): ");
    if (nmea.getAltitude(alt))
      Serial.println(alt / 1000., 3);
    else
      Serial.println("not available");

    uplink_attempted = false;
  }

  
  readGPS();
  readSensors();
  
  if(temperature > 8 || temperature < 2){
    current_time = millis();
    if(current_time - previous_time < 1000){    
       i++;
    }
    else{
      i = 0;
    }
   
    if(i > 120){
      curr_alert_t = millis();
      if((curr_alert_t - last_alert_t >  60000) || last_alert_t == 0 ){  //send alert again after every 5 mins if problem persists
        alert_temp = 1;
        async_timer_send();   //warn that the temp in freezer is out of range 
        i = 0;
        last_alert_t = curr_alert_t;
      }
    }

    previous_time = current_time;
  }

  current_t_GPS = millis();
  
  if( _speed != 0 ){  
    prev_t_GPS = current_t_GPS;
  }
  
  if(current_t_GPS - prev_t_GPS > 900000){
    idle_time += 0.25; //store the current idle time of vehicle in hours
    
    if(idle_time > thresh_idle_time){
      alert_idle = 1;
      async_timer_send();   //sends alert if idle time is greater than 1 hour
    }
    prev_t_GPS = current_t_GPS;
  }
  if(door_state == 1){
    Serial.println("Door open!!");  
  }
  
}
