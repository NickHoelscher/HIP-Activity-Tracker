#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include <Adafruit_GPS.h>
#include <Arduino_JSON.h>

#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
#include <SoftwareSerial.h>
#endif

#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"
#define DEBUG 0
#define LED_PIN 7



// Connect the GPS Power pin to 3.3V
// Connect the GPS Ground pin to ground
// Connect the GPS TX (transmit) pin to Digital 9
// Connect the GPS RX (receive) pin to Digital 6

// you can change the pin numbers to match your wiring:
SoftwareSerial mySerial(9, 6);
Adafruit_GPS GPS(&mySerial);

// Create the bluefruit object, either software serial...uncomment these lines

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
Adafruit_BluefruitLE_UART ble(Serial1, BLUEFRUIT_UART_MODE_PIN);

void error(const __FlashStringHelper*err) {
#if DEBUG
  Serial.println(err);
#endif
  while (1);
}


/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
int step_count = 0;
bool already_counted = false;
float total_cal_burned = 0;
void displaySensorDetails(void)
{
  sensor_t sensor;
  accel.getSensor(&sensor);
#if DEBUG
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");
  Serial.println("------------------------------------");
  Serial.println("");                
#endif
  delay(500);

}
void print_accel_values(sensors_event_t event)
{
  /* Display the results (acceleration is measured in m/s^2) */
#if DEBUG
  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  "); Serial.println("m/s^2 ");         //print in xyz acceleration
#endif

}

float total_accel_calc(sensors_event_t event)
{


  float totaccel = sqrt(((event.acceleration.x) * (event.acceleration.x)) + ((event.acceleration.y) * (event.acceleration.y)) + ((event.acceleration.z) * (event.acceleration.z)));    //formula for total acceleration vector
  return totaccel;
}

float get_avg_accel(sensors_event_t event, int points_to_average)
{

  float total_accel = 0;

  float sum_vect = 0;

  float avg_vect = 0;                                    //set up variables

  for (int i = 1; i <= points_to_average; i++)
  {
    accel.getEvent(&event);
    total_accel = total_accel_calc(event);
    sum_vect = total_accel + sum_vect;                              //sum vectors
    delay(500 / points_to_average);
  }

  avg_vect = sum_vect / points_to_average;                              //Average of the values


  return avg_vect;

}
bool detect_step(float average_accel, float threshold, int gravity)            //formula to detect steps
{
  float diff = abs(average_accel - gravity);
  if (diff > threshold)
  {
    return true;
  }
  else
  {
    return false;
  }
}



bool getUserInput(char buffer[], uint8_t maxSize)
{
  // timeout in 100 milliseconds
  TimeoutTimer timeout(100);

  memset(buffer, 0, maxSize);
  while ( (!Serial.available()) && !timeout.expired() ) {
    delay(1);
  }

  if ( timeout.expired() ) return false;

  delay(2);
  uint8_t count = 0;
  do
  {
    count += Serial.readBytes(buffer + count, maxSize);
    delay(2);
  } while ( (count < maxSize) && (Serial.available()) );

  return true;
}


void setup(void)
{
 pinMode(LED_PIN, OUTPUT);

 
  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  
#ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif
  //Serial.begin(9600);
#if DEBUG
  Serial.println("Accelerometer Test"); Serial.println("");
#endif
  /* Initialise the sensor */
  if (!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
#if DEBUG
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
#endif
    while (1);
  }
  else {
#if DEBUG
    Serial.println("yes");
#endif
  }
  /* Display some basic information on this sensor */
  displaySensorDetails();

#if DEBUG
  Serial.println(F("Adafruit Bluefruit Command Mode Example"));
  Serial.println(F("---------------------------------------"));
#endif

  /* Initialise the module */
#if DEBUG
  Serial.print(F("Initialising the Bluefruit LE module: "));
#endif

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
#if DEBUG
  Serial.println( F("OK!") );
#endif

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
#if DEBUG
    Serial.println(F("Performing a factory reset: "));
#endif
    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

#if DEBUG
  Serial.println("Requesting Bluefruit info:");
#endif
  /* Print Bluefruit information */
  ble.info();

#if DEBUG
  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
  Serial.println(F("Then Enter characters to send to Bluefruit"));
  Serial.println();
#endif

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
    delay(500);
  }

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
#if DEBUG
    Serial.println(F("******************************"));
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
#endif
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
#if DEBUG
    Serial.println(F("******************************"));
#endif
  }

    // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time

  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
//  GPS.sendCommand(PGCMD_ANTENNA);

}
uint32_t timer = millis();
void loop(void)
{
  
char c = GPS.read();

    // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
//    Serial.println(GPS.parse(GPS.lastNMEA()));   // this also sets the newNMEAreceived() flag to false

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {

  // Light the LED solid if there's a GPS fix, otherwise flash it on and off once a second.
  if (GPS.fix) {
    digitalWrite(LED_PIN, HIGH);
  }
  else {
    // No fix, blink the LED once a second and stop further processing.
    digitalWrite(LED_PIN, (millis()/1000) % 2);
    return;
  }

  ble.print("AT+BLEUARTTXF=");
  ble.print(step_count);
  ble.print(","); 
  ble.print(GPS.latitudeDegrees, 4);
  ble.print(",");
  ble.print(GPS.longitudeDegrees, 4);
  ble.print("\\n\n");

//  Serial.print("AT+BLEUARTTX=");
  Serial.print(step_count);
  Serial.print(","); 
  Serial.print(GPS.latitudeDegrees, 4);
  Serial.print(",");
  Serial.print(GPS.longitudeDegrees, 4);
  Serial.print("\n\n");

    
    timer = millis(); // reset the timer

  } 
  
  /* Get a new sensor event */
  sensors_event_t event;
  accel.getEvent(&event);
  float calories_b_step = .0375;
  int points_to_average = 8;                  //half of how many points are collected per second
  float threshold = 1.5;                     //distance from average vector to both thresholds
  const int gravity = 10;
  float upper = gravity + threshold;
  float lower = gravity - threshold;
  float avg_vect = 0;                                           //set up variable
  avg_vect = get_avg_accel(event, points_to_average);
  bool step_d = detect_step(avg_vect, threshold, gravity);
  if (step_d && !already_counted)                                //when the total acceleration crosses one of the thresholds, it must go back to count another step
  {
    step_count++;
    already_counted = true;
  }
  else if (!step_d && already_counted)
  {
    already_counted = false;
  }
  total_cal_burned = step_count * calories_b_step;                                        //formula to calculate calories burned
#if DEBUG
  Serial.print("avg_vect: "); Serial.print(avg_vect); Serial.print(" ");
  Serial.print("step_count: "); Serial.print(step_count); Serial.print(" ");
  Serial.print("upper: "); Serial.print(upper); Serial.print(" ");
  Serial.print("lower: "); Serial.print(lower); Serial.print(" ");
  Serial.print("Total_Calories_Burned: "); Serial.print(total_cal_burned);
  Serial.print(" \n ");
#endif


  // check response stastus
  if (! ble.waitForOK() ) {
#if DEBUG
    Serial.println(F("Failed to send?"));
#endif
  }

  delay(100);


  /* Note: You can also get the raw (non unified values) for */
  /* the last data sample as follows. The .getEvent call populates */
  /* the raw values used below. */

}

/*  0.0375 calories burned per step
    can use acceleration measured to tell if running or walking
    can calculate distance travelled
    average distance of stride?
    display some data with LEDs
    reset button
    wear on shoe
    machined learning identification of running vs walking
*/
