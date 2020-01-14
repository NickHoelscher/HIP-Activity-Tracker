#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
int step_count = 0;
bool already_counted = false;
float total_cal_burned = 0;
void displaySensorDetails(void)
{
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}
void print_accel_values(sensors_event_t event)
  {
  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");          //print in xyz acceleration


  }

float total_accel_calc(sensors_event_t event)                     
  {
     
     
     float totaccel = sqrt(((event.acceleration.x)* (event.acceleration.x))+ ((event.acceleration.y)*(event.acceleration.y)) + ((event.acceleration.z)*(event.acceleration.z)));          //formula for total acceleration vector
     return totaccel;
  }

float get_avg_accel(sensors_event_t event,int points_to_average)
{

  float total_accel = 0;

  float sum_vect = 0;

  float avg_vect = 0;                                    //set up variables
   
  for (int i = 1; i <= points_to_average; i++)
   {
    accel.getEvent(&event);
   total_accel = total_accel_calc(event);
    sum_vect = total_accel + sum_vect;                              //sum vectors
  delay(500/points_to_average);
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

void setup(void)
{
#ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif
  Serial.begin(9600);
  Serial.println("Accelerometer Test"); Serial.println("");

  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }

  /* Display some basic information on this sensor */
  displaySensorDetails();


}


void loop(void)
{
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
  if(step_d && !already_counted)                                 //when the total acceleration crosses one of the thresholds, it must go back to count another step
  {
    step_count++;
    already_counted = true;
  }
  else if(!step_d && already_counted)
  {
    already_counted = false;
  }
  total_cal_burned = step_count * calories_b_step;                                        //formula to calculate calories burned
    Serial.print("avg_vect: "); Serial.print(avg_vect); Serial.print(" ");
    Serial.print("step_count: "); Serial.print(step_count); Serial.print(" ");
    Serial.print("upper: "); Serial.print(upper); Serial.print(" ");
    Serial.print("lower: "); Serial.print(lower); Serial.print(" ");
    Serial.print("Total_Calories_Burned: "); Serial.print(total_cal_burned); 
    Serial.print(" \n ");
  /* Note: You can also get the raw (non unified values) for */
  /* the last data sample as follows. The .getEvent call populates */
  /* the raw values used below. */

}

/*  0.0375 calories burned per step
 *  can use acceleration measured to tell if running or walking
 *  can calculate distance travelled
 *  average distance of stride?
 *  display some data with LEDs
 *  reset button
 *  wear on shoe
 *  machined learning identification of running vs walking
 */
