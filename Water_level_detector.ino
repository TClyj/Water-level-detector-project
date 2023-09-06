/**
 *   @file   EE_474_Lab_4_Project.ino
 *   @author    Yijia Lu, Kevin Zhao
 *   @date      28-May-2022
 *   @brief   File that contain the Tasks to run our final project
 *   
 *  We include the FreeRTOS library to make sure all our code is runing under the FreeRTOS environment
 *  Also, we use Servo library which can control the Servo motor. 
 *  We are going to make a water level alarm and its supporting facility. We will detect the 
 *  real time  water level by using a water level sensor and output a noise through the 
 *  speaker while the output voltage of the water sensor is over the chosen limitation. The limitation 
 *  will be water level and we will display its number on a 7 segment display.
 *  For the supporting facility part, a joystick will be used and users can interact with it. 
 *  The sticker can control a motor which can open a gate to release the water. When the output 
 *  voltage of the water sensor is high, the joystick will have a new function and we can quickly 
 *  click the switch in the sticker to make the motor run to the open gate state.

 */

#include <Arduino_FreeRTOS.h>
#include <Servo.h>  //Servo library to control the servo motor

#define SENSOR_POWER 41 //power pin for the water level sensor
#define SENSOR_READ 43 //Pin which read the sensor's output
#define SENSOR_MIN 0 //Minimum number sensor can read
#define SENSOR_MAX 520 //Maximum number sensor can read

#define BIT_X A1 // pin A1 Joystick X axis
#define BIT_SW A2 // pin A2 Joystick Y axis

#define MOTOR 31 //pin to control the motor

#define LED A7 //pin for LED in RT1
#define SPEAKER A6 //pin for speaker

Servo myservo;

int level = 0; // global int of water level, which control by the sensor input
int x = 0; //global int of x, which control by the jystick
int switchState; //global int of the switch's State, which will be 0 or 1

//array which control the 7 segment display
byte seven_seg_digits[10][7] = { { 1,1,1,1,1,1,0 },  // = 0
                                 { 0,1,1,0,0,0,0 },  // = 1
                                 { 1,1,0,1,1,0,1 },  // = 2
                                 { 1,1,1,1,0,0,1 },  // = 3
                                 { 0,1,1,0,0,1,1 },  // = 4
                                 { 1,0,1,1,0,1,1 },  // = 5
                                 { 1,0,1,1,1,1,1 },  // = 6
                                 { 1,1,1,0,0,0,0 },  // = 7
                                 { 1,1,1,1,1,1,1 },  // = 8
                                 { 1,1,1,0,0,1,1 }   // = 9
                                                           };

/**
 * @Basic setup for our project
 * Setup and create all the Task we need, also start the serial window
 * @param no param
 */
void setup() {
  Serial.begin(9600);

  xTaskCreate(
    RT1
    ,  "Blink" 
    ,  128  
    ,  NULL
    ,  3  // Priority
    ,  NULL
  );

  xTaskCreate(
    TaskReadValue
    ,  "Read water level"
    ,  256
    ,  NULL
    ,  1
    ,  NULL
  );

  xTaskCreate(
    TaskJoyStick
    ,  "update X value"
    ,  256
    ,  NULL
    ,  1
    ,  NULL
  );

  xTaskCreate(
    TaskMotor
    ,  "let motor work"
    ,  256
    ,  NULL
    ,  1
    ,  &TaskHandle_Motor
  );

  xTaskCreate(
    TaskUrgency
    ,  "start urgency function"
    ,  256
    ,  NULL
    ,  1
    ,  NULL
  );

  xTaskCreate(
    TaskDisplayLevel
    ,  "display level number"
    ,  256
    ,  NULL
    ,  1
    ,  NULL
  );

}


/**
 * @Loop funtion
 * We don't need to fill anything in it since we have our loop in Tasks
 * @param no param
 */
void loop() {}

/**
 * @Task for LED blink
 * We set the LED pin as output and switch its state by setting delay.
 * @param no param
 */
void RT1(void *pvParameters) {
  pinMode(LED, OUTPUT);
  for (;;) {
    digitalWrite(LED, HIGH);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    digitalWrite(LED, LOW);
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

/**
 * @Task for raed value from sensor
 * We will control the power of sensor to make sure it read number and read data from the other pin.
 * However, after test, we found our water level sensor was shorted. The expected data when sensor
 * touch nothing is 0, but what we get is around 500, which means its resistor is pretty low.
 * So we use a potentiometer to simulate the input data value. 
 * After we read the value, we will transfer this value to a water level data.
 * @param no param
 */
void TaskReadValue(void *pvParameters) {
  pinMode(SENSOR_POWER, OUTPUT);
  digitalWrite(SENSOR_POWER, LOW);
  int value = 0;
  int sensorValue = 0;
  for (;;) {
    digitalWrite(SENSOR_POWER, HIGH);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    value = analogRead(SENSOR_READ);
    digitalWrite(SENSOR_POWER, LOW);
    Serial.println(value);

    sensorValue = analogRead(A15);
    //Serial.println(sensorValue);
    vTaskDelay(50/portTICK_PERIOD_MS);
    
    level = map(sensorValue, 0, 500, 0, 4);
    //Serial.println(level);
  }
}

/**
 * @Task to control the joystick
 * We set Pins for x axis and switch as inputs and read their value and update the 
 * global x and switchState int.
 * @param no param
 */
void TaskJoyStick(void *pvParameters) {
  pinMode(BIT_X, INPUT);
  pinMode(BIT_SW, INPUT);
  digitalWrite(BIT_SW, HIGH);
  for(;;){
    x = analogRead(BIT_X);
    switchState = digitalRead(BIT_SW);
    //Serial.println(x);
  }
}


/**
 * @Task to control the Servo motor
 * We will use servo library in this part. First we set our motor pin by using .attach
 * and then we can control the servo motor by using .write("The angle we want").
 * For motor in lab kit it can rotate from 0 to 180 degrees.
 * @param no param
 */
void TaskMotor(void *pvParameters) {
  myservo.attach(MOTOR);
  myservo.write(0);
  int angle = 0;
  for (;;) {
    //Serial.println(5);
    if ((x > 700) && (angle < 175)) {
      angle += 5;
      myservo.write(angle);
      vTaskDelay(10 / portTICK_PERIOD_MS);
    } else if ((x < 200) && (angle > 5)) {
      angle -= 5;
      myservo.write(angle);
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }    
  }
}

/**
 * @Task which includes all the urgency function
 * When the global value of water level is greater than 3, the speaker will work and now we can 
 * use the switch to quickly run the motor which control our water gate. For the speaker 
 * we just use the tone function. I tried to use a timer to control the speaker 
 * but when we use a timer, a vector in the timer has the same name as a vector 
 * in the Servo library and our code will not compile. 
 * @param no param
 */
void TaskUrgency(void *pvParameters) {
  for (;;) {
    //Serial.println(9);
    if (level > 3) {
      tone(SPEAKER, 1000, 100);
      vTaskDelay(10/portTICK_PERIOD_MS);
      if (switchState == LOW) {
        myservo.write(90);
        vTaskDelay(10/portTICK_PERIOD_MS);
        myservo.write(180);   
      }      
    } else if (level <= 3) {
      noTone(SPEAKER);
    }
  }
}


/**
 * @Task to display water level in 7 segments display
 * Set all the pins which connected between bread board and 7 segment display. 
 * Then run a for loop to display the number based on the global int water level number.
 * @param no param
 */
void TaskDisplayLevel(void *pvParameters) {
  pinMode(2, OUTPUT);  
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  for (;;) {
    byte pin = 2;
    for (byte segCount = 0; segCount < 7; ++segCount) {
      digitalWrite(pin, seven_seg_digits[level][segCount]);
      pin++;
    }
  }
}
