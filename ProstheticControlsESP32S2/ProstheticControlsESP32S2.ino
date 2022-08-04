// These define's must be placed at the beginning before #include "TimerInterrupt_Generic.h"
// _TIMERINTERRUPT_LOGLEVEL_ from 0 to 4
// Don't define _TIMERINTERRUPT_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system.
#define TIMER_INTERRUPT_DEBUG       0
#define _TIMERINTERRUPT_LOGLEVEL_   4

#include "ESP32_S2_TimerInterrupt.h"
#include <ESP32AnalogRead.h>
//C++ libraries are not native to arduino, included within ESPIDF
#include <deque> 

int duty = 205;

//Hardware Timer Defintions
#define TIMER0_INTERVAL_MS  1000
#define TIMER0_PERIOD_MS    10

//General PWM Configuration
#define PWM_FREQ 333 //Sets 333 Hz
#define PWM_RESOLUTION 8 //Sets 8 bit resolution
#define MOTOR_HOME 50 //30% duty cycle @8 bit resolution

//Defines PWM Pins
#define FINGER_THUMB_GPIO 42
#define FINGER_INDEX_GPIO 41
#define FINGER_MIDDLE_GPIO 40
#define FINGER_RING_GPIO 39
#define FINGER_PINKY_GPIO 38

//Defines PWM Channels
#define FINGER_THUMB_CHANNEL 0
#define FINGER_INDEX_CHANNEL 1
#define FINGER_MIDDLE_CHANNEL 2
#define FINGER_RING_CHANNEL 3
#define FINGER_PINKY_CHANNEL 4

//Max Distance Drivable
#define FINGER_PINKY_DRIVE_LIMIT 189
#define FINGER_RING_DRIVE_LIMIT 159
#define FINGER_MIDDLE_DRIVE_LIMIT 180
#define FINGER_INDEX_DRIVE_LIMIT 205
#define FINGER_THUMB_DRIVE_LIMIT 129

//sets max deque size to 5 seconds, 500 samples [0,499]
#define MAX_DEQUE_SIZE 499

//Creates ADC objects
ESP32AnalogRead myoware1; //Myoware Sensor Inside Fore arm
ESP32AnalogRead myoware2; // Myoware Sensor Outside Fore arm
ESP32AnalogRead thumbFingerCurrent;
ESP32AnalogRead indexFingerCurrent;
ESP32AnalogRead middleFingerCurrent;
ESP32AnalogRead ringFingerCurrent;
ESP32AnalogRead pinkyFingerCurrent;

//Deque definitions
std::deque<double> myo1Deque;
std::deque<double> myo2Deque;

//used to to ensure vector size does not exceed 500 samples
int myo1DequeSize;
int myo2DequeSize;

//stores ADC reading Myoware sensor
volatile double myo1Volts;
volatile double myo2Volts;

//stores ADC reading current sensor
volatile double thumbCurrent;
volatile double indexCurrent;
volatile double middleCurrent;
volatile double ringCurrent;
volatile double pinkyCurrent;

//Timer Flags
volatile bool timer0 = false;

// Init ESP32 timer 0 and 1
ESP32Timer ITimer0(0);

//debugging for interrupt timer
volatile int timerDebug = 0;
bool IRAM_ATTR TimerHandler0(void * timerNo) { 
  timer0 = true;
  timerDebug = millis();
  //acquire signal input
  myo1Volts = myoware1.readVoltage();
  myo2Volts = myoware2.readVoltage();

  //current sensor inputs
  thumbCurrent = thumbFingerCurrent.readVoltage();
  indexCurrent = indexFingerCurrent.readVoltage();
  middleCurrent = middleFingerCurrent.readVoltage();
  ringCurrent = ringFingerCurrent.readVoltage();
  pinkyCurrent = pinkyFingerCurrent.readVoltage();

  return true;
}
bool maxedCurrent(){
  if (indexCurrent > 2.0){
    return true;
  }else if (thumbCurrent > 2.0){
    return true;
  }else if (thumbCurrent > 2.0){
    return true;
  }else if (thumbCurrent > 2.0){
    return true;
  }else  if (thumbCurrent > 2.0){
    return true;
  }else if (thumbCurrent > 2.0){
    return true;
  }
  return false;
}
//predefine functions
bool motorDriveCheck(uint8_t PwmChannel, uint8_t duty){
  if (PwmChannel == FINGER_THUMB_CHANNEL){
    if (duty < FINGER_THUMB_DRIVE_LIMIT){
      return true; 
    } else{
      return false;
    }
  } else if (PwmChannel == FINGER_INDEX_CHANNEL){
    if (duty < FINGER_INDEX_DRIVE_LIMIT){
      return true;
    } else {
      return false;
    }
  }else if (PwmChannel == FINGER_MIDDLE_CHANNEL){
    if (duty < FINGER_MIDDLE_DRIVE_LIMIT){
      return true;
    } else {
      return false;
    }
  } else if (PwmChannel == FINGER_RING_CHANNEL){
    if (duty < FINGER_RING_DRIVE_LIMIT){
      return true;
    } else {
      return false;
    }
  } else if (PwmChannel == FINGER_PINKY_CHANNEL){
    if (duty < FINGER_PINKY_DRIVE_LIMIT){
      return true;
    } else {
      return false; 
    }
  } else {
    return false;
  }
}

void setup() {
  Serial.begin(115200);

  //Set CPU clock to 80MHz
  setCpuFrequencyMhz(240); 

  //Hardware TImer Configuration
  while (!Serial);

  delay(100);

  Serial.print(F("\nStarting Argument_None on ")); Serial.println(ARDUINO_BOARD);
  Serial.println(ESP32_S2_TIMER_INTERRUPT_VERSION);
  Serial.print(F("CPU Frequency = ")); Serial.print(F_CPU / 1000000); Serial.println(F(" MHz"));

  // Interval in microsecs
  if (ITimer0.attachInterruptInterval(TIMER0_INTERVAL_MS * TIMER0_PERIOD_MS, TimerHandler0)){
    Serial.print(F("Starting  ITimer0 OK, millis() = ")); Serial.println(millis());
  }else
    Serial.println(F("Can't set ITimer0. Select another Timer, freq. or timer"));

  //attatch ADC object to GPIO pins
  myoware1.attach(1); //Myoware 1 is attatched to GPIO 1
  myoware2.attach(2); //Myoware 2 is attatched to GPIO 2

  //attach current sensors to gpoi
  thumbFingerCurrent.attach(3);
  indexFingerCurrent.attach(4);
  middleFingerCurrent.attach(5);
  ringFingerCurrent.attach(6);
  pinkyFingerCurrent.attach(7);

  //Configures PWM Frequency and Resulution for all channels
  for (int channel = 0; channel < 5; channel++){
     ledcSetup(channel, PWM_FREQ, PWM_RESOLUTION);
  }

  //Attaches PWM pins to Motor Coltrol pins
  ledcAttachPin(FINGER_THUMB_GPIO, FINGER_THUMB_CHANNEL);
  ledcAttachPin(FINGER_INDEX_GPIO, FINGER_INDEX_CHANNEL);
  ledcAttachPin(FINGER_MIDDLE_GPIO, FINGER_MIDDLE_CHANNEL);
  ledcAttachPin(FINGER_RING_GPIO, FINGER_RING_CHANNEL);
  ledcAttachPin(FINGER_PINKY_GPIO, FINGER_PINKY_CHANNEL);

  //Initializes Motor Control to home position: 30%
  for (int channel = 0; channel < 5; channel++){
    ledcWrite(channel, MOTOR_HOME);
  }
  Serial.println("finished setup");
}

void loop() {

  if (timer0){
    Serial.println("timer flag");
    //reset timer flag
    timer0 = false;

    //adds latest reading to the front of the vector
    myo1Deque.push_front(double(myo1Volts));
    myo2Deque.push_front(double(myo2Volts));

    //matains Deque size of 500 elements,
    if ((myo1DequeSize = myo1Deque.size()) >= MAX_DEQUE_SIZE) 
      myo1Deque.pop_back();
    else if ((myo2DequeSize = myo1Deque.size()) >= MAX_DEQUE_SIZE)
      myo1Deque.pop_back();

    Serial.print(double(thumbCurrent));Serial.print(", ");
    Serial.print(double(indexCurrent));Serial.print(", ");
    Serial.print(double(middleCurrent));Serial.print(", ");
    Serial.print(double(ringCurrent));Serial.print(", ");
    Serial.println(double(pinkyCurrent));
  }
  
  //Tempoary drive for motors
  //if either myoware sensor reads above 1V then all motors are driven to max position
  if (maxedCurrent()){
    //drives motors to halt motion if current threshold is reached
    for (int i = 0; i < 5; i++){
      ledcWrite(i, 0);
    }
  }else if (double(myo1Volts) > 1.0){
    while(duty > MOTOR_HOME){ //205 is 80% duty cycles @8 bit resolution
      //Serial.println(duty);
      for (int i = 0; i < 5; i++){
        //if (motorDriveCheck(i, duty)){
          ledcWrite(i, duty);
       /// }
      }
      duty -= 5; // current iterating in steps of 5 
    }
  }else if (double(myo2Volts) > 1.0){
    while(duty > MOTOR_HOME){ //205 is 80% duty cycles @8 bit resolution
      //Serial.println(duty);
      for (int i = 0; i < 5; i++){
        if (i != 2){
         // if (motorDriveCheck(i, duty)){
           ledcWrite(i, duty);
          //}
        }
      }
      duty -= 5; // current iterating in steps of 5 
    }
  }else{
    //Motor Control to home position: 30%
    //Serial.println("motor home");
    for (int channel = 0; channel < 5; channel++){
      duty = 205;
      ledcWrite(channel, 205);
    }

  }
}
