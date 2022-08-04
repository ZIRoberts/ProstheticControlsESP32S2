// These define's must be placed at the beginning before #include "TimerInterrupt_Generic.h"
// _TIMERINTERRUPT_LOGLEVEL_ from 0 to 4
// Don't define _TIMERINTERRUPT_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system.
#define TIMER_INTERRUPT_DEBUG       0
#define _TIMERINTERRUPT_LOGLEVEL_   4

#include "ESP32_S2_TimerInterrupt.h"
#include <ESP32AnalogRead.h>
//C++ libraries are not native to arduino, included within ESPIDF
#include <deque> 

//Hardware Timer Defintions
#define TIMER0_INTERVAL_MS  1000
#define TIMER0_PERIOD_MS    10

//General PWM Configuration
#define PWM_FREQ 333 //Sets 333 Hz
#define PWM_RESOLUTION 8 //Sets 8 bit resolution
#define MOTOR_HOME 76 //30% duty cycle @8 bit resolution

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

//sets max deque size to 5 seconds, 500 samples [0,499]
#define MAX_DEQUE_SIZE 499

//Creates ADC objects
ESP32AnalogRead myoware1; //Myoware Sensor Inside Fore arm
ESP32AnalogRead myoware2; // Myoware Sensor Outside Fore arm
ESP32AnalogRead fsrIndex; // FSR Sensor for Index Finger
ESP32AnalogRead fsrMiddle; // FSR Sensor for Middle Finger
ESP32AnalogRead fsrRing; // FSR Sensor for Ring Finger
ESP32AnalogRead fsrPinky; // FSR Sensor for Pinky Finger
ESP32AnalogRead fsrThumb; // FSR Sensor for Thumb
ESP32AnalogRead fsrPalm; // FSR Sensor for Palm of Hand

//Deque definitions
std::deque<double> myo1Deque;
std::deque<double> myo2Deque;

//used to to ensure vector size does not exceed 500 samples
int myo1DequeSize;
int myo2DequeSize;

//stores ADC reading 
volatile double myo1Volts;
volatile double myo2Volts;
volatile double fsrI_Volts;
volatile double fsrM_Volts;
volatile double fsrR_Volts;
volatile double fsrPi_Volts;
volatile double fsrT_Volts;
volatile double fsrPa_Volts;

//Timer Flags
volatile bool timer0 = false;

// Init ESP32 timer 0 and 1
ESP32Timer ITimer0(0);

//debugging for interrupt timer
volatile int timerDebug = 0;
bool IRAM_ATTR TimerHandler0(void * timerNo) { 
  timer0 = true;
  timerDebug = millis();
  myo1Volts = myoware1.readVoltage();
  myo2Volts = myoware2.readVoltage();
  fsrI_Volts = fsrIndex.readVoltage();
  fsrM_Volts = fsrMiddle.readVoltage();
  fsrR_Volts = fsrRing.readVoltage();
  fsrPi_Volts = fsrPinky.readVoltage();
  fsrT_Volts = fsrThumb.readVoltage();
  fsrPa_Volts = fsrPalm.readVoltage();

  return true;
}

double forceValue(double voltage){
  double forceConversion;
  forceConversion = pow((271/(47000*((3.3/voltage)-1))),(1/0.69));
  
  return forceConversion;
}

void setup() {
  Serial.begin(115200);

  //Hardware Timer Configuration

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
  fsrIndex.attach(8); //FSR on Index finger is attached to GPIO 8
  fsrMiddle.attach(9); //FSR on Middle finger is attached to GPIO 9
  fsrRing.attach(10); //FSR on Ring finger is attached to GPIO 10
  fsrPinky.attach(11); //FSR on Pinky is attached to GPIO 11
  fsrThumb.attach(12); //FSR on Thumb is attached to GPIO 12
  fsrPalm.attach(13); //FSR on the Palm of the hand is attached to GPIO 13

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
}

void loop() {

  if (timer0){
    //adds latest reading to the front of the vector
    myo1Deque.push_front(double(myo1Volts));
    myo2Deque.push_front(double(myo2Volts));

    //maintains Deque size of 500 elements,
    if ((myo1DequeSize = myo1Deque.size()) >= MAX_DEQUE_SIZE) 
      //myo1Vector.remove(myo1VectorSize - 1); //removes last element of myo1vector
      myo1Deque.pop_back();
    else if ((myo2DequeSize = myo1Deque.size()) >= MAX_DEQUE_SIZE)
      //myo2Vector.remove(myo2VectorSize - 1); //removes last element of myo2vector
     myo1Deque.pop_back();
    
    Serial.println(timerDebug);
    Serial.println(myo1Volts);
    Serial.println(myo2Volts);
    Serial.print("Voltage reading in V = ");
    Serial.println(fsrI_Volts);
    Serial.println(fsrM_Volts);
    Serial.println(fsrR_Volts);
    Serial.println(fsrPi_Volts);
    Serial.println(fsrT_Volts);
    Serial.println(fsrPa_Volts);
  }
  
  //Tempoary drive for motors
  //if either myoware sensor reads above 1V then all motors are driven to max position
  if (myo1Volts > 1.4 || myo2Volts > 1.4){
    hold = true;
    for (int channel = 0; channel < 5; channel++){
      ledcWrite(channel, 205); //205 is 80% duty cycles @8 bit resolution
    }
  } else {
     for (int channel = 0; channel < 5; channel++){
    ledcWrite(channel, MOTOR_HOME);
     }
    }

}
