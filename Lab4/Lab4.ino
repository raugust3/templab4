// 
//  MSE 2202 MSEBot base code for Lab 4
//
//  Language: Arduino (C++)
//  Target:   ESP32
//  Author:   Michael Naish, Richard Augustine, Matthew Linders
//  Date:     2025 01 26 
//
//  Run mode 0: Robot stopped
//  Run mode 1: Exercise
//  Run mode 2: Test ultrasonic sensor
//  Run mode 3: Test IR receiver
//  Run mode 4: Test claw servo
//  Run mode 5: Test arm servo
//

// Uncomment keywords to enable debugging output
// #define DEBUG_DRIVE_SPEED    1
// #define DEBUG_ENCODER_COUNTS  1

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// Function declarations
void doHeartbeat();
void setMotor(int dir, int pwm, int in1, int in2);
void ARDUINO_ISR_ATTR buttonISR(void* arg);
void ARDUINO_ISR_ATTR encoderISR(void* arg);

// Button structure
struct Button {
  const int pin;                                                       // GPIO pin for button
  volatile uint32_t numberPresses;                                     // counter for number of button presses
  uint32_t nextPressTime;                                              // time of next allowable press in milliseconds
  volatile bool pressed;                                               // flag for button press event
};

// Encoder structure
struct Encoder {
  const int PinA;                                                      // GPIO pin for encoder Channel A
  const int PinB;                                                      // GPIO pin for encoder Channel B
  volatile long pos;                                                   // current encoder position
};

// Ultrasonic sensor structure
struct Ultrasonic {
  const int triggerPin;                                                // GPIO pin for trigger
  const int echoPin;                                                   // GPIO pin for echo
  volatile uint32_t pulseBegin = 0;                                    // time of echo pulse start in microseconds
  volatile uint32_t pulseEnd = 0;                                      // time of echo pulse end in microseconds
  volatile bool newEcho = false;                                       // new (valid) echo flag
  volatile bool timeout = false;                                       // echo timeout flag
  hw_timer_t* pTimer = NULL;                                           // pointer to timer used by trigger interrupt
};

// Function declarations
void doHeartbeat();
void setMotor(int dir, int pwm, int in1, int in2);
void initUltrasonic(Ultrasonic* us);
void ping(Ultrasonic* us);
uint32_t getEchoTime(Ultrasonic* us);
uint32_t usToCm(uint32_t us);
uint32_t usToIn(uint32_t us);
void ARDUINO_ISR_ATTR buttonISR(void* arg);
void ARDUINO_ISR_ATTR encoderISR(void* arg);
void ARDUINO_ISR_ATTR usTimerISR(void* arg);
void ARDUINO_ISR_ATTR echoISR(void* arg);

// Constants
const int cHeartbeatInterval = 75;                                     // heartbeat update interval, in milliseconds
const int cSmartLED          = 23;                                     // when DIP switch S1-4 is on, SMART LED is connected to GPIO23
const int cSmartLEDCount     = 1;                                      // number of Smart LEDs in use
const long cDebounceDelay    = 170;                                    // switch debounce delay in milliseconds
const int cNumMotors         = 2;                                      // Number of DC motors
const int cIN1Pin[]          = {26, 16};                               // GPIO pin(s) for IN1 for left and right motors (A, B)
const int cIN2Pin[]          = {27, 17};                               // GPIO pin(s) for IN2 for left and right motors (A, B)
const int cPWMRes            = 8;                                      // bit resolution for PWM
const int cMinPWM            = 150;                                    // PWM value for minimum speed that turns motor
const int cMaxPWM            = pow(2, cPWMRes) - 1;                    // PWM value for maximum speed
const int cPWMFreq           = 20000;                                  // frequency of PWM signal
const int cCountsRev         = 1096;                                   // encoder pulses per motor revolution
const int cPotPin            = 36;                                     // GPIO pin for drive speed potentiometer (A0)
const int cMotorEnablePin    = 39;                                     // GPIO pin for motor enable switch (DIP S1-1)
const int cTrigger           = 11;                                     // trigger duration in microseconds
const int cMaxEcho           = 15000;                                  // allowable time for valid echo in microseconds
const int cArmServo          = 4;                                      // GPIO pin for arm servo
const int cClawServo         = 13;                                     // GPIO pin for claw servo
const int cIRReceiver        = 5;                                      // GPIO pin for signal from IR receiver
const int encoderCm          = 80;                                     // encoder value of one cm of wheel movement

//=====================================================================================================================
//
// IMPORTANT: The constants in this section need to be set to appropriate values for your robot. 
//            You will have to experiment to determine appropriate values.

const int cClawServoOpen     = 1000;                                   // Value for open position of claw
const int cClawServoClosed   = 2130;                                   // Value for closed position of claw
const int cArmServoUp        = 2200;                                   // Value for shoulder of arm fully up
const int cArmServoDown      = 1000;                                   // Value for shoulder of arm fully down
const int cMotorAdjustment[] = {0, 0};                                 // PWM adjustment for motors to run closer to the same speed

// Variables
boolean motorsEnabled        = true;                                   // motors enabled flag
boolean timeUp4sec           = false;                                  // 3 second timer elapsed flag
boolean timeUp2sec           = false;                                  // 2 second timer elapsed flag
uint32_t lastHeartbeat       = 0;                                      // time of last heartbeat state change
uint32_t curMillis           = 0;                                      // current time, in milliseconds
uint32_t timerCount4sec      = 0;                                      // 3 second timer count in milliseconds
uint32_t timerCount2sec      = 0;                                      // 2 second timer count in milliseconds
uint32_t robotModeIndex      = 0;                                      // robot operational state                              
uint32_t lastTime            = 0;                                      // last time of motor control was updated
Button modeButton            = {0, 0, 0, false};                       // NO pushbutton PB1 on GPIO 0, low state when pressed
Encoder encoder[]            = {{35, 32, 0},                           // left encoder (A) on GPIO 35 and 32, 0 position 
                                {33, 25, 0}};                          // right encoder (B) on GPIO 33 and 25, 0 position
uint8_t driveSpeed           = 0;                                      // motor drive speed (0-255)
uint8_t driveIndex           = 0;                                      // state index for run mode
Ultrasonic ultrasonic        = {21, 22};                               // trigger on GPIO21 and echo on GPIO22
int encoder1;
int encoder2;
int encoder3;
int encoder4;

// Declare SK6812 SMART LED object
//   Argument 1 = Number of LEDs (pixels) in use
//   Argument 2 = ESP32 pin number 
//   Argument 3 = Pixel type flags, add together as needed:
//     NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//     NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//     NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//     NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//     NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel SmartLEDs(cSmartLEDCount, cSmartLED, NEO_RGB + NEO_KHZ800);

// Smart LED brightness for heartbeat
unsigned char LEDBrightnessIndex = 0; 
unsigned char LEDBrightnessLevels[] = {0, 0, 0, 5, 15, 30, 45, 60, 75, 90, 105, 120, 135, 
                                       150, 135, 120, 105, 90, 75, 60, 45, 30, 15, 5, 0};

uint32_t modeIndicator[6]    = {                                       // colours for different modes
  SmartLEDs.Color(255, 0, 0),                                          //   red - stop
  SmartLEDs.Color(0, 255, 0),                                          //   green - run
  SmartLEDs.Color(0, 0, 255),                                          //   blue - ultrasonic
  SmartLEDs.Color(255, 255, 0),                                        //   yellow - IR detector
  SmartLEDs.Color(0, 255, 255),                                        //   cyan - claw servo
  SmartLEDs.Color(255, 0, 255)                                         //   magenta - arm servo
};                                                                            

void setup() {
  Serial.begin(115200);                                                // Standard baud rate for ESP32 serial monitor

  // Set up SmartLED
  SmartLEDs.begin();                                                   // initialize smart LEDs object
  SmartLEDs.clear();                                                   // clear pixel
  SmartLEDs.setPixelColor(0, SmartLEDs.Color(0,0,0));                  // set pixel colours to black (off)
  SmartLEDs.setBrightness(0);                                          // set brightness [0-255]
  SmartLEDs.show();                                                    // update LED

  // setup motors with encoders
  for (int k = 0; k < cNumMotors; k++) {
    ledcAttach(cIN1Pin[k], cPWMFreq, cPWMRes);                         // setup INT1 GPIO PWM Channel
    ledcAttach(cIN2Pin[k], cPWMFreq, cPWMRes);                         // setup INT2 GPIO PWM Channel
    pinMode(encoder[k].PinA, INPUT);                                   // configure GPIO for encoder Channel A input
    pinMode(encoder[k].PinB, INPUT);                                   // configure GPIO for encoder Channel B input
    // configure encoder to trigger interrupt with each rising edge on Channel A
    attachInterruptArg(encoder[k].PinA, encoderISR, &encoder[k], RISING);
  }

  // Set up servos
  pinMode(cArmServo, OUTPUT);                                          // configure arm servo GPIO for output
  ledcAttach(cArmServo, 50, 14);                                       // setup arm servo pin for 50 Hz, 14-bit resolution
  pinMode(cClawServo, OUTPUT);                                         // configure claw servo GPIO for output
  ledcAttach(cClawServo, 50, 14);                                      // setup claw servo pin for 50 Hz, 14-bit resolution

  // Set up push button
  pinMode(modeButton.pin, INPUT_PULLUP);                               // configure GPIO for mode button pin with internal pullup resistor
  attachInterruptArg(modeButton.pin, buttonISR, &modeButton, FALLING); // configure ISR to trigger on low signal on pin
  
  // Setup IR receiver using UART2
  Serial2.begin(2400, SERIAL_8N1, cIRReceiver);                        // 2400 baud rate, 8 data bits, no parity, 1 stop bit
  
  pinMode(cPotPin, INPUT);                                             // set up drive speed potentiometer
  pinMode(cMotorEnablePin, INPUT);                                     // set up motor enable switch (uses external pullup)
  initUltrasonic(&ultrasonic);                                         // initialize ultrasonic sensor
}

void loop() {
  long pos[] = {0, 0};                                                 // current motor positions
  int pot = 0;                                                         // raw ADC value from pot
  uint32_t clawServoSetpoint;                                          // desired position of claw servo
  uint32_t armServoSetpoint;                                           // desired position of arm servo
  uint32_t pulseDuration;                                              // duration of pulse read from HC-SR04
  uint32_t lastPulse;                                                  // last valid pulse duration from HC-SR04
  uint32_t cycles  = 0;                                                // motor cycle count, used to start US measurements
  uint8_t receivedData;                                                // data received from IR receiver (byte)


  // store encoder position to avoid conflicts with ISR updates
  noInterrupts();                                                      // disable interrupts temporarily while reading
  for (int k = 0; k < cNumMotors; k++) {
      pos[k] = encoder[k].pos;                                         // read and store current motor position
  }
  interrupts();                                                        // turn interrupts back on
 
  uint32_t curTime = micros();                                         // capture current time in microseconds
  if (curTime - lastTime > 1000) {                                     // wait 1 ms
    lastTime = curTime;                                                // update start time for next control cycle

    // 4 second timer, counts 4000 milliseconds
    timerCount4sec += 1;                                               // increment 4 second timer count
    if (timerCount4sec > 4000) {                                       // if 4 seconds have elapsed
      timerCount4sec = 0;                                              // reset 4 second timer count
      timeUp4sec = true;                                               // indicate that 4 seconds have elapsed
    }  
   
    // 2 second timer, counts 2000 milliseconds
    timerCount2sec += 1;                                               // increment 2 second timer count
    if (timerCount2sec > 2000) {                                       // if 2 seconds have elapsed
      timerCount2sec = 0;                                              // reset 2 second timer count
      timeUp2sec = true;                                               // indicate that 2 seconds have elapsed
    }

    if (modeButton.pressed) {                                          // Change mode on button press
      robotModeIndex++;                                                // switch to next mode
      robotModeIndex = robotModeIndex & 7;                             // keep mode index between 0 and 7
      timerCount4sec = 0;                                              // reset 3 second timer count
      timeUp4sec = false;                                              // reset 3 second timer
      modeButton.pressed = false;                                      // reset flag
    }

    // check if drive motors should be powered
    motorsEnabled = !digitalRead(cMotorEnablePin);                     // if SW1-1 is on (low signal), then motors are enabled

    // modes 
    // 0 = Default after power up/reset.           Robot is stopped
    // 1 = Press mode button once to enter.        Run exercise
    // 2 = Press mode button twice to enter.       Test ultrasonic sensor 
    // 3 = Press mode button three times to enter. Test IR receiver
    // 4 = Press mode button four times to enter.  Test claw servo 
    // 5 = Press mode button five times to enter.  Test arm servo 
    // 6 = Press mode button six times to enter.   Add your code to do something 
    switch (robotModeIndex) {
      case 0: // Robot stopped
        setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);                        // stop left motor
        setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);                        // stop right motor
        ledcWrite(cArmServo, 1450);
        ledcWrite(cClawServo, 1000);
        encoder[0].pos = 0;                                            // clear left encoder
        encoder[1].pos = 0;                                            // clear right encoder
        driveIndex = 0;                                                // reset drive index
        timerCount2sec = 0;                                            // reset 2 second timer count
        timeUp2sec = false;                                            // reset 2 second timer
        break;

      case 1: // run exercise
        // two seconds before initialization
        switch(driveIndex) {
          case 0: // waiting 2 seconds before executing
            if (timeUp2sec && motorsEnabled) {

              // Read pot to update drive motor speed 
              pot = analogRead(cPotPin);
              driveSpeed = map(pot, 0, 4095, cMinPWM, cMaxPWM);
              
              // increment step
              driveIndex++;
            }
            break;

          case 1: // driving forward 45 cm (the length of 1.5 pieces of paper)
            if (pos[0] < encoderCm * 45) {
              setMotor(1, driveSpeed, cIN1Pin[0], cIN2Pin[0]);                  // drive left motor forward at speed determined by pot
              setMotor(-1, driveSpeed - 8, cIN1Pin[1], cIN2Pin[1]);            // drive right motor forward at speed determined by pot, rich: pwm adjust 8
              // Serial.printf("pos: %lu\n", pos[0]);
            } else {
              setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);                           // stop left motor
              setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);                           // stop right motor

              encoder1 = pos[0]; // capture encoder position

              // increment step
              driveIndex++;

              // reset timer for next step 2 sec wait
              timerCount2sec = 0;
              timeUp2sec = false;
            }
            break;

          case 2: // waiting 2 seconds
            if (timeUp2sec) {

              //ledcWrite(cClawServo, 1800);

              encoder[0].pos = 0;                                            // clear left encoder
              encoder[1].pos = 0;                                            // clear right encoder
              // increment step
              driveIndex++;
            }
            break;

          case 3: // turn left (counterclockwise) - motors spin in same direction
            if (abs(pos[0]) < encoderCm * 9) {
              setMotor(-1, driveSpeed, cIN1Pin[0], cIN2Pin[0]);                  // drive left motor backward at speed determined by pot
              setMotor(-1, driveSpeed, cIN1Pin[1], cIN2Pin[1]);                 // drive right motor forward at speed determined by pot
              // Serial.printf("pos: %lu\n", abs(pos[0]));
            } else {
              setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);                           // stop left motor
              setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);                           // stop right motor

              encoder2 = abs(pos[0]);

              // increment step
              driveIndex++;

              // reset timer for next step 2 sec wait
              timerCount2sec = 0;
              timeUp2sec = false;
            }
            break;
          
          case 4: // waiting 2 seconds
            if (timeUp2sec) {
              encoder[0].pos = 0;                                            // clear left encoder
              encoder[1].pos = 0;                                            // clear right encoder
              // increment step
              driveIndex++;
            }
            break;
          
          case 5: // drive forward 38.74 cm (the length of 1.5 pieces of paper)
            if (pos[0] < encoderCm * 38.74) {
              setMotor(1, driveSpeed, cIN1Pin[0], cIN2Pin[0]);                  // drive left motor forward at speed determined by pot
              setMotor(-1, driveSpeed - 8, cIN1Pin[1], cIN2Pin[1]);            // drive right motor forward at speed determined by pot, rich: pwm adjust 8
              // Serial.printf("pos: %lu\n", pos[0]);
            } else {
              setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);                           // stop left motor
              setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);                           // stop right motor

              encoder3 = pos[0];

              // increment step
              driveIndex++;

              // reset timer for next step 2 sec wait
              timerCount2sec = 0;
              timeUp2sec = false;
            }
            break;
          
          case 6: // waiting 2 seconds
            if (timeUp2sec) {
              encoder[0].pos = 0;                                            // clear left encoder
              encoder[1].pos = 0;                                            // clear right encoder
              // increment step
              driveIndex++;
              //driveIndex = 11;
            }
            break;

          case 7:
            ledcWrite(cClawServo, cClawServoClosed);                   
            // increment step
            driveIndex++;

            // reset timer for next step 4 sec wait
            timerCount4sec = 0;
            timeUp4sec = false;
            break;

          case 8: // waiting 4 seconds
            if (timeUp4sec) {
              // increment step
              driveIndex++;
            }
            break;
          
          case 9:
            ledcWrite(cArmServo, 1600);                   
            // increment step
            driveIndex++;

            // reset timer for next step 2 sec wait
            timerCount2sec = 0;
            timeUp2sec = false;
            break;

          case 10: // waiting 2 seconds
            if (timeUp2sec) {
              // increment step
              driveIndex++;
            }
            break;
          
          case 11:
            if (pos[0] + encoder1 + encoder2 + encoder3 > encoder1 + encoder2) {
              setMotor(-1, driveSpeed, cIN1Pin[0], cIN2Pin[0]);                  // drive left motor backward at speed determined by pot
              setMotor(1, driveSpeed, cIN1Pin[1], cIN2Pin[1]);                 // drive right motor backward at speed determined by pot
              // Serial.printf("pos: %lu\n", pos[0] + encoder1 + encoder2 + encoder3);
            } else {
              setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);                           // stop left motor
              setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);                           // stop right motor             

              // increment step
              driveIndex++;

              // reset timer for next step 2 sec wait
              timerCount2sec = 0;
              timeUp2sec = false;
            }
            break;
          
          case 12:
            if (timeUp2sec) {
              encoder[0].pos = 0;                                            // clear left encoder
              encoder[1].pos = 0;                                            // clear right encoder
              // increment step
              driveIndex++;
            }
            break;
          
          case 13:
            if (encoder1 + encoder2 + encoderCm - pos[0] > encoder1) {
              setMotor(1, driveSpeed, cIN1Pin[0], cIN2Pin[0]);                  // drive left motor forward at speed determined by pot
              setMotor(1, driveSpeed, cIN1Pin[1], cIN2Pin[1]);                 // drive right motor forward at speed determined by pot
              // Serial.printf("pos: %lu\n", encoder1 + encoder2 - pos[0]);
            } else {
              setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);                           // stop left motor
              setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);                           // stop right motor

              encoder2 = abs(pos[0]);

              // increment step
              driveIndex++;

              // reset timer for next step 2 sec wait
              timerCount2sec = 0;
              timeUp2sec = false;
            }
            break;
          
          case 14:
            if (timeUp2sec) {
              encoder[0].pos = 0;                                            // clear left encoder
              encoder[1].pos = 0;                                            // clear right encoder
              // increment step
              driveIndex++;
            }
            break;
          
          case 15:
            if (encoder1 + pos[0] > 0) {
              setMotor(-1, driveSpeed, cIN1Pin[0], cIN2Pin[0]);                  // drive left motor forward at speed determined by pot
              setMotor(1, driveSpeed, cIN1Pin[1], cIN2Pin[1]);                 // drive right motor forward at speed determined by pot
              // Serial.printf("pos: %lu\n", encoder1 + pos[0]);
            } else {
              setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);                           // stop left motor
              setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);                           // stop right motor

              // increment step
              driveIndex++;
              // robotModeIndex = 0;

              // reset timer for next step 2 sec wait
              timerCount2sec = 0;
              timeUp2sec = false;
            }
            break;

          case 16:
            if (timeUp2sec) {
              //ledcWrite(cArmServo, 1450); 
              // increment step
              driveIndex++;
            }
            break;

          case 17:
            //ledcWrite(cClawServo, 1500);
            // increment step
            robotModeIndex = 0;
            break;
        }

      case 2: // Test ultrasound
        setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);                        // stop left motor
        setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);                        // stop right motor
       
        // start US measurement once every 100 cycles (~100 ms)
        if (cycles == 0) {
          ping(&ultrasonic);                                           // start US measurement
        }
        cycles++;
        if (cycles >= 100) {                                           // 100 cycles = ~100 ms
          cycles = 0;
        }
        pulseDuration = getEchoTime(&ultrasonic);                      // check if new measurement is available
        // if valid measurement, update variable and output to serial
        if (pulseDuration > 0) {
          lastPulse = pulseDuration;
          // Serial.printf("Ultrasound: %ld ms = %ld cm = %ld in\n", lastPulse, usToCm(lastPulse), usToIn(lastPulse));
        }
        break;

      case 3: // Test IR receiver
        setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);                        // stop left motor
        setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);                        // stop right motor

        if (Serial2.available() > 0) {                                 // if data available
          receivedData = Serial2.read();                               // read incoming byte
          Serial.printf("Received: %c\n", receivedData);               // output received byte
        }
        break;

      case 4: // Test claw servo with pot
        setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);                        // stop left motor
        setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);                        // stop right motor

        // read pot and map to allowable servo range for claw
        pot = analogRead(cPotPin);
        clawServoSetpoint = map(pot, 0, 4095, cClawServoOpen, cClawServoClosed);
        ledcWrite(cClawServo, clawServoSetpoint);                      // set the desired servo position
        // limit output rate to serial monitor
        if (cycles >= 200) {                                           // 200 cycles = ~200 ms
          cycles = 0;
          Serial.printf("Claw: Pot R1 = %d, mapped = %d\n", pot, clawServoSetpoint);
        }
        break;

      case 5: // Test arm servo with pot
        setMotor(0, 0, cIN1Pin[0], cIN2Pin[0]);                        // stop left motor
        setMotor(0, 0, cIN1Pin[1], cIN2Pin[1]);                        // stop right motor

        // read pot and map to allowable servo range for claw
        pot = analogRead(cPotPin);
        armServoSetpoint = map(pot, 0, 4095, cArmServoDown, cArmServoUp);
        ledcWrite(cArmServo, armServoSetpoint);                        // set the desired servo position
        // limit output rate to serial monitor
        if (cycles >= 200) {                                           // 200 cycles = ~200 ms
          cycles = 0;
          Serial.printf("Arm: Pot R1 = %d, mapped = %d\n", pot, armServoSetpoint);
        }
        break;

      case 6: //add your code to do something 
        robotModeIndex = 0;                                            //  !!!!!!!  remove if using the case
        break;
    }
  }

  doHeartbeat();                                                       // update heartbeat LED
}

// update heartbeat LED
void doHeartbeat() {
  curMillis = millis();                                                // get the current time in milliseconds
  // check to see if elapsed time matches the heartbeat interval
  if ((curMillis - lastHeartbeat) > cHeartbeatInterval) {
    lastHeartbeat = curMillis;                                         // update the heartbeat time for the next update
    LEDBrightnessIndex++;                                              // shift to the next brightness level
    if (LEDBrightnessIndex >= sizeof(LEDBrightnessLevels)) {           // if all defined levels have been used
      LEDBrightnessIndex = 0;                                          // reset to starting brightness
    }
    SmartLEDs.setBrightness(LEDBrightnessLevels[LEDBrightnessIndex]);  // set brightness of heartbeat LED
    SmartLEDs.setPixelColor(0, modeIndicator[robotModeIndex]);         // set pixel colors to = mode 
    SmartLEDs.show();                                                  // update LED
  }
}

// Function to move the claw servo smoothly
void moveClawSmoothly(uint32_t startPos, uint32_t endPos, int speed) {
  if (startPos < endPos) {
    for (uint32_t pos = startPos; pos <= endPos; pos += 10) { // Increment in small steps
      ledcWrite(cClawServo, pos);
      delay(speed); // Adjust speed with delay
    }
  } else {
    for (uint32_t pos = startPos; pos >= endPos; pos -= 10) { // Decrement in small steps
      ledcWrite(cClawServo, pos);
      delay(speed);
    }
  }
}

// send motor control signals, based on direction and pwm (speed)
void setMotor(int dir, int pwm, int in1, int in2) {
  if (dir == 1) {                                                      // forward
    ledcWrite(in1, pwm);
    ledcWrite(in2, 0);
  }
  else if (dir == -1) {                                                // reverse
    ledcWrite(in1, 0);
    ledcWrite(in2, pwm);
  }
  else {                                                               // stop
    ledcWrite(in1, 0);
    ledcWrite(in2, 0);
  }
}

// button interrupt service routine
// argument is pointer to button structure, which is statically cast to a Button structure, 
// allowing multiple instances of the buttonISR to be created (1 per button)
void ARDUINO_ISR_ATTR buttonISR(void* arg) {
  Button* s = static_cast<Button*>(arg);                               // cast pointer to static structure

  uint32_t pressTime = millis();                                       // capture current time
  if (pressTime > s->nextPressTime) {                                  // if enough time has passed to consider a valid press
    s->numberPresses += 1;                                             // increment button press counter
    s->pressed = true;                                                 // indicate valid button press state
    s->nextPressTime = pressTime + cDebounceDelay;                     // update time for next valid press
  }  
}

// encoder interrupt service routine
// argument is pointer to an encoder structure, which is statically cast to a Encoder structure, allowing multiple
// instances of the encoderISR to be created (1 per encoder)
void ARDUINO_ISR_ATTR encoderISR(void* arg) {
  Encoder* s = static_cast<Encoder*>(arg);                             // cast pointer to static structure
  
  int b = digitalRead(s->PinB);                                        // read state of Channel B
  if (b > 0) {                                                         // high, leading Channel A
    s->pos++;                                                          // increase position
  }
  else {                                                               // low, lagging Channel A
    s->pos--;                                                          // decrease position
  }
}

// initialize ultrasonic sensor I/O and associated interrupts
void initUltrasonic(Ultrasonic* us) {
  pinMode(us->triggerPin, OUTPUT);                                     // configure trigger pin for output
  pinMode(us->echoPin, INPUT);                                         // configure echo pin for input
  attachInterruptArg(us->echoPin, echoISR, us, CHANGE);                // attach interrupt to state change on echo pin
  us->pTimer = timerBegin(1000000);                                    // start timer with 1 MHz frequency
  timerAttachInterruptArg(us->pTimer, usTimerISR, us);                 // configure timer ISR
}

// start ultrasonic echo measurement by sending pulse to trigger pin
void ping(Ultrasonic* us) {
  timerRestart(us->pTimer);                                            // start timer at 0
  timerAlarm(us->pTimer, cTrigger, false, 0);                          // one-shot interrupt
  digitalWrite(us->triggerPin, HIGH);                                  // start pulse on trigger pin
  us->timeout = false;                                                 // clear echo timeout flag
}

// if new pulse received on echo pin calculate echo duration in microseconds
uint32_t getEchoTime(Ultrasonic *us) {
  if (ultrasonic.newEcho && !ultrasonic.timeout) {
    ultrasonic.newEcho = false;                                        // clear new echo flag
    // Serial.printf("begin: %ld, end: %ld\n", ultrasonic.pulseBegin, ultrasonic.pulseEnd);
    return ultrasonic.pulseEnd - ultrasonic.pulseBegin;
  }
  else {                                                               // no new echo or echo timed out
    return 0;                                    
  }
}

// convert echo time in microseconds to centimetres
uint32_t usToCm(uint32_t us) {
   return (uint32_t) (double) us * 0.01724;                            // echo time in microseconds / 29 / 2 (29 us per cm)
}

// convert echo time in microseconds to inches
uint32_t usToIn(uint32_t us) {
   return (uint32_t) (double) us * 0.006757;                           // echo time in microseconds / 74 / 2 (73.746 us per inch)
}

// timer interrupt service routine
// single timer (and ISR) is used to end trigger pulse (short interval), then determine echo timeout
void ARDUINO_ISR_ATTR usTimerISR(void* arg) {
  Ultrasonic* us = static_cast<Ultrasonic*>(arg);                      // cast pointer to static structure

  // very short duration before interrupt (trigger length + timer overhead)
  // end pulse on trigger pin and set new alarm for echo timeout
  if (timerRead(us->pTimer) < cTrigger + 10) {                         // timer count < trigger duration + timer overhead
    digitalWrite(us->triggerPin, LOW);                                 // end pulse on trigger pin
    timerRestart(us->pTimer);                                          // restart for timer for echo timeout timing
    timerAlarm(us->pTimer, cMaxEcho, false, 0);                        // set duration of echo timeout
  }
  // longer duration before interrupt indicates that echo has timed out
  else {                                        
    us->timeout = true;                                                // set echo timeout flag
  }
}

// echo interrupt service routine
void ARDUINO_ISR_ATTR echoISR(void* arg) {
  Ultrasonic* us = static_cast<Ultrasonic*>(arg);                      // cast pointer to static structure

  // capture time when echo pin state changes from LOW to HIGH
  if (digitalRead(us->echoPin)) {
    us->pulseBegin = micros();
  } 
  // capture time when echo pin state changes from HIGH to LOW
  // only if transistion happens before echo timeout
  else if (!us->timeout) {
    us->pulseEnd = micros();
    us->newEcho = true;                                                // set new echo flag
  }
}