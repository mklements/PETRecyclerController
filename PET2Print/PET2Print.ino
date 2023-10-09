#include <PID_v1.h>
#include <thermistor.h>                     //https://github.com/miguel5612/Arduino-ThermistorLibrary
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>

#define SCREEN_WIDTH 128                    //OLED display width, in pixels
#define SCREEN_HEIGHT 64                    //OLED display height, in pixels

#define OLED_RESET     -1                   //Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C                 //< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define initialTemp 220                     //Define parameters for each edjustable setting
#define minTemp 190                      
#define maxTemp 230
#define initialSpeed 22
#define minSpeed 1
#define maxSpeed 28
#define initialMot 1
#define minMot 0
#define maxMot 2

int encLowLim = minTemp;                    //Variables to store the encoder limits and increment
int encHighLim = maxTemp;
int encIncrement = 1;
int encCurrent = initialTemp;
int dataInputNo = 0;                        //Data input tracking, 0 - temp, 1 - speed, 2 - motor

static int pinA = 2;                        //Hardware interrupt digital pin 2
static int pinB = 3;                        //Hardware interrupt digital pin 3
volatile int encoderPos = initialTemp;      //Current value corresonding to the encoder position
volatile int prevEncoderPos = initialTemp;  //Tracker for the previous encoder position to check whether to update

byte oldButtonState = HIGH;                 //First button state is open because of pull-up resistor
const unsigned long debounceTime = 10;      //Debounce delay time
unsigned long buttonPressTime;              //Time button has been pressed for debounce
boolean pressed = false;

const int temperaturePin = A2;              //Define the remaining IO pins for motor, pushbutton & thermistor
const int pwmPin = 9;
const int enablePin = 5;
const int motDirPin = 6;
const int motStepPin = 7;
const int encButton = 4;

int loopTime = 500;                          //Define time for each loop cycle
unsigned long currentTime = 0;

double Kp = 80.0;                            //Define PID constants
double Ki = 35.0;
double Kd = 80.0;

thermistor therm1(temperaturePin,0);         //Connect thermistor on A2

double setpoint = initialTemp;               //Define PID variables & establish PID loop
double input, output;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

int motSpeed = initialSpeed;                 //Define motor parameters
int motDir = initialMot;
int motMaxDelay = 100;
int motDelayTime = 500;

void setup() 
{
  Serial.begin(9600);                                     //Initialize serial communication

  //SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  display.clearDisplay();                                 //Clear the buffer
  
  pinMode(pwmPin, OUTPUT);                                //Configure PWM pin
  
  pid.SetMode(AUTOMATIC);                                 //Set the PID parameters
  pid.SetOutputLimits(0, 255);

  input = therm1.analog2temp();                           //Read and set the initial input value
  
  pinMode(pinA, INPUT_PULLUP);                            //Set pinA as an input, pulled HIGH to the logic voltage
  pinMode(pinB, INPUT_PULLUP);                            //Set pinB as an input, pulled HIGH to the logic voltage
  attachInterrupt(digitalPinToInterrupt(pinA), rotaryInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinB), rotaryInterrupt, CHANGE);
  pinMode(encButton, INPUT_PULLUP);                       //Set the encoder button as an input, pulled HIGH to the logic voltage
  pinMode(enablePin, INPUT);                              //Open circuit enable pin, disables motor
  pinMode(motDirPin, OUTPUT);                             //Define the stepper motor pins
  pinMode(motStepPin, OUTPUT);
  digitalWrite(motDirPin, HIGH);                          //Set the initial direction of motion for motor

  Serial.println("Setup complete");                       //Write to serial monitor to indicate the setup function is complete
}

void loop() 
{
  //Record the start time for the loop
  currentTime = millis();

  //Read the temperature
  input = therm1.analog2temp(); // read temperature
  
  //Compute the PID output
  pid.Compute();
  
  //Update the PWM output
  analogWrite(pwmPin, output);
  int temp = input;
  
  //Print the temperature and PWM output
  Serial.print("Temperature: ");
  Serial.print(temp);
  Serial.print(" \u00B0C");
  Serial.print("\tPWM Output: ");
  Serial.print(output); 
  Serial.print("\tEncoder: ");
  Serial.println(encCurrent);

  //Update the OLED display
  updateDataDisplay ();

  //Check for input on the pushbutton
  while(millis() < currentTime + loopTime)
  {
    byte buttonState = digitalRead (encButton); 
    if (buttonState != oldButtonState)
    {
      Serial.println("Button change");
      if (millis () - buttonPressTime >= debounceTime)      //Debounce button
      {
        buttonPressTime = millis ();                        //Time when button was pushed
        oldButtonState =  buttonState;                      //Remember button state for next time
        if (buttonState == LOW)
        {
          pressed = true;
          Serial.println("Button Pressed");
        }
        else 
        {
          if (pressed == true)                              //Confirm the input once the button is released again
          {
            boolean pressed = false;
            Serial.println("Button Released");
            if (dataInputNo == 0)                           //Set which parameter is being edited and define limits
            {
              dataInputNo = 1;
              encCurrent = motSpeed;
              encLowLim = minSpeed;
              encHighLim = maxSpeed;
            }
            else if (dataInputNo == 1)
            {
              dataInputNo = 2;
              encCurrent = motDir;
              encLowLim = minMot;
              encHighLim = maxMot;
            }
            else
            {
              dataInputNo = 0;
              encCurrent = setpoint;
              encLowLim = minTemp;
              encHighLim = maxTemp;
            }
          }
        }  
      }
    }

    //Set the parameter being edited equal to the current encoder position
    if (dataInputNo == 0)                                    
    {
      setpoint = encCurrent;
    }
    else if (dataInputNo == 1)
    {
      motSpeed = encCurrent;
      motDelayTime = 100 * (1 + maxSpeed - motSpeed);
    }
    else
    {
      motDir = encCurrent;
    }

    //Set the motor direction
    if (motDir == 0)
    {
      pinMode(enablePin, OUTPUT);                   //Enable motor
      digitalWrite(motDirPin, LOW);                 //Reverse motor direction
      updateDataDisplay ();
    }
    else if (motDir == 2)
    { 
      pinMode(enablePin, OUTPUT);                   //Enable motor 
      digitalWrite(motDirPin, HIGH);                //Forward motor direction
      updateDataDisplay ();
    }
    else
    {
      pinMode(enablePin, INPUT);                    //Disable motor
    }
    
    //Pulse the stepper motor if forward or reverse is selected
    while (motDir != 1)
    {
      runMotor ();
      motDir = encCurrent;
    }
  }
}

//Increment the current setting if the rotary encoder is turned
void rotaryInterrupt ()
{
  encoderPos = digitalRead(pinA);
  if ((prevEncoderPos == 0) && (encoderPos == 1)) 
  {
      if (digitalRead(pinB) == 1) 
      {
        if (encCurrent < encHighLim)
          encCurrent=encCurrent+encIncrement;               //Increase field if turned clockwise
      }
      else 
      {
        if (encCurrent > encLowLim)
          encCurrent=encCurrent-encIncrement;               //Decrease field if turned anti-clockwise
      }
  }
  prevEncoderPos = encoderPos;
  delay(1);
}

//Update the OLED display contents
void updateDataDisplay ()
{
  display.clearDisplay();                                   //Clear display
  display.setTextSize(1);                                   //Set the text size
  display.setTextColor(SSD1306_WHITE);                      // Draw white text
  display.setCursor(2,10);                                  //Set the display cursor position
  display.print(F("Current Temp: "));                       //Set the display text
  display.setCursor(2,20);
  display.print(F("Set Temp: "));
  display.setCursor(2,30);
  display.print(F("Extrude Speed: "));
  display.setCursor(2,40);
  display.print(F("Extrude: "));
  int temp = input;
  int setPointInt = setpoint;
  int selected = 0;
  if (dataInputNo == 0)                                     //Set the cursor position
  {
    selected = 20;
  }
  else if (dataInputNo == 1)
  {
    selected = 30;
  }
  else
  {
    selected = 40;
  }
  display.setCursor(87,selected);                            //Set the display cursor position
  display.print(F(">"));
  display.setCursor(97,10);
  display.print(temp);
  display.print(F("C"));
  display.setCursor(97,20);
  display.print(setPointInt);
  display.print(F("C"));
  display.setCursor(97,30);
  display.print(motSpeed);
  display.setCursor(97,40);
  if (motDir == 0)
    display.print(F("Rev"));
  else if (motDir == 2)
    display.print(F("FWD"));
  else
    display.print(F("OFF"));
  display.display();                                          //Output the display text
}

//Turn the reel motor and maintain hot end temperature
void runMotor ()
{
  digitalWrite(motStepPin, HIGH);
  delayMicroseconds(motDelayTime);
  digitalWrite(motStepPin, LOW);
  if(millis() < currentTime + loopTime)
  {
    delayMicroseconds(motDelayTime);
  }
  else
  {
    //Record the start time for the loop
    currentTime = millis();

    //Read the temperature
    input = therm1.analog2temp(); // read temperature
  
    //Compute the PID output
    pid.Compute();
  
    //Update the PWM output
    analogWrite(pwmPin, output);
  }
}