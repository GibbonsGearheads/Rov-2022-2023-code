
// Arduino Serial communications
#include <SoftwareSerial.h>    // used for Sabretooth & BlueTooth

// Sabertooth device driver
#include <SabertoothSimplified.h>

// include the DISPLAY library
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

int SerialBaudRate = 9600;
bool armMode = false;
int doubleTap = 0;
float doubleTapTime = 0;
float doubleTapTimeMax = 10;

float refresh = 0;
float refreshMax = 16;
// Initialize the LCD screen
int LCD_I2C_Addr = 0x27;   //I2C address for the LCD Screen (Default=0x27)
LiquidCrystal_I2C lcd(LCD_I2C_Addr, 20, 4); // Set the LCD I2C address. Use 20 Space 4-line LCD.

// Initialize Arduino serial communications
SoftwareSerial SWSerial(NOT_A_PIN, 10); // RX on no pin (unused), TX on pin 10 (to S1).

SoftwareSerial BTSerial(2, 3);   // For communication to with the Bluetooth Device
int BT_Ena = 4;

// Initialize Sabertooth driver passing it the Arduino serial communications object
SabertoothSimplified ST(SWSerial);      // Use SWSerial as the serial port.
int ST1 = 8;                         // Arduino pin attached to Sabertooth controller
int ST2 = 9;                         // Arduino pin attached to Sabertooth controller


void setup()
{
  // Set Arduino pin for each sabertooth as OUTPUT
  pinMode(ST1, OUTPUT);            // Arduino pin control to sabertooth
  pinMode(ST2, OUTPUT);            // Arduino pin control to sabertooth
  pinMode(BT_Ena, OUTPUT);            // Bluetooth ena pin
  digitalWrite(BT_Ena, LOW);          // Set the Bluetooth ena low
  // Start Serial Communications
  SWSerial.begin(SerialBaudRate);    // Start the Sabretooth channel
  BTSerial.begin(SerialBaudRate);    // Start the Bluetooth channel

  // Set up LCD
  lcd.init();
  lcd.backlight();
  lcd.write(12);
  lcd.setCursor(0, 0);
  lcd.print("Connection at: ");
  lcd.print(SerialBaudRate);
  lcd.setCursor(0, 1);
  lcd.print("Test...");


  lcd.setCursor(8, 1);
  lcd.print("COMPLETE!");
  delay(1000);
  lcd.clear();
}

void loop()
{
  // Declare variables
  int Joy1_H, Joy1_V, Joy2_H, Joy2_V;
  int pwr1, pwr2, pwr3, pwr4;
  int MtrHR, MtrHL, MtrVR, MtrVL;

  // Read Joystick Output
  Joy1_V = analogRead(A0);       // get the left vertical (Y) joystick input
  Joy1_H = analogRead(A1);       // get the left horizontal (X) joystick input
  Joy2_V = analogRead(A2);       // get the right vertical (Y) joystick input
  Joy2_H = analogRead(A3);       // get the right horizontal (X) joystick input

  // Re-Map Joysticks to motor level
  pwr1 = joystickMapping(Joy1_V);   //scale the right inputs for direct use
  pwr2 = joystickMapping(Joy2_V);   //scale the right inputs for direct use
  pwr3 = joystickMapping(Joy1_H);   //scale the right inputs for direct use
  pwr4 = joystickMapping(Joy2_H);   //scale the right inputs for direct use


  // Set Motor variables to power variables
  //(intermediate use until accelerometer added)
  MtrVL = pwr1;
  MtrVR = pwr2;
  MtrHL = pwr3;
  MtrHR = pwr4;
  //most of the logic for switching modes, except the actual servo stuff.
  // by the way i dont know which joy stick corresponds to th right one so if im wrong just change it.
  if(pwr3 > 120){
    doubleTap ++;
  }
  DoubleTap();
  if(armMode == false){
    //dont want to mess with the code anymore but this is where the rotations would go
  }else{
    //dont have the servos yet  but this is where the arm rotations would go
  }
  // Set the power in each motor
  setMotor(ST2, 1, MtrVL);  // not sure why -sign is needed here
  delay(1);
  setMotor(ST2  , 2, MtrVR);
  delay(1);
  setMotor(ST2, 1, MtrHL);  // not sure why -sign is needed here
  delay(1);
  setMotor(ST2, 2, MtrHR);
  delay(1);

  // Display on LCD Screen
  if (refresh < refreshMax) {
    refresh += 1;
  } else {
    refresh = 0;
    lcd.setCursor(0, 0);
    lcd.print("               ");
    lcd.setCursor(0, 1);
    lcd.print("               ");
    lcd.setCursor(0, 2);
    lcd.print("               ");
  }

  // Print Motor Values
  lcd.setCursor(0, 0);
  lcd.print("VL: ");
  lcd.print(MtrVL);
  lcd.print(" ");
  lcd.print("Vr: ");
  lcd.print(MtrVR);
  lcd.print(" ");
  lcd.setCursor(0, 1);
  lcd.print("HL: ");
  lcd.print(MtrHL);
  lcd.print(" ");
  lcd.print("HR: ");
  lcd.print(MtrHR);
  lcd.print(" ");

  // Print Raw Joystick Values
  /*lcd.setCursor(0, 2);
  lcd.print(Joy1_V);
  lcd.print(" ");
  lcd.print(Joy2_V);
  lcd.print(" ");
  lcd.print(Joy1_H);
  lcd.print(" ");
  lcd.print(Joy2_H);
  lcd.print(" ");
  lcd.println("Arm Mode:" + armMode);// prints arm mode status;*/
}

int joystickMapping(int input) {

  // ReMap input
  int output = map(input, 0, 1023, -127, 127);

  //Set Motors to 0 if Joysticks are near center
  if (abs(output) <= 10) {
    output = 0;
  }

  // return
  return output;
}

// Initialize motor test parameters
void setMotor(int SabertoothNum, int motorNum, int power) {

  // Set Motor power
  digitalWrite(SabertoothNum, HIGH);
  ST.motor(motorNum, power);
  delayMicroseconds(50);
  digitalWrite(SabertoothNum, LOW);
}
void DoubleTap(){
  if(doubleTap > 0){ // time between
    doubleTapTime --;
  }
  if(doubleTapTime <= 0){
    doubleTap = 0;
    doubleTapTime = doubleTapTimeMax;
  }
  if(doubleTap >= 2){
    if(armMode == false){
      armMode = true;
    }else{
      armMode = false;
    }
    doubleTap = 0;
    doubleTapTime = 0;
  }
}
