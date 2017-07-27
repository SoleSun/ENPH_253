/* 
 *  Main file that switches between states
 *  and handles the UI 
 */
 
#include <phys253.h>
#include <LiquidCrystal.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include "Configuration.h"
#include "MenuItem.h"
#include "Encoder.h"
#include "Gate_Navigator.h"
#include "TestProcedures.h"

/*
 * A cross is black tape that crosses the path, indicating either the entrance
 * to the agent bowl or the location of the agents. Once the first cross is 
 * encountered, the code assumes that following crosses are agent locations
 */
int noOfCrossesEncountered = 0;

uint16_t MenuItem::MenuItemCount = 0;
/* Add the menu items here */
MenuItem * Speed;
MenuItem * ProportionalGain;
MenuItem * DerivativeGain;
MenuItem * Threshold; /* Thershold for detecting black line */
MenuItem * Error;
MenuItem * DistanceToGate; 
MenuItem * ThresholdGate;
MenuItem * LeftTargetDistanceValue;
MenuItem * RightTargetDistanceValue;
MenuItem * ManeuverLeftConstant;
MenuItem * ManeuverRightConstant;
MenuItem * MinMotorSpeed;
MenuItem * menuItems[numberOfPIDMenuOptions];
Gate_Navigator * gateSequence;

// *array[1]
// **(array + 1)

/* 
 *  Mediator responsible for delegating and 
 *  switching between the different states
 */
void Pilot() {
  gateSequence = 
  new Gate_Navigator (Threshold->Value, ProportionalGain->Value, DerivativeGain->Value, Speed->Value, DistanceToGate->Value, ThresholdGate->Value);
  gateSequence->Drive();
  
}

void testMenu() {
  LCD.clear(); LCD.home();
  LCD.print("Entering");
  LCD.setCursor(0,1); LCD.print("Test Menu");
  delay(500);

  TestProcedures t = TestProcedures (); 
  
  while (true) {
    int menuIndex = map(knob(6), 0, 1023, 0, 6);

    LCD.clear(); LCD.home();
    switch (menuIndex){
      case 0:
        LCD.print(">Gate PID");
        LCD.setCursor(0,1); LCD.print("Motor Lift");
        break;
      case 1:
        LCD.print("Gate >PID");
        LCD.setCursor(0,1); LCD.print("Motor Lift");
        break;
      case 2:
        LCD.print("Gate PID");
        LCD.setCursor(0,1); LCD.print(">Motor Lift");
        break;
      case 3:
        LCD.print("Gate PID");
        LCD.setCursor(0,1); LCD.print("Motor >Lift");
        break;
      case 4:
        LCD.print("Motor Lift");
        LCD.setCursor(0,1); LCD.print(">MinMotor Maneuver");
        break;
     case 5: 
      LCD.print("Motor Lift");
      LCD.setCursor(0,1); LCD.print("MinMotor >Maneuver");
      break;
    }
    delay(100);

    if (startbutton()) {
      delay(100);
      if (startbutton()) { 
        switch (menuIndex){
          case 0:
            t.testGateSensors();
            break;
          case 1:
            t.testPID(Threshold->Value, ProportionalGain->Value, DerivativeGain->Value, Speed->Value);
            break;
          case 2:
            t.testMotors();
            break;
          case 3:
//            t.testLifts();
            break;
          case 4;
            t.testMinMotor();
            break;
          case 5;
            t.testManeuver(LeftTargetDistanceValue->Value,RightTargetDistanceValue->Value,ManueverLeftConstant->Value, ManueverRightConstant->Value, MinMotorSpeed->Value);
            break;
        }
      } // if - cross check start button
    }

    if (stopbutton())
    {
      delay(100);
      if (stopbutton())
      { 
        LCD.clear(); LCD.home();
        LCD.print("Exiting");
        LCD.setCursor(0,1); LCD.print("Test Menu");
        delay(500);
        return;
      }
    }
  }
}

void PIDMenu() {
  LCD.clear(); LCD.home();
  LCD.print("Entering");
  LCD.setCursor(0,1); LCD.print("PID Menu");
  delay(500);
 
  while (true) {
    /* Show MenuItem value and knob value */
    int menuIndex = map(knob(6), 0, 1023, 0, numberOfPIDMenuOptions); /* Menu items plus the Drive option */
    
    LCD.clear(); LCD.home();
    LCD.print(menuItems[menuIndex]->Name); LCD.print(" "); LCD.print(menuItems[menuIndex]->Value);
    LCD.setCursor(0, 1);
    LCD.print("Set to "); LCD.print(knob(7)); LCD.print("?");
    delay(100);
    
    /* Press start button to save the new value */
    if (startbutton())
    {
      delay(100);
      if (startbutton())
      { 
        menuItems[menuIndex]->Value = knob(7);
        menuItems[menuIndex]->Save();
        delay(250);
      } // if - cross check start button
    } //if - first check start button

    if (stopbutton())
    {
      delay(100);
      if (stopbutton())
      { 
        LCD.clear(); LCD.home();
        LCD.print("Exiting");
        LCD.setCursor(0,1); LCD.print("PID Menu");
        delay(500);
        return;
      }
    }
  }
}

void mainMenu()
{
  LCD.clear(); LCD.home();
  LCD.print("Entering menu");
  delay(500);
 
  while (true){
    int menuIndex = map(knob(6), 0, 1024, 0, numberOfMainMenuOptions); /* Menu items plus the Drive option */
    
    LCD.clear(); LCD.home();
    switch (menuIndex){
      case 0:
        LCD.print("->PID Drive");
        LCD.setCursor(0,1); LCD.print("Test");
        break;
      case 1:
        LCD.print("PID ->Drive");
        LCD.setCursor(0,1); LCD.print("Test");
        break;
      case 2:
        LCD.print("PID Drive");
        LCD.setCursor(0,1); LCD.print("->Test");
        break;
    }
    delay(100);

    if (startbutton()) {
      delay(100);
      if (startbutton()) { 
        switch (menuIndex){
          case 0:
            PIDMenu();
            break;
          case 1:
            Pilot();
            break;
          case 2:
            testMenu();
            break;
        }
      } // if - cross check start button
    } //if - first check start button
  }
}

void setup()
{
  LCD.clear();
  LCD.home();
  #include <phys253setup.txt>
  LCD.print("Welcome!");
  delay(1000);
  Serial.begin(9600);

  Speed                    = new MenuItem("Speed");
  ProportionalGain         = new MenuItem("P-gain");
  DerivativeGain           = new MenuItem("D-gain");
  Threshold                = new MenuItem("Threshold");
  Error                    = new MenuItem("Error");
  DistanceToGate           = new MenuItem("GateDist");
  ThresholdGate            = new MenuItem("ThreshGate");
  LeftTargetDistanceValue  = new MenuItems("LeftTargetDistance");
  RightTargetDistanceValue = new MenuItems("RightTargetDistance");
  ManeuverLeftConstant     = new MenuItems("LeftManeuverP");
  ManeuverRightConstant    = new MenuItems("RightManeuverP");
  MinMotorSpeed            = new MenuItems("MinMotorSpeed");
 
  menuItems[0]             = Speed; 
  menuItems[1]             = ProportionalGain; 
  menuItems[2]             = DerivativeGain; 
  menuItems[3]             = Error;
  menuItems[4]             = Threshold;
  menuItems[5]             = DistanceToGate;
  menuItems[6]             = ThresholdGate;
  menuItems[7]             = LeftTargetDistanceValue;
  menuItems[8]             = RightTargetDistanceValue;
  menuItems[9]             = ManeuverLeftConstant;
  menuItems[10]            = ManeuverRightConstant;
  menuItems[11]            = MinMotorSpeed;
}
 
void loop()
{
  mainMenu();
}
