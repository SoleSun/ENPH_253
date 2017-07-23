#include <phys253.h>
#include <LiquidCrystal.h>
#include <avr/eeprom.h>

class Menu {
  public:
    void displayOptions () {
      
    }
    
  private:
    
}

class MenuItem
{
public:
  String    Name;
  uint16_t  Value;
  uint16_t* EEPROMAddress;
  static uint16_t MenuItemCount;
  MenuItem(String name)
  {
    MenuItemCount++;
    EEPROMAddress = (uint16_t*)(2 * MenuItemCount);
    Name      = name;
    Value         = eeprom_read_word(EEPROMAddress);
  }
  void Save()
  {
    eeprom_write_word(EEPROMAddress, Value);
  }
};

//Global Variables
const int leftMotor = 0, rightMotor = 3;
/*
 * A cross is black tape that crosses the path, indicating either the entrance
 * to the agent bowl or the location of the agents. Once the first cross is 
 * encountered, the code assumes that following crosses are agent locations
 */
int noOfCrossesEncountered = 0;

uint16_t MenuItem::MenuItemCount = 0;
/* Add the menu items here */
MenuItem Speed            = MenuItem("Speed");
MenuItem ProportionalGain = MenuItem("P-gain");
MenuItem DerivativeGain   = MenuItem("D-gain");
MenuItem IntegralGain     = MenuItem("I-gain");
MenuItem Threshold        = MenuItem("Threshold");
MenuItem menuItems[]      = {Speed, ProportionalGain, DerivativeGain, Threshold};

void setup()
{
  LCD.clear();
  LCD.home();
  #include <phys253setup.txt>
  LCD.print("Welcome!");
  delay(2000);
  Serial.begin(9600);
}
 
void loop()
{
  mainMenu();
}

//Depending on the platform, robot will either make CW or CCW 
//rotation around the agent circle
void turnCW (){
  motor.speed(leftMotor, Speed.Value);
  motor.speed(rightMotor, Speed.Value);
  delay(100);
  return;
}

void turnCCW () {
  motor.speed(leftMotor, -Speed.Value);
  motor.speed(rightMotor, -Speed.Value);
  delay(100);
  return;
}

void retrieveAgent () {
  delay(100);
  return;
}

void Drive() {
  int lastError = 0, recentError = 0;
  int q = 0, m = 0, con = 0;

  while (true){
    double 
    L = analogRead(0),
    CL = analogRead(1),
    CR = analogRead(2),
    R = analogRead(3); 

    //The first cross has been detected
    if ( (L > Threshold.Value) && (CL > Threshold.Value) && (CR > Threshold.Value) && (R > Threshold.Value)){
      noOfCrossesEncountered++;

      if (noOfCrossesEncountered == 1){
        turnCCW();
      } else{
        retrieveAgent();
      }
    } 
    
    else {
      int error;
      if ( (CL > Threshold.Value) && (CR > Threshold.Value) )       error = 0;
      else if ( (CL > Threshold.Value)&&(CR < Threshold.Value) )    error = -1;
      else if ( (CL < Threshold.Value)&&(CR > Threshold.Value) )    error = 1;
      else{
         if( lastError > 0 )    error = 5;
         else                 error = -5;
      }
    
      if(!(error == lastError)){
        recentError = lastError;
        q=m;
        m=1;
      }
    
      int proportional = ProportionalGain.Value * error,
          derivative   = (int) (DerivativeGain.Value * (float)(error - recentError) / (q + m) );
      con = proportional + derivative;
    
      m++;
      motor.speed(leftMotor, -Speed.Value + con);
      motor.speed(rightMotor, Speed.Value + con);
    
      lastError = error;
    }
  
    LCD.clear(); LCD.home();
    LCD.print(CL); LCD.print(" "); LCD.print(CR);
    LCD.setCursor(0,1);
    LCD.print(L); LCD.print(" "); LCD.print(R);
    
    delay(50);

    if (stopbutton())
    {
      delay(100);
      if (stopbutton())
      { 
        noOfCrossesEncountered = 0;
        motor.speed(leftMotor, 0);
        motor.speed(rightMotor, 0);
        LCD.clear(); LCD.home();
        LCD.print("Exit Drive");
        delay(500);
        return;
      }
    }
  }
}

void PIDMenu() {
  
}

void mainMenu()
{
  LCD.clear(); LCD.home();
  LCD.print("Entering menu");
  delay(500);
 
  while (true)
  {
    /* Show MenuItem value and knob value */
    int menuIndex = map(knob(6), 0, 1023, 0, MenuItem::MenuItemCount + 1); /* Menu items plus the Drive option */
    
    LCD.clear(); LCD.home();
    if (menuIndex > MenuItem::MenuItemCount){
      LCD.print ("Drive?");  
    }
    else{
      LCD.print(menuItems[menuIndex].Name); LCD.print(" "); LCD.print(menuItems[menuIndex].Value);
      LCD.setCursor(0, 1);
      LCD.print("Set to "); LCD.print(knob(7)); LCD.print("?");
      delay(100);
    }
    
    /* Press start button to save the new value */
    if (startbutton())
    {
      delay(100);
      if (startbutton())
      { 
        if (menuIndex > MenuItem::MenuItemCount){
          Drive();
        }
        else{
          menuItems[menuIndex].Value = knob(7);
          menuItems[menuIndex].Save();
          delay(250);
        } // if - Assigning parameter value 
      } // if - cross check start button
    } //if - first check start button
    
  }
}
