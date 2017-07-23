#include <phys253.h>
#include <LiquidCrystal.h>
#include <avr/eeprom.h>

class Menu {
  private: 

    
  public:
    void showMenu () {
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

uint16_t MenuItem::MenuItemCount = 0;
/* Add the menu items here */
MenuItem Speed            = MenuItem("Speed");
MenuItem ProportionalGain = MenuItem("P-gain");
MenuItem DerivativeGain   = MenuItem("D-gain");
MenuItem IntegralGain     = MenuItem("I-gain");
MenuItem Threshold        = MenuItem("Threshold");
MenuItem menuItems[]      = {Speed, ProportionalGain, DerivativeGain, Threshold};

void PIDMenu() {
  
}

void mainMenu()
{
}
