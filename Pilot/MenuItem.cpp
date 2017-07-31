#include <avr/eeprom.h>
#include "include/MenuItem.h"

MenuItem::MenuItem (String menuName) {
	  MenuItemCount++;
    EEPROMAddress = (uint16_t*)(2 * MenuItemCount);
    Name      = menuName;
    Value         = eeprom_read_word(EEPROMAddress);
}

void MenuItem::Save() {
    eeprom_write_word(EEPROMAddress, Value);
<<<<<<< HEAD
}
=======
}
>>>>>>> 3df4dbc030fe0c0648df8af69a0aaaaddf1a7d71
