#include <phys253.h>
#include <LiquidCrystal.h>

void setup() {
  // put your setup code here, to run once:
  #include <phys253setup.txt>
  Serial.begin(9600);
  LCD.clear(); LCD.home();
  LCD.print("Hello!");
}

String cmd = ""; 

void loop() {
  LCD.clear(); LCD.home();
  // put your main code here, to run repeatedly:
   int angle = knob(6);
   int servo0 = map(angle, 0, 1023, 0, 180) ;
   RCServo0.write(servo0);
   
   LCD.setCursor(0, 0); LCD.print(servo0);
   delay(100);
}
