#include <phys253.h>
#include <LiquidCrystal.h>

void setup() {
  // put your setup code here, to run once:
  #include <phys253setup.txt>
  Serial.begin(9600);
  LCD.clear(); LCD.home();
  LCD.print("Ahn yeoung!");
  delay(2000);
}

void loop() {
  // put your main code here, to run repeatedly:
  //Left, Center Left, Center Right, Right
  double 
  L = analogRead(0),
  CL = analogRead(1),
  CR = analogRead(2),
  R = analogRead(3);

  LCD.clear(); LCD.home();
  LCD.print(L); LCD.print(" "); LCD.print(CL);
  LCD.setCursor(0,1);
  LCD.print(CR); LCD.print(" "); LCD.print(R);
  
  delay(50);
}
