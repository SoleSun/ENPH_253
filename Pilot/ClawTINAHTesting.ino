#include <phys253.h>          
#include <LiquidCrystal.h>
#include "ClawTINAH.h"

#define ARM_SERVO_PIN 9
#define CLAW_SERVO_PIN 8

#define CLAW_SERVO_OPEN 130
#define CLAW_SERVO_CLOSE 0
#define CLAW_SERVO_DELAY_MS 10

#define ARM_SERVO_UP 0
#define ARM_SERVO_DOWN 150
#define ARM_SERVO_DELAY_MS 15


Claw * newClaw;
 ServoTINAH newServo;
 
void setup() {
 #include <phys253setup.txt>
  Serial.begin(9600) ;

  // setting up claw object
  newClaw = new Claw(CLAW_SERVO_OPEN, CLAW_SERVO_CLOSE, ARM_SERVO_UP,CLAW_SERVO_DELAY_MS,ARM_SERVO_DELAY_MS,CLAW_SERVO_PIN,ARM_SERVO_PIN);

}

void loop() {
    
    // 
    LCD.home();
    LCD.print("twoAnimal");
    delay(2000);
    //    delay(2000);
    //newClaw ->clawSetUp();
        newClaw->retrieve(ARM_SERVO_DOWN);
        LCD.clear();
  delay(2000);

  
//    //newServo.write(90);

//    //newServo.write(180);
//    newClaw->retrieve(ARM_SERVO_DOWN);

}
