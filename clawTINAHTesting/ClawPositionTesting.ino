#include <phys253.h>          
#include <LiquidCrystal.h>
#include "ClawTINAH.h"

#define ARM_SERVO_PIN 9
#define CLAW_SERVO_PIN 8

#define CLAW_SERVO_OPEN 0
#define CLAW_SERVO_CLOSE 140
#define CLAW_SERVO_DELAY_MS 2

#define ARM_SERVO_UP 0
#define ARM_SERVO_DOWN 150
#define ARM_SERVO_DELAY_MS 5


Claw * newClaw;
 ServoTINAH newServo;
 
void setup() {
 #include <phys253setup.txt>
  Serial.begin(9600) ;

  // setting up claw object
  newClaw = new Claw(CLAW_SERVO_OPEN, CLAW_SERVO_CLOSE, ARM_SERVO_UP,CLAW_SERVO_DELAY_MS,ARM_SERVO_DELAY_MS,CLAW_SERVO_PIN,ARM_SERVO_PIN);

}

void loop() {
    int armDownDegree = 0; 

    while(!startbutton()){
        LCD.home();
        armDownDegree = map(knob(6), 0, 1024, 0, 170);
        LCD.print(armDownDegree);
        delay(100);
        LCD.clear();
        
    }
    LCD.home();
    LCD.print("Select");
    delay(1000);
    LCD.clear();

    if(startbutton() )
    {
          LCD.print("Setup");
          delay(500);
         newClaw ->clawSetUp();
    }else if(stopbutton()){
        LCD.print("Running");
         delay(500);
          newClaw->retrieve(armDownDegree);
    }
   
     
        LCD.clear();
  delay(2000);

  
//    //newServo.write(90);

//    //newServo.write(180);
//    newClaw->retrieve(ARM_SERVO_DOWN);

}
