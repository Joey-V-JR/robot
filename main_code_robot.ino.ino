/* 
 *  This program is used to introduce students to motor encoders for the Ultimate 2.0 Robots used in EGR111. While it doesn't encompass
 * all encoder commands it can be used to see the code structure to get them to work. Encoders will allow for a robot to move a more  
 * exact distance in degree and with more programming be used to calculate distance traveled or heading. Be careful though, since  
 * the IR sensor can get bent on the back of the motor which will cause the encoder to not work and will result in unpredictable behavior.
 * For more explanation on the commands the encoder library can perform go to https://github.com/Makeblock-official/Makeblock-Libraries/blob/master/src/MeEncoderOnBoard.cpp
 * Here you can see all command and a brief description of what they do, note that not all commands can be used in the main loop and some when called 
 * successively can stop the robot from moving. Use the serial.print function to make sure the encoders are working properly is something isn't working properly
 * Finally, try to avoid long delay commands when using encoders. Because there is an interrupt process delays longer than 500ms can cause some issues in the 
 * encoders ability to count pulses.
 * 
 * Good Luck,
 * NF
 */

#include "MeMegaPi.h"

MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);

MeUltrasonicSensor ultraSensor(PORT_8); 

MePort port(PORT_5);

MeLineFollower lineFinder(PORT_6); 

MeMegaPiDCMotor motor1(PORT1A);
MeMegaPiDCMotor motor2(PORT1B);
MeMegaPiDCMotor motor3(PORT2A);
MeMegaPiDCMotor motor4(PORT2B);

Servo myservo1;                                                // Create servo object to control a servo 
int16_t servo1pin =  port.pin1();                              // Attaches the servo on PORT_5 SLOT1 to the servo object
int left = 0;
int right = 0;
//int sensorState = lineFinder.readSensor();

void isr_process_encoder1(void)                               // This sets up the interrupt process for encoder 1
{
  if(digitalRead(Encoder_1.getPortB()) == 0)
  {
    Encoder_1.pulsePosMinus();
  }
  else
  {
    Encoder_1.pulsePosPlus();
  }
}

void isr_process_encoder2(void)                               // This sets up the interrupt process for encoder 1
{
  if(digitalRead(Encoder_2.getPortB()) == 0)
  {
    Encoder_2.pulsePosMinus();
  }
  else
  {
    Encoder_2.pulsePosPlus();
  }
}


void setup() 
{ 
  myservo1.attach(servo1pin);                                               // attaches the servo on servopin1
  
  
  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);     //attaches interupt to encoder 1 to count the pulses 
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);     //attaches interupt to encoder 2 to count the pulses

  Serial.begin(115200);
  
  //Set PWM 8KHz
  TCCR1A = _BV(WGM10);                                  //Addresses pins on the motor driver IC DO NOT CHANGE!
  TCCR1B = _BV(CS11) | _BV(WGM12);                      //Addresses pins on the motor driver IC DO NOT CHANGE!

  TCCR2A = _BV(WGM21) | _BV(WGM20);                     //Addresses pins on the motor driver IC DO NOT CHANGE!
  TCCR2B = _BV(CS21);                                   //Addresses pins on the motor driver IC DO NOT CHANGE!

  Encoder_1.setPulse(7);                                //Sets the number of pulse per one encoder revolution DO NOT CHANGE!
  Encoder_2.setPulse(7);                                //Sets the number of pulse per one encoder revolution DO NOT CHANGE!
  Encoder_1.setRatio(26.9);                             //Sets the ratio of pulse per one motor shaft revolution DO NOT CHANGE!
  Encoder_2.setRatio(26.9);                             //Sets the ratio of pulse per one motor shaft revolution DO NOT CHANGE!
  Encoder_1.setPosPid(1.8,0,1.2);                       //Sets the PIDs for maintaining a given postion DO NOT CHANGE!
  Encoder_2.setPosPid(1.8,0,1.2);                       //Sets the PIDs for maintaining a given postion DO NOT CHANGE!
  Encoder_1.setSpeedPid(0.18,0,0);                      //Sets the PIDs for maintaining a given speed DO NOT CHANGE!
  Encoder_2.setSpeedPid(0.18,0,0);                      //Sets the PIDs for maintaining a given speed DO NOT CHANGE!!
} 

void loop() 
{ 
  myservo1.write(90);
  delay(150);
   myservo1.write(91);
  delay (150);
  myservo1.write(89);
  delay(150);      
  motor1.run(-50);
  motor2.run(-50);
  motor3.run(50);
  motor4.run(50);
  delay(400); 
  motor1.stop();
  motor2.stop();
  motor3.stop();
  motor4.stop();
  
  /*
  delay(1000);
  motor1.stop();
  motor2.stop();
  motor3.stop();
  motor4.stop();
  delay(1000);
*/
  switch(lineFinder.readSensors())
  {
    case S1_IN_S2_IN: Serial.println("Sensor 1 and 2 are inside of black line");
          ;
          break;
          
    case S1_IN_S2_OUT: Serial.println("Sensor 2 is outside of black line");
          //backupFast();
          turnR();
          break;
          
    case S1_OUT_S2_IN: Serial.println("Sensor 1 is outside of black line");
          //backupFast();
          turnL();      
          break;
          
    case S1_OUT_S2_OUT: Serial.println("Sensor 1 and 2 are outside of black line"); 
          backupFast();
          
          
          break;
         
              
    default: 
          break;
  }

 
/*  
  if(ultraSensor.distanceCm() <= 25)
  {
     motor1.stop();
     motor2.stop();
     motor3.stop();
     motor4.stop();
     delay(2000);

    Encoder_1.reset(SLOT1);
    Encoder_2.reset(SLOT2);
     
    myservo1.write(65);
    delay(1000);             
    left = ultraSensor.distanceCm();
    delay(1000);

    myservo1.write(115);
    delay(1000);             
    right = ultraSensor.distanceCm();
    delay(1000);

    myservo1.write(90);
    delay(200);        
    do
    {
      if(left > right)
      {
        Encoder_1.moveTo(200,50);
        Encoder_2.moveTo(200,50);
      }
     if(left < right)
      {
        Encoder_1.moveTo(-200,50);
        Encoder_2.moveTo(-200,50);
      }
      if(left == right)
      {
        Encoder_1.moveTo(-400,50);
        Encoder_2.moveTo(-400,50);
      }
        Encoder_1.loop();
        Encoder_2.loop(); 
     }while(Encoder_1.isTarPosReached() != true && Encoder_2.isTarPosReached() != true); 

     delay(200);
  }
  */
}

 void backupFast () 
  {
          motor1.stop();
          motor2.stop();
          motor3.stop();
          motor4.stop();
          delay(2000);
          motor1.run(250);
          motor2.run(250);
          motor3.run(-250);
          motor4.run(-250);
          delay(200); 
          motor1.stop();
          motor2.stop();
          motor3.stop();
          motor4.stop();
          
    
  }

  void turnL ()
  {
      //backupFast();
      motor1.run(250);
      motor2.run(250);
      delay(250);
      motor1.stop();
      motor2.stop(); 
      delay (250);
  }

  void turnR()
  {
    //turn issues
       //backupFast();
      motor3.run(-150);
      motor4.run(-150);
      delay(150);
      motor1.stop();
      motor2.stop(); 
      delay (150);
  }

  void turnCyl ()

  {

          myservo1.write(180);
          delay(300);
          motor1.run(-250);
          motor2.run(-250);
          delay (200);
          motor1.stop();
          motor2.stop();
          delay (200);
          motor1.run(-250);
          motor2.run(-250);
          motor3.run(250);
          motor4.run(250);
          delay(400); 
          motor1.stop();
          motor2.stop();
          motor3.stop();
          motor4.stop();
          delay(200); 
          motor3.run(250);
          motor4.run(250);
          delay (300);
          motor3.stop();
          motor4.stop();
          //sound detect here
          motor1.run(-250);
          motor2.run(-250);
          motor3.run(250);
          motor4.run(250);
          delay(100); 
          motor1.stop();
          motor2.stop();
          motor3.stop();
          motor4.stop();
          delay(200);
          motor3.run(250);
          motor4.run(250);
          delay (300);
          motor3.stop();
          motor4.stop();
          delay (200);
          motor1.run(-250);
          motor2.run(-250);
          motor3.run(250);
          motor4.run(250);
          delay(100); 
          motor1.stop();
          motor2.stop();
          motor3.stop();
          motor4.stop();
          delay(4000); 
          motor1.run(-250);
          motor2.run(-250);
          delay (200);
          motor1.stop();
          motor2.stop();

  }

  
