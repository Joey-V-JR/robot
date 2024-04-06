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
#include "MeRGBLed.h"

MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);

MeUltrasonicSensor ultraSensor(PORT_8); 

MePort port(PORT_5);
MeRGBLed led(PORT_7);

MeLineFollower lineFinder(PORT_6); 

MeMegaPiDCMotor motor1(PORT1A);
MeMegaPiDCMotor motor2(PORT1B);
MeMegaPiDCMotor motor3(PORT2A);
MeMegaPiDCMotor motor4(PORT2B);

int t, j, f, k;

Servo myservo1;                                                // Create servo object to control a servo 
int16_t servo1pin =  port.pin1();                              // Attaches the servo on PORT_5 SLOT1 to the servo object
int left = 0;
int right = 0;
//int sensorState = lineFinder.readSensor();

// ************************************************************************************

int battV = 10.1;

// **********************************************************************************
bool startTurn = true;
int cylCount = 0;
bool tourette = true;
int servoAng = 20;
bool loopDec = false;
int counter1 = 0;
int bumpC = -1;
bool jerkSpeed = false;
bool endMove = false;
bool bumpCyl = false;
int delayTimeTurn;
int startTurnTime;
int cylError = 0;
int servoAngReset = 15;
bool cyl2TurnCheck = true;
int servoOffSet = 0;
int servoMotorOffSet = 0;
int cylServoTol = 2;


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
  
      if (battV < 10.7)
      {
        delayTimeTurn = 260;
        startTurnTime = 65;
      }
      else if (battV > 11)
      {
        delayTimeTurn = 220;
        startTurnTime = 40;
      }
      else
      {
        delayTimeTurn = 240;
        startTurnTime = 50;
      }
} 

void loop() 
{ 
  
  
  if (tourette == true)
  {
      headN();
      delay (150);
  }
  if (startTurn == true)
  {
    moveFmid ();

  }
  else if (jerkSpeed == true)
  {
    moveFsli ();
  }
  else if (cylCount >= 3 and endMove == false)
  {
    moveFultra();
    turnL();
    moveFultra();
    endMove = true;
  }
  else
  {
    
    moveF();
  }
  counter1++;
  delay (150);

  switch(lineFinder.readSensors())
  {
    case S1_IN_S2_IN: Serial.println("Sensor 1 and 2 are inside of black line");
          
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
                delay(50);
                for(uint8_t t = 0; t <= bumpC + 1; t++)
                {
                  led.setColorAt(t, 254, 0, 0);
                }
                led.show();
                //led.reset();
                delay(50);
                backupFast();
          
          bumpC = bumpC + 1;
          
          if (bumpC == 1 and startTurn == false and cylCount > 0)
          {
            counter1 = 0;
            delay(100);
            jerkSpeed = true;
          }
          
          if (bumpC > 2)
          {
            if (cylCount == 1 or cylCount == 3)
            {
              //turnLsli(250);
              //delay(100);
              //turnL();
              //turnLsli(250);
              //delay(100);
              turnL();
              delay(100);
              //turnLsli();
              //delay(100);
              bumpCreset ();
              
              
            }
            
            if (cylCount == 2)
            {
              turnR();
              delay(100);
              bumpCyl = true;
              bumpCreset ();
             
            }
            
          }
          
          break;
         
              
    default: 
          break;
  }
  delay(50);
      while( startTurn == false and servoAng < 165 + servoOffSet and counter1 > 12 and cylCount < 3)
      {
              
              tourette = false;
              myservo1.write(servoAng + servoOffSet + servoMotorOffSet);
              
              if (servoAng < 20 + servoOffSet)
              {
                delay (100);
              }
              else
              {
                delay (40); // down from 150
              }
              delay (40);
              
              if (servoAng < 20)
              {
                delay(50);
                LEDcylDis(255, 165, 2);
                delay (100);
                
              }
              else
              {
                ultraSonic ();
              }
              
              //ultraSonic ();
              servoAng = servoAng + 5;
              delay (40);
              loopDec = true;      
      }   
      
      if (startTurn == false and loopDec == true and counter1 > 12 and cylCount < 3)
      {
        //IamHERE();
        fullHeadSwep();
        loopDec = false;
        if (servoAng < 20 + servoOffSet)
              {
                delay(50);
              }
        else
        {      
          servoAng = servoAngReset + servoOffSet;
        }
      }
      
      
  
  
  }

 void backupFast () 
  {
          motor1.stop();
          motor2.stop();
          motor3.stop();
          motor4.stop();
          delay(300);
          if (startTurn == true)
          {
            motor1.run(250);
            motor2.run(250);
            motor3.run(-250);
            motor4.run(-250);
            delay(startTurnTime); 
            motor1.stop();
            motor2.stop();
            motor3.stop();
            motor4.stop();
            delay (startTurnTime);
          }

          else
          {
            motor1.run(250);
            motor2.run(250);
            motor3.run(-250);
            motor4.run(-250);
            delay(150); 
            motor1.stop();
            motor2.stop();
            motor3.stop();
            motor4.stop();
            delay (100);
          }
          if (startTurn == true)
          {
            startTurn = false;
            turnR();
            
          }
          
    
  }

  void backUPmeh ()
  {
    {
            motor1.run(250);
            motor2.run(250);
            motor3.run(-250);
            motor4.run(-250);
            delay(50); 
            motor1.stop();
            motor2.stop();
            motor3.stop();
            motor4.stop();
            delay (100);
          }
  }

  void turnL ()
  {
      //backupFast();
      motor1.run(250);
      motor2.run(250);
      motor3.run(250);
      motor4.run(250);
      delay(delayTimeTurn);
      motor1.stop();
      motor2.stop(); 
      motor3.stop();
      motor4.stop(); 
      delay (delayTimeTurn);
  }

  void turnLsli(int speed)
  {
      motor1.run(speed);
      motor2.run(speed);
      motor3.run(speed);
      motor4.run(speed);
      delay(startTurnTime);
      motor1.stop();
      motor2.stop();
      motor3.stop();
      motor4.stop(); 
      delay (startTurnTime);
  }

  void turnR()
  {
    //turn issues
       //backupFast();
      
      motor1.run(-250);
      motor2.run(-250);
      motor3.run(-250);
      motor4.run(-250);
      delay(delayTimeTurn);
      motor1.stop();
      motor2.stop();
      motor3.stop();
      motor4.stop(); 
      delay (delayTimeTurn);
  }

  void turnRsli(int speed)
  {
      motor1.run(-speed);
      motor2.run(-speed);
      motor3.run(-speed);
      motor4.run(-speed);
      delay(startTurnTime);
      motor1.stop();
      motor2.stop();
      motor3.stop();
      motor4.stop(); 
      delay (startTurnTime);
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

  void headN ()
  {
      myservo1.write(90);
      delay(40);
      myservo1.write(91);
      delay (40);
      myservo1.write(89);
      delay(40);
    
  }
  void moveFultra ()
  {
      motor1.run(-250);
      motor2.run(-250);
      motor3.run(250);
      motor4.run(250);
      delay(300); 
      motor1.stop();
      motor2.stop();
      motor3.stop();
      motor4.stop();

  }
  void moveF ()
  {
      motor1.run(-80);
      motor2.run(-80);
      motor3.run(80);
      motor4.run(80);
      delay(300); 
      motor1.stop();
      motor2.stop();
      motor3.stop();
      motor4.stop();
      //delay(400);
  }

  void moveFmid ()
  {
     motor1.run(-70);
      motor2.run(-70);
      motor3.run(70);
      motor4.run(70);
      delay(350); 
      motor1.stop();
      motor2.stop();
      motor3.stop();
      motor4.stop();
      //delay(400);
  }

  void moveFsli ()
  {
      motor1.run(-45);
      motor2.run(-45);
      motor3.run(45);
      motor4.run(45);
      delay(300); 
      motor1.stop();
      motor2.stop();
      motor3.stop();
      motor4.stop();
  }

  void ultraSonic ()
  {
      if(ultraSensor.distanceCm() <= 15)
        {
              motor1.stop();
              motor2.stop();
              motor3.stop();
              motor4.stop();
              LEDcylDis(254,254,254);
              
              //Encoder_1.reset(SLOT1);
              //Encoder_2.reset(SLOT2);
               
              myservo1.write(65 + servoOffSet + servoMotorOffSet);
              delay(500);             
              left = ultraSensor.distanceCm();
              delay(500);
              
              myservo1.write(115 + servoOffSet + servoMotorOffSet);
              delay(500);             
              right = ultraSensor.distanceCm();
              delay(500);
              
              myservo1.write(90 + servoOffSet + servoMotorOffSet);
              delay(200);        
              //do
              {
                if(left > right)
                //if(left > right and abs (left - right) > cylServoTol)
                {
                  backUPmeh ();
                  //backUPmeh ();
                  cylTurnSet();
                  delay (40); 
                  //turnL();
                  delay (40);
                  //turnRsli(100);
                  delay (40);  
                  //cylErrorCheck ();
                  //Encoder_1.moveTo(200,50);
                  //Encoder_2.moveTo(200,50);
                  delay (50);  
                  //cylCount++;
                  //loopDec = false;
                  counter1 = 0;
                }
                if(left < right)
               //if(left < right and abs (left - right) > cylServoTol)
                {
                  backUPmeh ();
                  //backUPmeh ();
                  //turnR();
                  //Encoder_1.moveTo(-200,50);
                  //Encoder_2.moveTo(-200,50);
                  cylTurnSet();
                  delay (50);
                  
                  
                  //cylCount++;
                  //loopDec = false;
                  counter1 = 0;
                }
                /*
                // ****** FUCKING FIX AND TEST HERE *************
                if (abs(left - right) < cylServoTol)
                {
                  LEDcylDis(5, 5, 5);
                  delay (1000); 
                   if (cylCount == 1)
                  {
                    
                    backupFast ();
                    delay (50);
                    turnL();
                    delay (50);
                    turnRsli(250);
                    delay (50);
                    cylCount++;
                    delay (50);
                  }
                  else if (cylCount == 0 or cylCount == 2)
                  {
                    backupFast ();
                    delay (50);
                    turnR();
                    delay (50);
                    cylCount++;
                    delay (50);
                  }
                  counter1 = 0;
                } 
                */
                if(left == right)
                {
                  LEDcylDis(254, 5, 254);
                  cylTurnSet();                 

                  //else 
                  //Encoder_1.moveTo(-400,50);
                  //Encoder_2.moveTo(-400,50);
                }
                  //Encoder_1.loop();
                  //Encoder_2.loop(); 
               }//while(Encoder_1.isTarPosReached() != true && Encoder_2.isTarPosReached() != true); 
          
               delay(200);
            }
        
            
            else if (ultraSensor.distanceCm() <= 22 and ultraSensor.distanceCm() > 15 and servoAng < 90 + servoOffSet )
            {
                if (servoAng < 20 + servoOffSet)
                {
                  turnRsli(250);
                }
                else if (servoAng < 40 + servoOffSet)
                {
                  turnRsli(230);
                }
                else if (servoAng < 60 + servoOffSet)
                {
                  turnRsli(210);
                }
                else if (servoAng < 80 + servoOffSet) 
                {
                  turnRsli(180);
                }
                else
                {
                  LEDcylDis(254, 254, 5);
                  delay (300);
                }
                delay(100);
                moveFsli ();
                servoAng = servoAngReset + servoOffSet;
                // insert code
                LEDcylDis(0, 254, 0);
                delay (200);
                //headN ();
                //delay (400);
                IamHERE();
                
            }

            else if (ultraSensor.distanceCm() <= 22 and ultraSensor.distanceCm() > 15 and servoAng > 90 + servoOffSet)
            {
                if (servoAng > 145 + servoOffSet)
                {
                  turnLsli(250);
                }
                else if (servoAng > 130 + servoOffSet)
                {
                  turnLsli(230);
                }
                else if (servoAng > 115 + servoOffSet)
                {
                  turnLsli(210);
                }
                else if (servoAng > 100 + servoOffSet)
                {
                  turnLsli(180);
                }
                else
                {
                  LEDcylDis(254, 254, 5);
                  delay (300);
                }
                delay(100);
                moveFsli ();
                servoAng = servoAngReset + servoOffSet;
                // insert code
                LEDcylDis(0, 254, 0);
                delay(200);
                IamHERE();
                //headN ();
                //delay (400);
            }

            else if (ultraSensor.distanceCm() <= 33 and ultraSensor.distanceCm() > 22 and servoAng < 90 + servoOffSet)
            {
                if (servoAng < 20 + servoOffSet)
                {
                  turnRsli(250);
                }
                else if (servoAng < 40 + servoOffSet)
                {
                  turnRsli(220);
                }
                else if (servoAng < 60 + servoOffSet)
                {
                  turnRsli(200);
                }
                else if (servoAng < 80 + servoOffSet)
                {
                  turnRsli(180);
                }
                else
                {
                  LEDcylDis(254, 254, 5);
                  delay (300);
                }
                delay(100);
                moveF ();
                servoAng = servoAngReset + servoOffSet;
                LEDcylDis(0, 0, 254);
                delay(400);
                IamHERE();
                //headN ();
                //delay (400);
            }

            else if (ultraSensor.distanceCm() <= 33 and ultraSensor.distanceCm() > 22 and servoAng > 90 + servoOffSet)
            {
                if (servoAng > 145 + servoOffSet)
                {
                  turnLsli(250);
                }
                else if (servoAng > 130 + servoOffSet)
                {
                  turnLsli(220);
                }
                else if (servoAng > 115 + servoOffSet)
                {
                  turnLsli(200);
                }
                else if (servoAng > 100 + servoOffSet)
                {
                  turnLsli(180);
                }
                else
                {
                  LEDcylDis(254, 254, 5);
                  delay (300);
                }
                delay(100);
                moveF ();
                servoAng = servoAngReset + servoOffSet;
                LEDcylDis(0, 0, 254);
                delay(400);
                IamHERE();
                //headN ();
                //delay (400);
            }
            else if (ultraSensor.distanceCm() <= 22 and ultraSensor.distanceCm() > 15 and servoAng == 90 + servoOffSet)
            {
              moveF ();
                servoAng = servoAngReset + servoOffSet;
                LEDcylDis(0, 254, 0);
                delay(400);
            }
            else if (ultraSensor.distanceCm() <= 33 and ultraSensor.distanceCm() > 15 and servoAng == 90 + servoOffSet)
            {
             moveF ();
                servoAng = servoAngReset + servoOffSet;
                LEDcylDis(0, 0, 254);
                delay(400); 
            }
  }

void bumpCreset ()
  {

        jerkSpeed = false;
        bumpC = 0;
        led.reset(PORT_7);
        if (cylCount == 2 and bumpCyl == true)
        {
          counter1 = 8;
          bumpC = false;
        }
        else if (cylCount == 2 and bumpCyl == false)
        {
          counter1 = 2;
        }
        else
        {
          counter1 = 0;
        }
  }


void color_loop ()
    {
    for(uint8_t t = 0; t < 3; t++)
    {
      uint8_t red	= 64 * (1 + sin(t / 2.0 + j / 4.0) );
      uint8_t green = 64 * (1 + sin(t / 1.0 + f / 9.0 + 2.1) );
      uint8_t blue = 64 * (1 + sin(t / 3.0 + k / 14.0 + 4.2) );
      led.setColorAt(t, red, green, blue);
    }
    led.show();
    j += random(1, 6) / 6.0;
    f += random(1, 6) / 6.0;
    k += random(1, 6) / 6.0;
  }  

  void LEDcylDis(int R, int G, int B)

  {
                delay(400);
                led.reset(PORT_7);
                delay(50);
                for(t = 0; t <= cylCount; t++)
                  {
                    uint8_t red	= 64 * (1 + sin(t / 2.0 + j / 4.0) );
                    uint8_t green = 64 * (1 + sin(t / 1.0 + f / 9.0 + 2.1) );
                    uint8_t blue = 64 * (1 + sin(t / 3.0 + k / 14.0 + 4.2) );
                    led.setColorAt(t, R, G, B);
                  }
                  led.show();
                  delay(400);

  }  


  
  void fullHeadSwep()
  {
        myservo1.write(90);
        delay (250);
        myservo1.write(180);
        delay (250);
        myservo1.write(0);
        delay (250);
        myservo1.write(90);
        delay (250);
  }

  void IamHERE()
  {
    fullHeadSwep();
    LEDcylDis(5, 5, 5);
    delay (2000);
    fullHeadSwep();
    delay (2000);
                
 }


 void cylTurnSet()
 {
    if (cylCount == 1)
                  {
                    
                    backupFast ();
                    delay (50);
                    turnL();
                    delay (50);
                    turnRsli(250);
                    delay (50);
                    cylCount++;
                    delay (50);
                  }
                  else if (cylCount == 0 or cylCount == 2)
                  {
                    backupFast ();
                    delay (50);
                    turnRsli(250);
                    turnRsli(250);
                    delay (50);
                    //turnLsli(200);
                    //delay (50);
                    cylCount++;
                    delay (50);
                  }
                  counter1 = 0;
                  delay(1000);
 }
/*
 void cylErrorCheck ()
  {
    moveFsli();
    delay (3000);
    if (cylCount == 2)
    {
       myservo1.write(145 + servoOffSet + servoMotorOffSet);
      delay(400);
      if(ultraSensor.distanceCm() <= 21)
      {
        cylError = cylError + 1;
        errorLED ();
      }
      else
      {
          myservo1.write(123 + servoOffSet + servoMotorOffSet);
          delay(400);
          if(ultraSensor.distanceCm() <= 21)
          {
            cylError = cylError + 1;
            errorLED ();
          }
          else
          {
            myservo1.write(100 + servoOffSet + servoMotorOffSet);
            delay(400);
            if(ultraSensor.distanceCm() <= 21)
            {
              cylError = cylError + 1;
              errorLED ();
            }
          }
        }
    }
    else
    {
      myservo1.write(35 + servoOffSet + servoMotorOffSet);
      delay(100);
      if(ultraSensor.distanceCm() <= 21)
      {
        cylError = cylError + 1;
        errorLED ();
      }
      else
      {
          myservo1.write(50 + servoOffSet + servoMotorOffSet);
          delay(400);
          if(ultraSensor.distanceCm() <= 21)
          {
            cylError = cylError + 1;
            errorLED ();
          }
          else
          {
            myservo1.write(75 + servoOffSet + servoMotorOffSet);
            delay(400);
            if(ultraSensor.distanceCm() <= 21)
            {
              cylError = cylError + 1;
              errorLED ();
            }
          }
        }
    }
  }
*/
  void errorLED ()
  {
      led.setColorAt(3, 5, 5, 5);
                  
      led.show();
      delay(400);
  }

  
