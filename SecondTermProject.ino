#include <C:\Users\KaanPC\Desktop\SecondTermProject\Libraries.h>
#include <Arduino.h>
#include <Wire.h>
#include <SD.h>
#include <Servo.h> 
#include <SPI.h>
#include <user_config.h>
#include <time.h>
#include <TimeLib.h>
#include <inttypes.h>

int pirPin1 = 14;
int pirPin2 = 15;
int pirPin3 = 16;
int pirValue1;
int pirValue2;
int pirValue3;

Servo leftServo;
Servo centerServo;
Servo rightServo;
int pos = 0;   // keep servo position value
int leftServoPin = 22;
int centerServoPin = 24;
int rightServoPin = 26;

void setup()
{
  leftServo.attach(leftServoPin)); 
  centerServo.attach(centerServoPin);
  rightServo.attach(rightServoPin);
  
  rtc.begin();
  rtc.autoTime();
  if (!rtc.isrunning())
  {
    Serial.println("Clock is not running");
  }
  
  pinMode(pirPin1, INPUT);  
  pinMode(pirPin2, INPUT);   
  pinMode(pirPin3, INPUT);  
  noInterrupts(); //Disable all interrupts
  XCLK_SETUP();   //Setup 8MHz clock at pin 11
  OV7670_PINS();  // Setup Data-in and interrupt pins from camera
  
  delay(1000);
  
  TWI_SETUP();    // Setup SCL for 100KHz
  interrupts();
  Wire.begin();
  Init_OV7670();
  Init_QVGA();
  Init_YUV422();
  WriteOV7670(0x11, 0x1F); //Range 00-1F
  noInterrupts();
  Serial.begin(9600);
  pinMode(CS_Pin, OUTPUT);
  SD.begin(CS_Pin);
}

void loop()
{  
  int leftPir = digitalRead(pirPin1); // read pin1 value
  int centerPir = digitalRead(pirPin2); // read pin2 value
  int rightPir = digitalRead(pirPin3); // read pin3 value
  Serial.println(leftPir);
  Serial.println(centerPir);
  Serial.println(rightPir);

  if(centerPir = HIGHT)
  {
    QVGA_Image(print_time()+".bmp"); // Capture image
    delay(2000);
  }
  else
  {
    if(leftPir = HIGH)
    {
      rightServo.write(90);
      centerServo.write(90);
      QVGA_Image(print_time()+".bmp"); // Capture image
      delay(2000);
      centerServo.write(270);
      rightServo.write(90);
    }
    else
    {
      leftServo.write(90);
      centerServo.write(270);
      QVGA_Image(print_time()+".bmp"); // Capture image
      delay(2000);
      centerServo.write(90);
      leftServo.write(90);
    }
  }
 
  delay(1000);
   while(1);
}
