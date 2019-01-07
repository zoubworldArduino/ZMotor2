//EXAMPLE OF SERVO WITH ROS.
// do
// roscore &
// for raspberry pi : rosrun rosserial_python serial_node.py /dev/ttyUSB0 &
// or for windows on COM4 : 
//        sudo chmod 666 /dev/ttyS4 if COM4
//        sudo chmod 666 /dev/ttyS24 if COM24
//        rosrun rosserial_python serial_node.py /dev/ttyS4 & 
//        rosrun rosserial_python serial_node.py /dev/ttyS24 & 
// rostopic pub motor2/A std_msgs/Int16 --once     0
// rostopic pub motor2/C std_msgs/Int16 --once     180
// rostopic pub motor2/A std_msgs/Int16 --once -- -180
// rostopic pub motor2/C std_msgs/Int16 --once -- -1080


// NOTE : WINDOWS 10 with ubuntu 16.04 as subsystem, after ros install do this :
//      sudo apt-get install ros-kinetic-rosserial-windows
//     sudo apt-get install ros-kinetic-rosserial-server

// validated on Arduino Mega
// validated on Pilo

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif
#include <Zmotor2.h> 




#if defined(BOARD_ID_Pilo)
#include <Wire.h>
#include <SPI.h>
#include <variant.h>

#include <WireUtility.h>

#endif

Zmotor2 card_motor2 = Zmotor2();

#define MySerial P_COM3.serial2 //usb
#define WireMotor2 (P_COM0_BIS.wire)

void test(int pin1,int pin2)
{
  pinMode(pin1, OUTPUT);
  if(pin2%2==0)
    digitalWrite(pin1, HIGH);   
    else
    digitalWrite(pin1, LOW);   
    
   
  
    if(pin2%2==1)
    digitalWrite(pin2, HIGH);  
    else
    digitalWrite(pin2, LOW);   
    
    
    if(pin2%2==0)
    analogWrite(pin2, 500);  
   else
    analogWrite(pin2, 1500);  
   
   }
void setup() {
  WireMotor2.begin();
  
        
if (0)
{
MySerial.begin(57600);  //115200 //9600
	MySerial.println("Setup");

	volatile int ip = scan(MySerial, WireMotor2);
	while (ip = scanNext(MySerial, WireMotor2) != 0);
}

    //card_motor2.begin(&Wire,0x43);
    card_motor2.begin(&(WireMotor2),0x21,0x41);
    setPinExtender(&card_motor2);
    
    /*
    card_motor2.io.digitalWordWrite(0);
      MySerial.println("MCP23017 DUMP");
    dump(MySerial,WireMotor2, 0x21,0, 0x1A+1);
     delay(100);
   
    
      card_motor2.io.digitalWordWrite(0xffff);
      MySerial.println("MCP23017 DUMP");
    dump(MySerial,WireMotor2, 0x21,0, 0x1A+1);
     delay(100);
   
    
      card_motor2.io.digitalWordWrite(0x5555);
      MySerial.println("MCP23017 DUMP");
    dump(MySerial,WireMotor2, 0x21,0, 0x1A+1);
     delay(100);
   
    
      card_motor2.io.digitalWordWrite(0xaaaa);
      MySerial.println("MCP23017 DUMP");
    dump(MySerial,WireMotor2, 0x21,0, 0x1A+1);
     delay(100);
   
    
    
    
    for(int i=0;i<16;i++)
     if(i%2==0)
    card_motor2.digitalWrite(MOTOR2_IO[i],HIGH);
     else
    card_motor2.digitalWrite(MOTOR2_IO[i],LOW);
    
      MySerial.println("MCP23017 DUMP");
    dump(MySerial,WireMotor2, 0x21,0, 0x1A+1);
     delay(100);
   
     
        for(int i=0;i<16;i++)
     if(i%2==0)
    card_motor2.digitalWrite(MOTOR2_IO[i],HIGH);
     else
    card_motor2.digitalWrite(MOTOR2_IO[i],LOW);
    
      MySerial.println("MCP23017 DUMP");
    dump(MySerial,WireMotor2, 0x21,0, 0x1A+1);
     delay(100);
    
     for (int i=0;i<16;i++)
    if(i%2==0)
    card_motor2.cmd( i,  1500 ) ;
    else
    card_motor2.cmd( i,0 ) ;
    
    
    MySerial.println("PCA9685 DUMP");
    dump(MySerial,WireMotor2, 0x41,0, 0x45+1);
    
      MySerial.println("MCP23017 DUMP");
    dump(MySerial,WireMotor2, 0x21,0, 0x1A+1);
  
    for (int i=0;i<16;i++){
    int pin1=card_motor2.getPin(MOTOR2_IO[i]);
    int pin2=card_motor2.getPin(MOTOR2_PWM[i]);    
   test(pin1,pin2);
   
   }
    delay(100);
   MySerial.println("PCA9685 DUMP");
    dump(MySerial,WireMotor2, 0x41,0, 0x45+1);
    
      MySerial.println("MCP23017 DUMP");
    dump(MySerial,WireMotor2, 0x21,0, 0x1A+1);
   
  delay(1000);
  
   for (int i=0;i<16;i++){
    int pin1=card_motor2.getPin(MOTOR2_IO[i]);
    int pin2=card_motor2.getPin(MOTOR2_PWM[i]);    
    digitalWrite(pin1, LOW);   
     digitalWrite(pin2, LOW);   
   
   }*/
}
int32_t ulValue=0;
void loop()
{
card_motor2.loop();



for(int i=0;i<16;i+=2)
{
  card_motor2.digitalWriteIO( i, LOW ) ;// card_motor2.digitalWriteIO( i+1, HIGH ) ;
  delay(25);
}

for(int i=0;i<16;i+=2)
{
  card_motor2.digitalWritePWM( i, LOW ) ;//card_motor2.digitalWritePWM( i+1, HIGH ) ;
  delay(25);
}

for(int i=0;i<16;i+=2)
{
  card_motor2.digitalWriteIO( i, HIGH ) ;//card_motor2.digitalWriteIO( i+1, LOW ) ;
   delay(25);
}
for(int i=0;i<16;i+=2)
{
  card_motor2.digitalWritePWM( i, HIGH ) ;//card_motor2.digitalWritePWM( i+1, LOW ) ;
  delay(25);
}
for(int i=0;i<16;i+=2)
{
  card_motor2.digitalWriteIO( i, LOW ) ;// card_motor2.digitalWriteIO( i+1, HIGH ) ;
  delay(25);
}

for(int i=0;i<16;i+=2)
{
  card_motor2.digitalWritePWM( i, LOW ) ;//card_motor2.digitalWritePWM( i+1, HIGH ) ;
  delay(25);
}

int value=2048;
for(int i=0;i<16;i+=2)
{
  card_motor2.analogWritePWM( i, value ) ;
  delay(25);
}

for(int i=8;i<16;i+=2)
{
  card_motor2.cmd( i, 0 ) ;
  delay(250);
  card_motor2.cmd( i, 1024 ) ;
  delay(250);
  card_motor2.cmd( i, 4096 ) ;
  delay(250);
  card_motor2.cmd( i, 0 ) ;
  delay(250);  
}
for(int i=8;i<16;i+=2)
{
  card_motor2.cmd( i, 0 ) ;
  delay(250);
  card_motor2.cmd( i, -1024 ) ;
  delay(250);
  card_motor2.cmd( i, -4096 ) ;
  delay(250);
  card_motor2.cmd( i, 0 ) ;
  delay(250);  
}
  delay(1000);
  card_motor2.cmd( 0x0, 0 ) ;
  card_motor2.cmd( 0x2, 0 ) ;
  card_motor2.cmd( 0x4, 0 ) ;card_motor2.cmd( 0x6, 0 ) ;
  delay(1000);
  card_motor2.cmd( 0x0, 2096 ) ;
  card_motor2.cmd( 0x2, 2096 ) ;
  card_motor2.cmd( 0x4, 2096 ) ;card_motor2.cmd( 0x6, 2096 ) ;
  delay(1000);
  card_motor2.cmd( 0x0, -2096 ) ;
  card_motor2.cmd( 0x2, -2096 ) ;
  card_motor2.cmd( 0x4, -2096 ) ;card_motor2.cmd( 0x6, -2096 ) ;
  delay(1000);
  
/*
  for(int ulPin=0;ulPin<16;ulPin+=2)
  card_motor2.cmd( ulPin, ulValue ) ;*/
  int pin2=card_motor2.getPin(MOTOR2_PWM[1]);
   analogWrite(pin2, ulValue);  
  /*
  card_motor2.cmd( 0, ulValue ) ;
  card_motor2.cmd( 1, 0 ) ;
  card_motor2.cmd( 2, ulValue ) ;
  card_motor2.cmd( 3, 0 ) ;
  */
  ulValue+=250;
  
   MySerial.println("PCA9685 DUMP");
    dump(MySerial,WireMotor2, 0x41,0, 0x45+1);
    
      MySerial.println("MCP23017 DUMP");
    dump(MySerial,WireMotor2, 0x21,0, 0x1A+1);
  
  delay(500);
  if (ulValue>4096) ulValue=-4096;
}
