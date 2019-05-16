/** @file Zmotor2.cpp



*/
#include <assert.h>

#include "zmotor2.h"

#include <Wire.h>

#include "PinExtender.h"
#define DEBUG(a) a
//#define DEBUG(a) {}

uint32_t MOTOR2_PWM[]={PIN_MOTOR2_PWM_0,PIN_MOTOR2_PWM_1,PIN_MOTOR2_PWM_2,PIN_MOTOR2_PWM_3,PIN_MOTOR2_PWM_4,PIN_MOTOR2_PWM_5,PIN_MOTOR2_PWM_6,PIN_MOTOR2_PWM_7,PIN_MOTOR2_PWM_8,PIN_MOTOR2_PWM_9,PIN_MOTOR2_PWM_10,PIN_MOTOR2_PWM_11,PIN_MOTOR2_PWM_12,PIN_MOTOR2_PWM_13,PIN_MOTOR2_PWM_14,PIN_MOTOR2_PWM_15};
uint32_t MOTOR2_IO[]={PIN_MOTOR2_IO_0,PIN_MOTOR2_IO_1,PIN_MOTOR2_IO_2,PIN_MOTOR2_IO_3,PIN_MOTOR2_IO_4,PIN_MOTOR2_IO_5,PIN_MOTOR2_IO_6,PIN_MOTOR2_IO_7,PIN_MOTOR2_IO_8,PIN_MOTOR2_IO_9,PIN_MOTOR2_IO_10,PIN_MOTOR2_IO_11,PIN_MOTOR2_IO_12,PIN_MOTOR2_IO_13,PIN_MOTOR2_IO_14,PIN_MOTOR2_IO_15};

/**************************************************************************/
/*! 
    @brief  Instantiates a new Zmotor2 PWM driver chip with the I2C address on the Wire interface. On Due we use Wire1 since its the interface on the 'default' I2C pins.
    @param  addr The 7-bit I2C address to locate this chip, default is 0x40
*/
/**************************************************************************/
Zmotor2::Zmotor2() : PinExtender(),  io(),  pwm() 
 {
	 
}
/**************************************************************************/
/*! 
    @brief  Setups the I2C interface and hardware
*/
/**************************************************************************/

  void Zmotor2::begin(TwoWire *i2c,uint8_t addr)
  {
	  begin(i2c, addr|MCP23017_ADDR_BASE, addr|PCA9685_ADDR_BASE);
  }
 void Zmotor2::begin(TwoWire *i2c,uint8_t addrio,uint8_t addrpwm)
  {
    i2c->begin();
	  io.begin(i2c, addrio);
      pwm.begin(i2c, addrpwm);
	 pwm.analogWriteResolution( 12);
	 pwm.setPWMFreq(1526);
    for (int i=0;i<16;i++)
     io.pinMode(io.getPin(i), OUTPUT);
    for (int i=0;i<16;i++)
     pwm.pinMode(pwm.getPin(i), OUTPUT);
      
	 
}


  bool Zmotor2::test()
  {
    bool b=pwm.test();
    b&=io.test();
  return b;
  }
/**************************************************************************/
/*! 
    @brief  Setups the I2C interface and hardware
*/
/**************************************************************************/

  void Zmotor2::begin(uint8_t addr)
  {
    begin(&Wire, addr);
}
/**************************************************************************/
/*! 
    @brief  Setups the I2C interface and hardware
*/
/**************************************************************************/

  void Zmotor2::begin(void)
  {
	  begin( 0x00);
  }
  



/**************************************************************************/
/*! 
    @brief  Sends a reset command to the Zmotor2 chip over I2C
*/
/**************************************************************************/
void Zmotor2::reset(void) {
 // io.reset();
  pwm.reset();
  
}
/**************************************************************************/
/*! 
    @brief  Sets the PWM frequency for the entire chip, up to ~1.6 KHz
    @param  freq Floating point frequency that we will attempt to match
*/
/**************************************************************************/
void Zmotor2::setPWMFreq(float freq) {
	pwm.setPWMFreq( freq);
}


bool Zmotor2::acceptlocal(uint32_t p)
{
  if (pwm.acceptlocal( p))
  return true;
  if (io.acceptlocal( p))
  return true; 
  return false;
  
}

     
/** dummy function
*/
uint8_t Zmotor2::digitalRead(uint32_t ulPin)
{
	 if (pwm.acceptlocal( ulPin))
  return pwm.digitalRead(ulPin);
else
  if (io.acceptlocal( ulPin))
  return  io.digitalRead(ulPin);
	else if (_next)
		return _next->digitalRead( ulPin);	
return 0;	
}
/**
 * Sets the pin mode to either INPUT or OUTPUT but for all, and input doesn't exist
 */
void Zmotor2::pinMode(uint32_t ulPin, uint8_t mode) {
	
		 if (pwm.acceptlocal( ulPin))
   pwm.pinMode(ulPin,mode);
else
  if (io.acceptlocal( ulPin))
    io.pinMode(ulPin,mode);
	else if (_next)
		 _next->pinMode( ulPin,mode);	
	
	
}
void Zmotor2::analogWriteResolution(int res)
{
	pwm.analogWriteResolution(res);
	io.analogWriteResolution(res);
  if (_next)
		return _next->analogWriteResolution( res);		
}
 void Zmotor2::analogWrite( uint32_t ulPin, uint32_t ulValue ) 
 {
	 if (pwm.acceptlocal( ulPin))
   pwm.analogWrite(ulPin,ulValue);
else
  if (io.acceptlocal( ulPin))
    io.analogWrite(ulPin,ulValue);
	else if (_next)
		 _next->analogWrite( ulPin,ulValue);		
	
 }
void Zmotor2::cmd( uint32_t ulchannel, int32_t siValue ) 
{
  ulchannel&=0xF;
	
        
	if(siValue>=0)
	{
          analogWritePWM(ulchannel,siValue);
	  digitalWriteIO( ulchannel,  LOW);
         /*
   #ifdef ISBUGL9110
    if (ulchannel%2==0)
      ulchannel=ulchannel+1;
    else
      ulchannel=ulchannel-1;
    int ulPinpwm1=pwm.getPin(MOTOR2_PWM[ulchannel]);
    pwm.digitalWrite(ulPinpwm1,LOW);
  #endif     
	*/
	
	}
	else
	{
          siValue=4096+ siValue;
	  if(siValue<0)
	  siValue=0;
          analogWritePWM(ulchannel,siValue);
	  digitalWriteIO( ulchannel,  HIGH);
          /*
           #ifdef ISBUGL9110
    if (ulchannel%2==0)
      ulchannel=ulchannel+1;
    else
      ulchannel=ulchannel-1;
    int ulPinpwm1=pwm.getPin(MOTOR2_PWM[ulchannel]);
    pwm.digitalWrite(ulPinpwm1,HIGH);
  #endif     
	*/
	
	}
	
}

 uint32_t Zmotor2::getPin(uint32_t ulPin)
 {
	 if ((ulPin & (PCA9685_ADDR_BASE<<16))==(PCA9685_ADDR_BASE<<16))
	 return pwm.getPin( pwm.getPin(ulPin));
 	 if ((ulPin & (MCP23017_ADDR_BASE<<16))==(MCP23017_ADDR_BASE<<16))
	 return io.getPin( io.getPin(ulPin)); 
     return NO_CHANNEL;
 }
 /*
 uint32_t Zmotor2::pin2channel(uint32_t ulPin)
 {
	 if (pwm.acceptlocal( ulPin))
	 return ulPin &0xff | PCA9685_ADDR_BASE<<16;
 	 if (io.acceptlocal( ulPin))
	 return ulPin &0xff | MCP23017_ADDR_BASE<<16;
 
 
 return NO_CHANNEL;
 }*/
 uint32_t Zmotor2::analogRead( uint32_t pin )
{ 
  return 0;
}

void Zmotor2::digitalWritePWM(uint32_t ulchannel, uint8_t ulValue)
{
    ulchannel&=0xF;
	uint32_t ulPinpwm=pwm.getPin(MOTOR2_PWM[ulchannel]);//MOTOR2_0
pwm.digitalWrite(ulPinpwm,ulValue);
  #ifdef ISBUGL9110
    if (ulchannel%2==0)
      ulchannel=ulchannel+1;
    else
      ulchannel=ulchannel-1;
    uint32_t ulPinpwm1=pwm.getPin(MOTOR2_PWM[ulchannel]);//MOTOR2_0
    pwm.digitalWrite(ulPinpwm1,ulValue==LOW?HIGH:LOW);
  #endif    
}
 void Zmotor2::analogWritePWM( uint32_t ulchannel, uint32_t ulValue, bool inverted ) 
 {   
    ulchannel&=0xF;
    uint32_t ulPinpwm=pwm.getPin(MOTOR2_PWM[ulchannel]);//MOTOR2_0
   pwm.analogWrite(ulPinpwm,ulValue,inverted);
   
   #ifdef ISBUGL9110
    if (ulchannel%2==0)
      ulchannel=ulchannel+1;
    else
      ulchannel=ulchannel-1;
    int ulPinpwm1=pwm.getPin(MOTOR2_PWM[ulchannel]);
   // pwm.digitalWrite( ulPinpwm1, LOW ) ;
    pwm.analogWrite(ulPinpwm1,ulValue,!inverted);
    /*
    uint32_t ulPinpwm1=pwm.getPin(MOTOR2_PWM[ulchannel]);//MOTOR2_0
    pwm.analogWrite(ulPinpwm1,ulValue,true);*/
  #endif     
 }
void Zmotor2::digitalWriteIO(uint32_t ulchannel, uint8_t ulValue)
{
    ulchannel&=0xF;
    uint32_t ulPinio=io.getPin(MOTOR2_IO[ulchannel]);//MOTOR2_0
    io.digitalWrite(ulPinio,ulValue);

  #ifdef ISBUGL9110
    if (ulchannel%2==0)
      ulchannel=ulchannel+1;
    else
      ulchannel=ulchannel-1;
    uint32_t ulPinio1=io.getPin(MOTOR2_IO[ulchannel]);//MOTOR2_0
    io.digitalWrite(ulPinio1,ulValue==LOW?HIGH:LOW);
  #endif
}

void Zmotor2::digitalWrite(uint32_t ulPin, uint8_t ulValue)
{
	if (pwm.acceptlocal( ulPin))
   pwm.digitalWrite(ulPin,ulValue);
else
  if (io.acceptlocal( ulPin))
    io.digitalWrite(ulPin,ulValue);
	else if (_next)
		 _next->digitalWrite( ulPin,ulValue);		
}




#ifdef ROS_USED 

Zmotor2 * myZmotor2;
static void callbackinstance0( const std_msgs::Int16& cmd_msg)
{	
myZmotor2->cmd(0,cmd_msg.data); 
}
static void callbackinstance1( const std_msgs::Int16& cmd_msg)
{	
myZmotor2->cmd(1,cmd_msg.data); 
}
static void callbackinstance2( const std_msgs::Int16& cmd_msg)
{	
myZmotor2->cmd(2,cmd_msg.data); 
}
static void callbackinstance3( const std_msgs::Int16& cmd_msg)
{	
myZmotor2->cmd(3,cmd_msg.data); 
}

static void callbackinstance4( const std_msgs::Int16& cmd_msg)
{	
myZmotor2->cmd(4,cmd_msg.data); 
}
static void callbackinstance5( const std_msgs::Int16& cmd_msg)
{	
myZmotor2->cmd(5,cmd_msg.data); 
}
static void callbackinstance6( const std_msgs::Int16& cmd_msg)
{	
myZmotor2->cmd(6,cmd_msg.data); 
}
static void callbackinstance7( const std_msgs::Int16& cmd_msg)
{	
myZmotor2->cmd(7,cmd_msg.data); 
}
static void callbackinstance8( const std_msgs::Int16& cmd_msg)
{	
myZmotor2->cmd(8,cmd_msg.data); 
}
static void callbackinstance9( const std_msgs::Int16& cmd_msg)
{	
myZmotor2->cmd(9,cmd_msg.data); 
}
static void callbackinstance10( const std_msgs::Int16& cmd_msg)
{	
myZmotor2->cmd(10,cmd_msg.data); 
}
static void callbackinstance11( const std_msgs::Int16& cmd_msg)
{	
myZmotor2->cmd(11,cmd_msg.data); 
}
static void callbackinstance12( const std_msgs::Int16& cmd_msg)
{	
myZmotor2->cmd(12,cmd_msg.data); 
}
static void callbackinstance13( const std_msgs::Int16& cmd_msg)
{	
myZmotor2->cmd(13,cmd_msg.data); 
}
static void callbackinstance14( const std_msgs::Int16& cmd_msg)
{	
myZmotor2->cmd(14,cmd_msg.data); 
}
static void callbackinstance15( const std_msgs::Int16& cmd_msg)
{	
myZmotor2->cmd(15,cmd_msg.data);    
}

static void(*callback[16])(const std_msgs::Int16& cmd_msg)={
	callbackinstance0,callbackinstance1,callbackinstance2,callbackinstance3,
	callbackinstance4,callbackinstance5,callbackinstance6,callbackinstance7,
	callbackinstance8,callbackinstance9,callbackinstance10,callbackinstance11,
	callbackinstance12,callbackinstance13,callbackinstance14,callbackinstance15	
	};

void Zmotor2::setup( ros::NodeHandle * myNodeHandle,	const char   *	topic ,int pin)
{
setup( myNodeHandle,		topic ,callback[pin], pin);
}
static  int index=0;
// ROS SECTION :
//char frameid[] = "/ir_ranger";
/** setup :
  At setup after NodeHandle setup, call this to initialise the topic
*/
void Zmotor2::setup( ros::NodeHandle * myNodeHandle,	const char   *	topic ,void callbackinstance( const std_msgs::Int16& cmd_msg),int pin)
{
    assert(index<16);
	myZmotor2= this;
	pin=pin&0xf;
  nh=myNodeHandle;
  subscriber[pin]=new ros::Subscriber<std_msgs::Int16> (topic, callbackinstance); 
  assert(subscriber[pin]!=0);
  nh->subscribe(*subscriber[pin]); 
  DEBUG(nh->loginfo("Zmotor2::setup()")); 
  DEBUG(nh->loginfo(topic)); 
  index++;
}
/** loop :
  on loop  before NodeHandle refresh(spinOnce), call this to update the topic
*/
void Zmotor2::loop()
{
	//nothing to do just keep for compatibility with others.
}
#endif 