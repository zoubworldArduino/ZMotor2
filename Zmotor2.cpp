/*************************************************** 
  This is a library for our Adafruit 16-channel PWM & Servo driver

  Pick one up today in the adafruit shop!
  -----. http://www.adafruit.com/products/815

  These displays use I2C to communicate, 2 pins are required to  
  interface.

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "zmotor2.h"

#include <Wire.h>

#include "PinExtender.h"

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
	  io.begin(i2c, addrio);
      pwm.begin(i2c, addrpwm);
    for (int i=0;i<16;i++)
     io.pinMode(io.getPin(i), OUTPUT);
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


 uint32_t Zmotor2::getPin(uint32_t ulPin)
 {
	 if ((ulPin & (PCA9685_ADDR_BASE<<16))==(PCA9685_ADDR_BASE<<16))
	 return pwm.getPin( ulPin);
 	 if ((ulPin & (MCP23017_ADDR_BASE<<16))==(MCP23017_ADDR_BASE<<16))
	 return io.getPin( ulPin); 
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