/*************************************************** 
  This is a library for our Adafruit 16-channel PWM & Servo driver

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These displays use I2C to communicate, 2 pins are required to  
  interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#ifndef _ZMOTOR_H
#define _ZMOTOR_H


#include "PinExtender.h"
#include <ZMCP23017.h>
#include <ZPCA9685.h>
    

#define MCP23017_ADDR_BASE 0x20
#define PCA9685_ADDR_BASE 0x40
#define MOTOR2_IO_0  (MCP23017_GPA7 | MCP23017_ADDR_BASE<<16)
#define MOTOR2_IO_1  (MCP23017_GPA6 | MCP23017_ADDR_BASE<<16)
#define MOTOR2_IO_2  (MCP23017_GPA5 | MCP23017_ADDR_BASE<<16)
#define MOTOR2_IO_3  (MCP23017_GPA4 | MCP23017_ADDR_BASE<<16)
#define MOTOR2_IO_4  (MCP23017_GPA3 | MCP23017_ADDR_BASE<<16)
#define MOTOR2_IO_5  (MCP23017_GPA2 | MCP23017_ADDR_BASE<<16)
#define MOTOR2_IO_6  (MCP23017_GPA1 | MCP23017_ADDR_BASE<<16)
#define MOTOR2_IO_7  (MCP23017_GPA0 | MCP23017_ADDR_BASE<<16)

#define MOTOR2_IO_8  (MCP23017_GPB7 | MCP23017_ADDR_BASE<<16)
#define MOTOR2_IO_9  (MCP23017_GPB6 | MCP23017_ADDR_BASE<<16)
#define MOTOR2_IO_10 (MCP23017_GPB5 | MCP23017_ADDR_BASE<<16)
#define MOTOR2_IO_11 (MCP23017_GPB4 | MCP23017_ADDR_BASE<<16)
#define MOTOR2_IO_12 (MCP23017_GPB3 | MCP23017_ADDR_BASE<<16)
#define MOTOR2_IO_13 (MCP23017_GPB2 | MCP23017_ADDR_BASE<<16)
#define MOTOR2_IO_14 (MCP23017_GPB1 | MCP23017_ADDR_BASE<<16)
#define MOTOR2_IO_15 (MCP23017_GPB0 | MCP23017_ADDR_BASE<<16)

#define MOTOR2_IO_A    MOTOR2_IO_10
#define MOTOR2_IO_B    MOTOR2_IO_11
#define MOTOR2_IO_C    MOTOR2_IO_12
#define MOTOR2_IO_D    MOTOR2_IO_13
#define MOTOR2_IO_E    MOTOR2_IO_14
#define MOTOR2_IO_F    MOTOR2_IO_15

#define MOTOR2_PWM_0  (PCA9685_LED15 | PCA9685_ADDR_BASE<<16)
#define MOTOR2_PWM_1  (PCA9685_LED14 | PCA9685_ADDR_BASE<<16)
#define MOTOR2_PWM_2  (PCA9685_LED13 | PCA9685_ADDR_BASE<<16)
#define MOTOR2_PWM_3  (PCA9685_LED12 | PCA9685_ADDR_BASE<<16)
#define MOTOR2_PWM_4  (PCA9685_LED11 | PCA9685_ADDR_BASE<<16)
#define MOTOR2_PWM_5  (PCA9685_LED10 | PCA9685_ADDR_BASE<<16)
#define MOTOR2_PWM_6  (PCA9685_LED9  | PCA9685_ADDR_BASE<<16)
#define MOTOR2_PWM_7  (PCA9685_LED8  | PCA9685_ADDR_BASE<<16)

#define MOTOR2_PWM_8  (PCA9685_LED7  | PCA9685_ADDR_BASE<<16)
#define MOTOR2_PWM_9  (PCA9685_LED6  | PCA9685_ADDR_BASE<<16)
#define MOTOR2_PWM_10 (PCA9685_LED5  | PCA9685_ADDR_BASE<<16)
#define MOTOR2_PWM_11 (PCA9685_LED4  | PCA9685_ADDR_BASE<<16)
#define MOTOR2_PWM_12 (PCA9685_LED3  | PCA9685_ADDR_BASE<<16)
#define MOTOR2_PWM_13 (PCA9685_LED2  | PCA9685_ADDR_BASE<<16)
#define MOTOR2_PWM_14 (PCA9685_LED1  | PCA9685_ADDR_BASE<<16)
#define MOTOR2_PWM_15 (PCA9685_LED0  | PCA9685_ADDR_BASE<<16)

#define MOTOR2_PWM_A  MOTOR2_PWM_10
#define MOTOR2_PWM_B  MOTOR2_PWM_11
#define MOTOR2_PWM_C  MOTOR2_PWM_12
#define MOTOR2_PWM_D  MOTOR2_PWM_13
#define MOTOR2_PWM_E  MOTOR2_PWM_14
#define MOTOR2_PWM_F  MOTOR2_PWM_15




/**************************************************************************/
/*! 
    @brief  Class that stores state and functions for interacting with ZPCA9685 PWM chip
*/
/**************************************************************************/
class Zmotor2 : public PinExtender  {
 public:
 Zmotor2();
  void begin(TwoWire *MyWire,uint8_t addr1);
   void Zmotor2::begin(TwoWire *i2c,uint8_t addrio,uint8_t addrpwm);
  void begin(uint8_t addr,uint8_t addr2);
  void begin(uint8_t addr);
  void begin(void);
  void SWRST (void);
bool check();

  void pinMode(uint32_t p, uint8_t d);
  void digitalWrite(uint32_t p, uint8_t d);
  uint8_t digitalRead(uint32_t p);
  /*
 * \brief Writes an analog value (PWM wave) to a pin.
 *
 * \param ulPin
 * \param ulValue
 */
 void analogWrite( uint32_t ulPin, uint32_t ulValue ) ;

  void analogWriteResolution(int res);

  
 uint32_t Zmotor2::getPin(uint32_t ulPin);
  void reset(void);
 
  uint32_t analogRead( uint32_t pin );

  void setPWMFreq(float freq);
  protected:
       bool acceptlocal(uint32_t p);
 private: 
 ZPCA9685  pwm;
 ZMCP23017  io;
 
  
};


#endif
