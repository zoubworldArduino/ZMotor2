
/** @file Zmotor2.h
   
   @par dependency :
   this library use the folowing ones :   ZMCP23017, ZPCA9685, PinExtender,Rosserial_Arduino_Library
   you can find it on https://github.com/zoubworldArduino/
   
   @par description
   This lib support a board called Motor2-A based on MCP23017 and PCA9685 with a L9110 as power stage.
   This board offers 16 motor channels 5-12V up to 5/800mA.
   Each channel have 2 pin, one where we can apply LOW or HIGH level thanks to digitalWrite(), and one where we can apply PWM value thanks to analogWrite()
   The pin can be identify by the generic name like #PIN_MOTOR2_IO_0 #PIN_MOTOR2_PWM_0, in this case you have to do instanceBoard.digitalWrite(PIN_MOTOR2_IO_0,LOW)
   The pin can be identify by the instance name like pin=instanceBoard.getpin(#PIN_MOTOR2_IO_0) or pin=instanceBoard.getpinIo(MOTOR2_0) or  or pin=instanceBoard.getpinPwm(MOTOR2_0),
   in this case you have to do digitalWrite(pin,LOW); from arduino API, this offer a compatibility with any library, but before you should link your board to arduino API by calling setPinExtender(&instanceBoard);
   if you have 2 board, then do it :instanceBoard.setPinExtender(&instanceBoard_2);
   
   generic name is used inside the instance of board, and the instance pin name allow to use generic arduino IPA, it content on it the I2C addresse of the device and the channel.
   note that instanceBoard.getpin() must be called after instanceBoard creation and initialisation with begin(), the pin number is a 32bit number.
   
   The cmd(channel, pwm) function allow to manage easly the motor control, channel is between  0 and 15 like on skillprint of the board.
   The pwm value is between -4096 and 4095, 0 give no power.
   
   The name #PIN_MOTOR2_IO_0 can be replace by MOTOR2_IO[0].
   The name #PIN_MOTOR2_PWM_0 can be replace by MOTOR2_PWM[0].
   
   Release A of board:
   - a bug on the board limit the channel to the even only, so only 8 channel
   - note the True table of L9110 isn't as writen in datasheet :
   |   Input A      |    input B        |  output A    |   output B   |
   |----------------|-------------------|--------------|--------------|
   |      0         |       0           |   LOW        |    LOW       |
   |      0         |       1           |   LOW        |    HIGH      |
   |      1         |       0           |   HIGH       |    LOW       |
   |      1         |       1           |   LOW        |    LOW       |
   |                |                   |              |              |
   but 
   |   Input A      |    input B        |  output A    |   output B   |
   |----------------|-------------------|--------------|--------------|
   |      0         |       0           |HIGH IMPEDENCE|HIGH INPEDENCE|
   |      0         |       1           |   LOW        |    HIGH      |
   |      1         |       0           |   HIGH       |    LOW       |
   |      1         |       1           |HIGH IMPEDENCE|HIGH IMPEDENCE|
   |                |                   |              |              |
   Release B of board:
   the bug is fix but the library must be differents.
   */
#ifndef _ZMOTOR_H
#define _ZMOTOR_H
#define ROS_USED

#ifdef ROS_USED 
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int16.h>
#endif 


#include "PinExtender.h"
#include <ZMCP23017.h>
#include <ZPCA9685.h>
    
#define MOTOR2_0 0
#define MOTOR2_1 1
#define MOTOR2_2 2
#define MOTOR2_3 3
#define MOTOR2_4 4
#define MOTOR2_5 5
#define MOTOR2_6 6
#define MOTOR2_7 7
#define MOTOR2_8 8
#define MOTOR2_9 9
#define MOTOR2_10 10
#define MOTOR2_11 11
#define MOTOR2_12 12
#define MOTOR2_13 13
#define MOTOR2_14 14
#define MOTOR2_15 15
#define MOTOR2_A MOTOR2_10
#define MOTOR2_B MOTOR2_11
#define MOTOR2_C MOTOR2_12
#define MOTOR2_D MOTOR2_13
#define MOTOR2_E MOTOR2_14
#define MOTOR2_F MOTOR2_15

#define MCP23017_ADDR_BASE 0x20
#define PCA9685_ADDR_BASE 0x40
   

#define PIN_MOTOR2_IO_0  (MCP23017_GPA7 | MCP23017_ADDR_BASE<<16)
#define PIN_MOTOR2_IO_1  (MCP23017_GPA6 | MCP23017_ADDR_BASE<<16)
#define PIN_MOTOR2_IO_2  (MCP23017_GPA5 | MCP23017_ADDR_BASE<<16)
#define PIN_MOTOR2_IO_3  (MCP23017_GPA4 | MCP23017_ADDR_BASE<<16)
#define PIN_MOTOR2_IO_4  (MCP23017_GPA3 | MCP23017_ADDR_BASE<<16)
#define PIN_MOTOR2_IO_5  (MCP23017_GPA2 | MCP23017_ADDR_BASE<<16)
#define PIN_MOTOR2_IO_6  (MCP23017_GPA1 | MCP23017_ADDR_BASE<<16)
#define PIN_MOTOR2_IO_7  (MCP23017_GPA0 | MCP23017_ADDR_BASE<<16)

#define PIN_MOTOR2_IO_8  (MCP23017_GPB7 | MCP23017_ADDR_BASE<<16)
#define PIN_MOTOR2_IO_9  (MCP23017_GPB6 | MCP23017_ADDR_BASE<<16)
#define PIN_MOTOR2_IO_10 (MCP23017_GPB5 | MCP23017_ADDR_BASE<<16)
#define PIN_MOTOR2_IO_11 (MCP23017_GPB4 | MCP23017_ADDR_BASE<<16)
#define PIN_MOTOR2_IO_12 (MCP23017_GPB3 | MCP23017_ADDR_BASE<<16)
#define PIN_MOTOR2_IO_13 (MCP23017_GPB2 | MCP23017_ADDR_BASE<<16)
#define PIN_MOTOR2_IO_14 (MCP23017_GPB1 | MCP23017_ADDR_BASE<<16)
#define PIN_MOTOR2_IO_15 (MCP23017_GPB0 | MCP23017_ADDR_BASE<<16)

#define PIN_MOTOR2_IO_A    PIN_MOTOR2_IO_10
#define PIN_MOTOR2_IO_B    PIN_MOTOR2_IO_11
#define PIN_MOTOR2_IO_C    PIN_MOTOR2_IO_12
#define PIN_MOTOR2_IO_D    PIN_MOTOR2_IO_13
#define PIN_MOTOR2_IO_E    PIN_MOTOR2_IO_14
#define PIN_MOTOR2_IO_F    PIN_MOTOR2_IO_15

#define PIN_MOTOR2_PWM_0  (PCA9685_LED15 | PCA9685_ADDR_BASE<<16)
#define PIN_MOTOR2_PWM_1  (PCA9685_LED14 | PCA9685_ADDR_BASE<<16)
#define PIN_MOTOR2_PWM_2  (PCA9685_LED13 | PCA9685_ADDR_BASE<<16)
#define PIN_MOTOR2_PWM_3  (PCA9685_LED12 | PCA9685_ADDR_BASE<<16)
#define PIN_MOTOR2_PWM_4  (PCA9685_LED11 | PCA9685_ADDR_BASE<<16)
#define PIN_MOTOR2_PWM_5  (PCA9685_LED10 | PCA9685_ADDR_BASE<<16)
#define PIN_MOTOR2_PWM_6  (PCA9685_LED9  | PCA9685_ADDR_BASE<<16)
#define PIN_MOTOR2_PWM_7  (PCA9685_LED8  | PCA9685_ADDR_BASE<<16)

#define PIN_MOTOR2_PWM_8  (PCA9685_LED7  | PCA9685_ADDR_BASE<<16)
#define PIN_MOTOR2_PWM_9  (PCA9685_LED6  | PCA9685_ADDR_BASE<<16)
#define PIN_MOTOR2_PWM_10 (PCA9685_LED5  | PCA9685_ADDR_BASE<<16)
#define PIN_MOTOR2_PWM_11 (PCA9685_LED4  | PCA9685_ADDR_BASE<<16)
#define PIN_MOTOR2_PWM_12 (PCA9685_LED3  | PCA9685_ADDR_BASE<<16)
#define PIN_MOTOR2_PWM_13 (PCA9685_LED2  | PCA9685_ADDR_BASE<<16)
#define PIN_MOTOR2_PWM_14 (PCA9685_LED1  | PCA9685_ADDR_BASE<<16)
#define PIN_MOTOR2_PWM_15 (PCA9685_LED0  | PCA9685_ADDR_BASE<<16)

#define PIN_MOTOR2_PWM_A  PIN_MOTOR2_PWM_10
#define PIN_MOTOR2_PWM_B  PIN_MOTOR2_PWM_11
#define PIN_MOTOR2_PWM_C  PIN_MOTOR2_PWM_12
#define PIN_MOTOR2_PWM_D  PIN_MOTOR2_PWM_13
#define PIN_MOTOR2_PWM_E  PIN_MOTOR2_PWM_14
#define PIN_MOTOR2_PWM_F  PIN_MOTOR2_PWM_15
   

extern uint32_t MOTOR2_PWM[];
extern uint32_t MOTOR2_IO[];


/**************************************************************************/
/*! 
    @brief  Class that stores state and functions for interacting with ZPCA9685 PWM chip
*/
/**************************************************************************/
class Zmotor2 : public PinExtender  {
 public:
 Zmotor2();
  void begin(TwoWire *MyWire,uint8_t addr1);
   void begin(TwoWire *i2c,uint8_t addrio,uint8_t addrpwm);
  void begin(uint8_t addr,uint8_t addr2);
  void begin(uint8_t addr);
  void begin(void);
  void SWRST (void);
bool check();

  void pinMode(uint32_t p//!< the pin requested, it is the instance number or the generic number .
               , uint8_t d);
  void digitalWrite(uint32_t p//!< the pin requested, it is the instance number or the generic number .
                    , uint8_t d);
  uint8_t digitalRead(uint32_t p//!< the pin requested, it is the instance number or the generic number .
                      );
  /*
 * \brief Writes an analog value (PWM wave) to a pin.
 *
 * \param ulPin
 * \param ulValue
 */
 void analogWrite( uint32_t ulPin //!< the pin requested, it is the instance number or the generic number .
                  , uint32_t ulValue ) ;

  void analogWriteResolution(int res);

 
  /** return the pin number of current instance from the generioc name */
 uint32_t getPin(uint32_t ulchannel//!< the generic pin number requested like #PIN_MOTOR2_IO_0 or #PIN_MOTOR2_PWM_0
                 );
 
  /** return the IO pin number of current instance */
 uint32_t getPinIo(uint32_t ulchannel//!< the channel requested between 0 to 15
                 );
 /** return the PWM pin number of current instance */
 uint32_t getPinPwm(uint32_t ulchannel//!< the channel requested between 0 to 15
                 );
 void reset(void);
 
  uint32_t analogRead( uint32_t pin //!< the pin requested, it is the instance number or the generic number .
                      );
  /** set the PWM freqeuncy.
  */
  void setPWMFreq(float freq);
  /** apply a PWM command on motor connected on IO[channel] and PWM[channel]
  */
  void cmd( uint32_t channel//!< the channel requested between 0 to 15 like  #MOTOR2_0
           , 
           int32_t ulValue //!< the PWM value requested between -4096 to 4095
           ) ;
  bool test();
 #ifdef ROS_USED 
    void setup( ros::NodeHandle * myNodeHandle,	const char   *	topic ,void callbackinstance( const std_msgs::Int16& cmd_msg),int pin);
	void setup( ros::NodeHandle * myNodeHandle,	const char   *	topic ,int pin);
	void loop();
#endif 
 ZPCA9685  pwm;
 ZMCP23017  io;
  protected:
       bool acceptlocal(uint32_t p);
 private: 
 
  #ifdef ROS_USED 
    ros::NodeHandle  *nh;    
    ros::Subscriber<std_msgs::Int16> *subscriber[16];
#endif

 
};


#endif
