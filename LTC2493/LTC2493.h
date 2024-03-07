/*************************************************************************
orignal  but most is not used 

Title:    LTC2493 / LTC2499 library
Authors:  Nathan D. Holmes <maverick@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2013 Nathan D. Holmes & Michael D. Petersen

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/


/** LTC2493 class.
 *  
 *  Used for interfacing with a LTC2493 24  Bit ADC  4 channel single ended 
 *  It has to be used with the  https://developer.mbed.org/users/wbeaumont/code/DevInterfaces/ package 
 *  It is based on https://github.com/avandalen/LTC2499-LTC2493-24bit-ADC/tree/master 
 *  but only for the LTC2493 
 *  This includes the "virtual" I2CInterface class that is the interface to the I2C device 
 *  An implementation of the I2Cinterface class for the MBED can be found  or Raspbery PI 
 *  For example / test program see https://github.com/wimbeaumont/peripheral_dev_tst
 *  There is no check for the bus frequency ( spec < 400kHZ) 
 *  V 0.10  inital version
 *  V 0.20  compiles 
 *  V 0.40  added getVersion
 * (C) Wim Beaumont Universiteit Antwerpen 2024
 *  
 */
 
 /*  implemented at version 0.40  so what is not mentioned here is not implemented or not checked 
  *  readADC always set channel and restart 
  *  getVoltage  to be checked for the correct conversion 
 */
 

#ifndef LTC2493_H
#define LTC2493_H

#include "getVersion.h"

#include "stdbool.h"

#include "dev_interface_def.h"
#include "I2CInterface.h" 
#include "getVersion.h"
//#include "ADCInterface.h" 

#define VERSION_LTC2493_HDR "0.40"


// The temperature channel is actually fake - we manipulate things internally
#define CHAN_TEMPERATURE   (_BV(FAKE_CONFIG1_TEMPERATURE))
//                      0b10ESO21OEIABS000 
#define CONFIG_ZB	0b1011100111111000   // 0 bit maks , the bits with 0 are always 0
#define CONFIG_EN	0b0010000000000000
#define CONFIG_SGL	0b0001000000000000
#define CONFIG_ODD	0b0000100000000000
#define CONFIG_A0	0b0000000100000000
#define CONFIG_EN2	0b0000000010000000
#define CONFIG_IM       0b0000000001000000
#define CONFIG_FA       0b0000000000100000
#define CONFIG_FB	0b0000000000010000
#define CONFIG_SPD	0b0000000000001000   

#define CONFIG_DIFF_START 0b1011000010000000


class LTC2493 :public virtual getVersion {

    public:
		
	LTC2493(I2CInterface* i2cinterface, int i2c_address,  float referenceVolts=0, unsigned int configurationinit=0b1010000010000000);
	// set the channel config bits in current config 
	// @channel  : the channel nr. no checks are done  
	// @startcinv: start the conversion after channel change  if false the EN bit will be set to 0 when writing the config register
	// @differntial , if -1 no change , if 0 set to single ended , if 1 set to differential, if 2 set to inverse 
	void   set_channelconfig( int ch , int diff  );
	// set the enable bit 
	// @enable set the enable bit  if true ,clear if false 
	// @start  set also en2 bit , 
	void setEnable( bool enable ,bool enable2 ) ;
	
	// set the ch and reads the 24 bits ADC register
	//@value , will be filled with the read result
	//@return the i2 read result 0 if no error 
	int  getRawADCvalue(unsigned int& value, int ch);
	
	
	// convert  the ADC register value to an int value 
	// @adcvalue  the bit pattern from a read 
	// @rangeerror will be set if out of range is detected value will retrun |max + 1lsb|
	// @return  int value of the ADC 
	int getADCINTvalue(unsigned int adcvalue, int& rangeerror );
	
	// returns the ADC int value to voltage 
	// @value the 24 bits ADC value + sign
	// @return the corresponding voltage 
	float  convert2Voltage(int value);
	
	//gives the ADC value , sets (always) the channel and starts a conversion
	//@value the ADC value read 
	//@ch the channel to be read
	//@new_value , start a new conversion ,will always be done 
	//@return status 
	int getADCvalue (int& value, int ch=0 , bool new_value=true );
	
	// gives the ADC read value in volt 
	//@value the ADC value read 
	//@ch the channel to be read
	//@new_value , start a new conversion ,will always be done 
	//@return status 
	int getVoltage (float &volt , int ch=0);
	// this function just reads the ADC register ( so also restart a new conversion)(of the CCh) 
	// fresh is not yet implemented 
	// so value has to be interpreted and converted to voltage 
	int getADCvalueCCh(unsigned int& value, bool fresh ) ; 
	
	int get_status(void) { return last_status; }
	unsigned int get_config(void) {return currentconfig;}
	void set_config(unsigned int newconfig) { currentconfig=newconfig ;}
    protected:
	/** pointer to the I2C interface driver. */
	I2CInterface* _i2c_interface;
	/** The full i2c device address. */
	int _device_address;
	float refVolt;
	
    private:
    
	    //@value returns the read value 
	    //@nr_bytes  nr of bytes to be read 
	    //@return  status of I2C read or other error. 
	    int readI2c(unsigned int& value ,  int nr_bytes );
	    int writeI2C( unsigned  int  dataword ); 
	    int last_status;
	    
	    unsigned int currentconfig;
    
};

#endif 
