#ifndef __I2CINTERFACE__
#define __I2CINTERFACE__

#include "getVersion.h"
#include "DevErrorReporter.h"
#define I2CINTERFACE_HDR_VER "2.11"


/*
 *  This is a the I2CInterface "virtual"  class used in all I2C devices 
 *  This file make part of the PeriperalDevice package see repository  
 *  https://github.com/wimbeaumont/PeripheralDevices
 *  For more info see 	the README.md in the top of repository 
 *
 *  ver  0:40  added wait_for_ms 
 *  ver  1:00  changed the interface , all functions will have a retrun value was not correct 
 *  ver  1.10  added lock and unlock, abort_transfer added more coments
 *  ver  2.00  added error status info 
 *  ver  2.10  added read_reg as this is often used. 
 *  ver  2.11   added  wait_for_us(int x) 
 * (C) Wim Beaumont Universiteit Antwerpen 2016,2017,2018,2019
 *
 * License see
 * https://github.com/wimbeaumont/PeripheralDevices/blob/master/LICENSE
*/ 

class I2CInterface : public virtual getVersion, public DevErrorReporter{
private: 
protected :
     void*  callback();
     int    lockstatus;	

public : 

        I2CInterface():getVersion( I2CINTERFACE_HDR_VER ,I2CINTERFACE_HDR_VER , __TIME__, __DATE__){
	  lockstatus=0;
	
	};   //Create an I2C Master interface
virtual int     frequency (int hz){ return 0;};//  Set the frequency of the I2C interface. returns 0 when ok,
virtual int     read (int address, char *data, int length, bool repeated=false){return 0;};//Read from an I2C slave.
											   // if repeated is true no stop is generated

// gererate a write (see write function_  with repeat is flase , writing the value reg to the device 
//  the register size is in byte and has the be  equal or smaller then the size_off (int) 
// it  is assumed that the most significant byte off the regeister address is sent first 
virtual int   read_reg( int address, char *data, int length, int reg, int regsize=1) {
		return 0;}

virtual int     read (int &data, int ack){data=0; return 0;};
					// Read a single byte from the I2C bus. ack indicates if the ack has to be generated



virtual int     write (int address, const char *data, int length, bool repeated=false){return 0;};// Write to an I2C slave.
											   // if repeated is true no stop is generated
virtual int     write (int data){return 0;};//  Write single byte out on the I2C bus.
virtual int     start (void){return 0;};// Creates a start condition on the I2C bus.
virtual int     stop  (void){return 0;};// Creates a stop condition on the I2C bus.
virtual int     transfer (int address, const char *tx_buffer, int tx_length, char *rx_buffer, int rx_length, void*  callbackptr, bool repeated=false){
            return 0;
        };     //   Start non-blocking I2C transfer.       not yet clear how to deal with the callback
             // proposol here is for the implementation a spefic call back function ,that includes the event type  
            // wait function that is sometimes needed , not I2C hardware related but different implementation for platforms
virtual void wait_for_ms(int x) { } ;
virtual void wait_for_us(int x)  {  }    
virtual int  abort_transfer(void) {return 0;} 
virtual int  lock(void) {if ( lockstatus) return -1;  lockstatus=1; return 0; } 
virtual int  unlock(void) { lockstatus=0; return 0; }  



}; 

#endif
