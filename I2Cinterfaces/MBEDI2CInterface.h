#ifndef __MBEDI2CINTERFACE_H  
#define __MBEDI2CINTERFACE_H  

#include <mbed_version.h>

/*
 *  This is a MBED implementation of the I2CInterface class 
 *  This file make part of the PeriperalDevice package see repository  
 *  https://github.com/wimbeaumont/PeripheralDevices
 *  For more info see 	the README.md in the top of repository 
 *  version 1.10  :added wait_for_ms 
 *  version 1.21  : followed changes of I2CInterface for start stop frequency
 *  version 1.30  : upated to mbed os5  interface should still work for mbed os2 
 *  version 1.40  : implemented read_reg methode
 *  version 1.41  : correction on implemented read_reg methode
 *  version 1.50  : added comerr  for I2C access
 *  version 1.60  : corrected error in readreg 
 *  (C) Wim Beaumont Universiteit Antwerpen 2019
 *
 *  License see
 *  https://github.com/wimbeaumont/PeripheralDevices/blob/master/LICENSE
 */

#include "I2CInterface.h" 

#define VERSION_MBEDI2CInterface_HDR "1.60" 


class MBEDI2CInterface :public I2CInterface {
    
    I2C i2cdev;
    public :
        
    MBEDI2CInterface(PinName sda, PinName scl): 
    getVersion( VERSION_MBEDI2CInterface_HDR,VERSION_MBEDI2CInterface_HDR, __TIME__, __DATE__),i2cdev(sda,scl){
        // no init code yet 
    } ;
    // next could perhaps more efficient  but not yet investigated 
virtual int 	frequency (int hz){ i2cdev.frequency(hz) ; return 0;};
virtual int     read (int address, char *data, int length, bool repeated=false){
			return comerr=i2cdev.read ( address, data, length, repeated);
		};
 virtual int    read (int& data, int ack){ data= i2cdev.read ( ack); return 0;};// Read a single byte from the I2C bus.


// gererate a write (see write function_  with repeat is flase , writing the value reg to the device 
//  the register size is in byte and has the be  equal or smaller then the len (int) 
// it  is assumed that the most significant byte off the regeister address is sent first 
virtual int   read_reg( int address, char *data, int length, int reg, int regsize=1) {
		// first check the register size 
		char regbytes[sizeof(int)];
		if ( (unsigned int) regsize > sizeof(int) ) return -11;
		for ( int lc = regsize-1 ; lc >= 0 ; lc--) {
			regbytes[lc] = (char )( reg  & 0xFF);
			reg = reg >>8;
		}
		comerr= i2cdev.write ( address, regbytes, regsize,true  ); // no stop !!
		if ( comerr == 0) {
			comerr = i2cdev.read ( address, data, length, false); // now stop I2C
		} else {
			comerr= 10*comerr;//so  -10 to get the difference with read error 
		}
		return comerr;
}


 virtual int     write (int address, const char *data, int length, bool repeated=false){
            return comerr=i2cdev.write ( address, data, length, repeated);
        }
virtual int    write (int data){return i2cdev.write (data);};//  Write single byte out on the I2C bus.
virtual int    start (void){i2cdev.start (); return 0; };// Creates a start condition on the I2C bus.
virtual int    stop (void){i2cdev.stop(); return 0; };// Creates a stop condition on the I2C bus.

#if DEVICE_I2C_ASYNCH

virtual int    transfer (int address, const char *tx_buffer, int tx_length, char *rx_buffer, int rx_length, void* callbackfunction,  bool repeated=false){
               return comerr=i2cdev.transfer (address, tx_buffer,  tx_length, rx_buffer,  rx_length, callback,  event,  repeated);
        };    
#else

virtual int    transfer (int address, const char *tx_buffer, int tx_length, char *rx_buffer, int rx_length, void* callbackfunction,  bool repeated=false){
				return setnotsupported();
				}

#endif 
// new since mbed os 5  
#if (MBED_MAJOR_VERSION  > 4 ) 
#if DEVICE_I2C_ASYNCH
virtual int  abort_transfer(void) {i2cdev.abort_transfer();return 0;}  // assumes it always works so return 0 
#else
virtual int  abort_transfer(void) {return setnotsupported();}
#endif

virtual int  lock(void) {i2cdev.lock(); return 0; } 
virtual int  unlock(void) { i2cdev.unlock(); return 0; } 
#endif  // mbed version   
// #else use the functions from the I2C interface 

virtual void wait_for_ms(int x)  {  wait_us(1000*x); }
    

} ;


#endif
