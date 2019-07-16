#ifndef __LINUXI2CINTERFACE_H  
#define __LINUXI2CINTERFACE_H  


/*
 *  This is a simplified of Linux implementation of the I2CInterface class 
 *  for test purposes. Will not be maintaned !!! 
 *  It's use the /dev/i2c device what is given as parameter 
 *  Most of the function are not implemented as the linux interface is very limitted 
 *  For more complex actions ( bus abort etc) it is not clear what is implemented. 
 *  But still good enough for simple devices 
 *  For the Raspberry Pi plan is  to use the bmc2835 lib but I wait a little bit with this
 *  to see what will be the best to use with the Raspberry Pi 4 
 *
 *
 *  This file make part of the PeriperalDevice package see repository  
 *  https://github.com/wimbeaumont/PeripheralDevices
 *  For more info see 	the README.md in the top of repository 
 *
 *  version 0.11  : initial version to see if it compiles at all 
 *
 * (C) Wim Beaumont Universiteit Antwerpen 2019
 *
 * License see
 * https://github.com/wimbeaumont/PeripheralDevices/blob/master/LICENSE
 */



#include "LinuxI2c.h" 
#define VERSION_LINUXI2CInterface_HDR "2.00" 


class LinuxI2CInterface  {
    LinuxI2C li2c;
    
    public :        
    LinuxI2CInterface(char* filedescr ):li2c( filedescr){

    };
    ~LinuxI2CInterface(void){ }	
    // next could perhaps more efficient  but not yet investigated 
virtual int 	frequency (int hz){  ; return -10;}; // not supported 	
virtual int     read (int address, char *data, int length, bool repeated=true){
			int lresult=-1000;
		 	if ( repeated == true ) return -100; // not supported 	
			lresult=li2c.setaddr(address);
			if ( lresult==0) lresult= li2c.linux_read(data, length);
			return lresult;  // so read is ok and data should be in the buffer 	
		};
virtual int    read (int& data , int ack){return  -10; };
				// Read a single byte from the I2C bus. returns the byte read not supported in linux 
virtual int    write (int address, char *data, int length, bool repeated=false){
			int lresult=-100;
		 	if ( repeated == true ) return -100; // not supported 	
			lresult=li2c.setaddr(address);
			if ( lresult==0) lresult= li2c.linux_write(data, length);
			return lresult;  // so read is ok and data should be in the buffer 	
		};


virtual int    write (int data){  return -10 ; } 
		//  Write single byte out on the I2C bus. returns  the ack status  not supported for linux
virtual int    start (void){return -10; };// not supported for Linux 
virtual int    stop (void){return -10; };// not supported for Linux
virtual int    transfer (int address, const char *tx_buffer, int tx_length, char *rx_buffer, int rx_length, void* callbackfunction,  bool repeated=false){
            return  -10;  // seems transfer not supported in Linux or not correctly 
        };    

virtual int  abort_transfer(void) {return -10;}  // not supported  for linux 
// use lock and unlock from the interface 


virtual void wait_for_ms(int x)  {  usleep(1000*x); }
    
} ;  //end class 


#endif
