#ifndef __LINUXI2CINTERFACE_H  
#define __LINUXI2CINTERFACE_H  


/*
 *  This is a Linux implementation of the I2CInterface class 
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
 *  version 0.10  : initial version to see if it compiles at all 
 *
 * (C) Wim Beaumont Universiteit Antwerpen 2019
 *
 * License see
 * https://github.com/wimbeaumont/PeripheralDevices/blob/master/LICENSE
 */

#include "I2CInterface.h" 
#include <unistd.h>  //for the ms 

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#define VERSION_LINUXI2CInterface_HDR "1.21" 


class LinuxI2CInterface :public I2CInterface {
    
    int fdev;
    public :        
    ~LinuxI2CInterface(void){ close(fdev);}	
    LinuxI2CInterface(char* filedescr ): // file points to the I2C device that is openend  
    getVersion( VERSION_LINUXI2CInterface_HDR,VERSION_LINUXI2CInterface_HDR, __TIME__, __DATE__){
	if ((fdev = open(filedescr, O_RDWR)) < 0) {
	    /* ERROR HANDLING: you can check errno to see what went wrong */
	    perror("Failed to open the i2c bus");
	    exit(1);
	}
	
    } ;
    // next could perhaps more efficient  but not yet investigated 
virtual int 	frequency (int hz){  ; return -10;}; // not supported 	
virtual int     read (int address, char *data, int length, bool repeated=false){
		 	if ( repeated == true ) return -100; // not supported 	
			int result= ioctl(fdev, I2C_SLAVE, address); // set the address
			if ( result  < 0) {return result; }  //else so no return 
			if ( read( fdev , data, length) != length	) { return -101;}
			return length;  // so read is ok and data should be in the buffer 	
		};
virtual int    read (int& data , int ack){return  -10; };
				// Read a single byte from the I2C bus. returns the byte read not supported in linux 
virtual int    write (int address, const char *data, int length, bool repeated=false){
		 	if ( repeated == true ) return -100; // not supported 	
			int result= ioctl(fdev, I2C_SLAVE, address); // set the address        
			if ( result  < 0) {return result; }  //else so no return 
			if ( write( fdev , data, length) != length	) { return -101;}
			return length;  // so write is ok and data should be sent to the device
	       }
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
