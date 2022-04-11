#ifdef __PICO__
//else skip  this 
#ifndef __PICOI2CINTERFACE_H  
#define __PICOI2CINTERFACE_H   

#define PICOI2CINTERFACE_HDR_VER "0.61"

#include "I2CInterface.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"


//#include <unistd.h>  //for the ms 
/*
 * This is a Pico implementation of the I2CInterface class for software developments
 * This file make part of the PeriperalDevice package see repository  
 * https://github.com/wimbeaumont/PeripheralDevices
 * For more info see 	the README.md in the top of repository 
 * ver  0.10  init version  all functions are dummy
 * ver  0.20  init version  compiles with read / write  
 * ver  0.30  tested with HTS221  , read /write , regread , wait , 
 * ver  0.50  tested with HTS221  , read /write , regread , wait , support only standard i2c 
 * ver  0.61  tested with AT30T   added error detection. 
 * (C) Wim Beaumont Universiteit Antwerpen 2022
 * 
 * License see
 * https://github.com/wimbeaumont/PeripheralDevices/blob/master/LICENSE
 * This example will use I2C0 on the default SDA and SCL pins (GP4, GP5 on a Pico
*/ 
#if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
#warning i2c/mcp9808_i2c example requires a board with I2C pins
    puts("Default I2C pins were not defined");
#endif 

class PicoI2CInterface :public I2CInterface  {
    private:
	//const int  I2C_Pico_buffersize=1000;
    //i2c_inst_t *i2cprort = i2c_default;
    public :
    
    PicoI2CInterface(void ) :
	getVersion( PICOI2CINTERFACE_HDR_VER,PICOI2CINTERFACE_HDR_VER, __TIME__, __DATE__){
	       // I2C Initialisation. Using it at 400Khz.
    i2c_init(i2c_default, 10 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    //bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    } ;

virtual int    frequency (int hz){ int getfreq = i2c_set_baudrate (i2c_default, (uint) hz) ; return getfreq;};

	
 virtual int  read (int address, char *data, int length, bool repeated=false){
		int returnvalue=0;
		address=address>>1; // 7 bits address
		comerr=i2c_read_blocking(i2c_default,  (uint8_t) address , (uint8_t*)  data, length,repeated) ;
		if (comerr == PICO_ERROR_GENERIC) returnvalue=-1;
		return returnvalue;
};// Read a single byte from the I2C bus.

virtual int    write (int address, const char *data, int length, bool repeated=false){
		int returnvalue=0;
		address=address>>1; // 7 bits address
		comerr=i2c_write_blocking(i2c_default,(uint8_t) address ,(uint8_t*) data ,length , repeated);
		if (comerr == PICO_ERROR_GENERIC ) returnvalue-1;
		return returnvalue;
};
		

// gererate a write (see write function_  with repeat is false , writing the value reg to the device 
//  the register size is in byte and has the be  equal or smaller then the size_off (int) 
// it  is assumed that the most significant byte off the regeister address is sent first 
virtual int   read_reg( int address, char *data, int length, int reg, int regsize=1) {
	int returnvalue=0;
	address=address>>1; // 7 bits address
	char regbytes[sizeof(int)];
	if ( (unsigned int) regsize > sizeof(int) ) return -11;
		for ( int lc = regsize-1 ; lc >= 0 ; lc--) {
			regbytes[lc] = (char )( reg  & 0xFF);
			reg = reg >>8;
		}
	comerr=i2c_write_blocking(i2c_default,(uint8_t) address ,(uint8_t*) regbytes ,regsize , true );
	if (comerr == PICO_ERROR_GENERIC ) returnvalue=-1;
	comerr=i2c_read_blocking(i2c_default,  (uint8_t) address , (uint8_t*)  data, length,false) ;
	if (comerr == PICO_ERROR_GENERIC ) returnvalue=-1;
	return returnvalue;
}


virtual int     read (int &data, int ack){  notsupported=true;return notsupportederrno; } //not supported 
					// Read a single byte from the I2C bus. ack indicates if the ack has to be generated


virtual int write (int data){ notsupported=true; return notsupportederrno;  };//  not supported 
virtual int start (void) { notsupported=true;return notsupportederrno;}; // not supported 
virtual int stop  (void) { notsupported=true;return notsupportederrno;}; // not supported
virtual int transfer (int address, const char *tx_buffer, int tx_length, char *rx_buffer, int rx_length, void* callbackfunction,  bool repeated=false){
	notsupported=true;return  notsupportederrno;  };    //not supported 
virtual void wait_for_ms(int x)  { sleep_ms(x); }

    } ;

#endif
#endif // __PICO__ 
