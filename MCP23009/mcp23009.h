#ifndef MCP_23009_H
#define MCP_23009_H
#include "getVersion.h"

#include "stdbool.h"


#include "dev_interface_def.h"
#include "I2CInterface.h" 
#include "DACInterface.h" 

#define VERSION_MCP23009_HDR "0.20"

/** MCP23009 class.
 *  Used for interfacing with a mcp23009 8 bit io expander
 *
 *  It has to be used with the  https://developer.mbed.org/users/wbeaumont/code/DevInterfaces/ package 
 *  This includes the "virtual" I2CInterface class that is the interface to the I2C device 
 *  An implementation of the I2Cinterface class for the MBED can be found at 
 *  interrupt is not supported (yet )
 *  V 0.10  initial version
 *  V 0.20  added outp_status , tested with hardware. 
 * (C) Wim Beaumont Universiteit Antwerpen 2016
 *  
 */
class MCP23009 : public virtual getVersion {
   
    public:
  
        
    

    
    /** Create an mcp23009 I2C interface
     * 
     * @param i2cinterface pointer to the   I2C interface (proto type) 
     * @param device_address_bits The 3bit address bits of the device ( set via an analogue value on the addresspin)
     */
    MCP23009(I2CInterface* i2cinterface,  int device_address_bits );
    
      
    /** set IO pin to input or all  , also set the pullup and  poloarity ( default = no change
     *  @param pinnr  range  0 .. 8  if 8 all pins are set as input 
     *  @param pullup  if positive  pull up will be enabled  0 pull up will be disabled if negative no change
     *  @param polarity if positive polaritye will be inverted  0 std polarity  if negative no change
     *  @return none zero in case of error 
     */
     int set_as_input(int pinnr,int pullup,  int polarity=-1  );   

    /** set IO pin to output  
     *  @param pinnr  range  0 ..   if 8 all pins are set as output 
     *  @param pullup  if true pull up is enabled 
     *  @param polarity if true inverted polarity  not yet tested if the polarity bit has effect on the output
     *  @return none zero in case of error 
     */
     int set_as_output(int pinnr ,int pullup,int polarity=-1 );   
    
    /** read the status of all IO pins and indicate if the pin, with pinnr is  1 or 0
     *  if pinnr= 8 the status of all pins is returned
     *  the status is the level of the port not the "output status" ( open drain so output status can be different) 
     *  @param pinnr : range 0 .. 8  , if 8 the return value is the status of all pins ,if 0 ..7 it returns 0 or 1 
     *  @return   0 or 1 if pinnr 0..7 ,  byte ( 0 ..255  ) if pinnr =8  neg value in case of I2C error 
     */
     int status( int pinnr=8);
     /** read the status of output register all IO pins and indicate if the pin, with pinnr is  1 or 0
     *  if pinnr= 8 the status of all pins is returned
     *  the outp_status is the status of the output latch!! not the actual levels on the IO lines 
     *  @param pinnr : range 0 .. 8  , if 8 the return value is the status of all pins ,if 0 ..7 it returns 0 or 1 
     *  @return   0 or 1 if pinnr 0..7 ,  byte ( 0 ..255  ) if pinnr =8  neg value in case of I2C error 
     */
     int outp_status( int pinnr=8);

     int set( int pinnr , int value);
     protected:
    /** pointer to the I2C interface driver. */
    I2CInterface* _i2c_interface;
    /** The full i2c device address. */
    int _device_address;
      private :
      char data[5];
      int set_io_pin(bool input ,int pinnr,int pullup,  int polarity  );  
      int pinmanipulation ( bool active , int pinnr );
      int status_gen( int pinnr , char reg);
};

#endif
