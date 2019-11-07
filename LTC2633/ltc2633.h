#ifndef LTC_2633_H
#define LTC_2633_H
#include "getVersion.h"

#include "stdbool.h"


#include "dev_interface_def.h"
#include "I2CInterface.h" 
#include "DACInterface.h" 

#define VERSION_LTC2633_HDR "1.0"

/** LTC2633 class.
 *  Used for interfacing with a LTC2633 12-Bit dual  Digital-to-Analog Converter.
 *  It has to be used with the  https://developer.mbed.org/users/wbeaumont/code/DevInterfaces/ package 
 *  This includes the "virtual" I2CInterface class that is the interface to the I2C device 
 *  An implementation of the I2Cinterface class for the MBED can be found at 
 *  The LTC2633 comes with different resolution, 12, 10 and 8 bits.  default is 12.
 *  There is no check for the bus frequency ( spec < 400kHZ) 
 *  V 0.10  inital version , copied from mcp4728 
 *  V 1.00  corrections tested with hardware ( only DAC setting not checked the power mode settings) 
 * (C) Wim Beaumont Universiteit Antwerpen 2019
 *  
 */
class LTC2633 : public DACInterface {
   
    public:
 /** The device supports two types of power modes: normal and power-down. In normal mode the device
     * operates a normal digital to analog conversion. In power-down mode all digitial to analog  
     * conversion is stopped, resulting in the device using less power (typically 60nA). Also, in power 
     * down mode Vout will be pulled to ground using either a 1k, 100k or 500k ohm internal resistors. */
    enum PowerMode {
        /** In normal mode the device operates a normal D2A conversion. */
        Normal=0,
        /** put  the ch  into a power down mode */
        PowerDown=1,
        /** Enter the ch  into a power down mode */
        PowerDownAll=2,
        InvalidPm=4,
    };
    /** Voltage reference can be either External, this means the Vdd or internal , 2.048 V 
        The output goes from 0 to Vref ( either 2.048 V or Vdd ), 
    */
    enum VrefMode {
        ExternRef=0,  
        InternRef=1    
    };
  
    
    /** Create an LTC4725 I2C interface
     * The device is power down mode ( all channels and reference) 
     * @param i2cinterface pointer to the   I2C interface (proto type) 
     * @param device_address 
     * @param Vext external reference voltage 
     * @param Vref can be 0 for  -L version ( Vref int = 1.25V) , or 1 for the -H  (Verf int= 2.048V )  1.25 is defaul ( Vref != 1) 
     * @param resolution the number of bits of the device ( 8, 10 ,12) if wrongly defined 8 bits is assumed
     */
    LTC2633(I2CInterface* i2cinterface,  int device_address,  float Vext, int  Vreftype=0,int resolution=8  );
    
     
    
    /**as the ltc2633 doesn't support read actions this function returns -1 
       @param value by reference , the DAC value for the channel
       @param ch  the channel nr 0..3  
       @return -1 
    */
    virtual int getDACvalue(int &value, int ch=0) {notsupported=true; return -1;}

   /** as the ltc2633 doesn't support read actions this function returns -1
       @param voltage by reference , the DAC output voltage for the channel
       @param ch  the channel nr 0..3  
       @return -1
    */
    virtual int getVoltage(float &voltage, int ch=0){notsupported=true; return -1; }
    
    /** set the dac value for a certain channel. 
        It also sets the DAC configuration bits  ( Vref, power mode , gain )  
        the value is in DAC bits so from 0 .. 255 for 8 bits device etc.     
        @param value  the DAC value to be set tuncates depending on the resolution
        @param ch  the channel nr 0..3  
    	@param activate if false writes to the input register but the DAC value is not updated 
        @return the I2C result when trying to set the DAC value
    */
    int setDACvalue( int value, int ch, bool activate );

    /** set the dac value for a certain channel. 
        Implementation of the setDACvalue of the DACInterface 	    it calls the setDACvalue with the parameter active is true 
        the value is in DAC bits so from 0 .. 255 for 8 bits device etc. values are truncated 
        @param value  the DAC value  int bits to be set negative value  result in output 0V
        @param ch  the channel nr 0, 1  15 , if set to 15 both channels are set   
    	@param activate if false writes to the input register but the DAC value is not updated 
        @return the I2C result when trying to set the DAC value
    */ 	
    virtual int setDACvalue( int value, int ch=0 ) { return setDACvalue( value , ch, true); };

    /**
       Set the requeste voltage.  The voltage is truncated to the lower value , depending on the respution
       This funcition use the vref condition (internal external) to calculate the bin value to be set 
       @param voltage by reference , the request  DAC output voltage for the channel negative voltages result in output 0V
       @param ch  the channel  nr 0, 1  15 , if set to 15 both channels are set   
       @return the I2C result when trying to set the DAC value or -111 when ch is wrong
    */   
    int setVoltage (float voltage, int ch, bool activate);

    
    /**
       Set the requeste voltage.  The voltage is truncated to the lower value , depending on the respution
       This funcition use the vref condition (internal external) to calculate the bin value to be set 
       ( it always activat the channel so it calles setVoltate with the paramter activate is true)  
       @param voltage by reference , the request  DAC output voltage for the channel negative voltages result in output 0V
       @param ch  the channel  nr 0, 1  15 , if set to 15 both channels are set   
       @return the I2C result when trying to set the DAC value or -111 when ch is wrong
    */   
    virtual int setVoltage (float voltage, int ch=0) {return setVoltage (voltage,ch, true);};
    
    /** as the ltc2633 doesn't support read actions this function returns -1
       @return -1
    */
    virtual int  update(void) {notsupported=true; return -1; } 

    /* set power mode for the channel.  the value of pm is chip specific.
	default , pm=0  is lowest power mode , pm > 0 active 
	@param ch  0,1, 7 , channel for which the power mode has to be set if set to 7 both channels 
	@param pm  , power mode to which the channel has to be set 
        @return  , result code, 0 is done, other or I2C result or -1 when not supported	
    */
    int setPowerMode(int ch, int pm);	

    /* set power mode for the device ( all channels) The value of pm is chip specific.
	default , pm=0  is lowest power mode , pm > 0 active 
	@param pm  , power mode to which the channel has to be set 0 is power down , 1 is normal 
        @return  , result code, 0 is done, other or I2C result or -1 when not supported	
    */
    int setPowerMode(int pm){ return setPowerMode(7,pm);}		

    /**  prepare to intern reference for a channel 
        @param the channel the internal reference has to be set 
	@return i2c return code 
    */
    int setRefIntern (void );
    
   
    /** prepare to extern reference  for a channel. 
        @param the channel the reference  has to be set for. 
	    @return i2c return code 
    */
    int setRefExtern (void );
  
    
    protected:
    /** pointer to the I2C interface driver. */
    I2CInterface* _i2c_interface;
    /** The full i2c device address. */
    int _device_address;
    char _adrbytes;
    /** calculates the voltage based on the value and the channel configuration bits 
        @param value : the value of the DAC register 
        @param gain  : the gain setting  x1 or x2 
        @param vref  : use internal or external reference 
        @return   : the voltage based on the parameters 
    */

    short int volt2dig( float voltage);
    int chkch(int);
    int writeI2C( char  cmd , int ch, int  dataword );
    char cmd_ch_byte( int ch , int cmd );
    /** holds the Vrefext  value , used to calculate the voltage in case of vref = external */
    float Vrefext;
     /** holds the Vrefint  value , used to calculate the voltage in case of vref = internal  */
    float Vrefint;
    int resolution; 
    /** used setting the configuration bits for each channel
        These values are the "requested" values so not the actual values. 
        These values are not updated after a update request !! 
     */
    VrefMode  vref;
   
};

#endif
