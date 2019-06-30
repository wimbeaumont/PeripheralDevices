#ifndef MCP_4725_H
#define MCP_4725_H


#include "stdbool.h"

#include "I2CInterface.h" 
#include "DACInterface.h" 

#define VERSION_MCP4725_HDR "0.40"

/** MCP4725 class.
 *  Used for interfacing with a mcp4725 12-Bit Digital-to-Analog Converter.
 *  To convert between the 12bit dac_value and Vout, 
 *     use the following formula: dac_value = (Vout*4096/Vref), where Vout is  
 *  the desired output analog voltage, Vref is the voltage connected to the Vdd pin of the device. 
 *   Typically Vdd will be 3.3volts.
 *
 *  Note: There is an accompanying test suite program "MCP_4725_Library_Test" 
 *  that can be used to test this library.
 
 *   WB  copied from Users » donalm » Code » 
 *      removed the I2C settings etc.  replaced to pointer to I2CInterface class. 
 *        for read by value instead of by pointer
 *  ver 0.1 : setDAC , get DAC working        
 *  ver 0.2 : implemented setVoltage, getVoltage      
 *  ver 0.3 : chang class initialization order,  test eeprom writing. 
 *  ver 0.4 : change code to try to get it running on a Raspberry Pi no functional changes 
 *  
 *  This file make part of the PeriperalDevice package see repository  
 *  https://github.com/wimbeaumont/PeripheralDevices
 *
 *  (C) Wim Beaumont Universiteit Antwerpen 2015  2019 
 *  License see
 *  https://github.com/wimbeaumont/PeripheralDevices/blob/master/LICENSE
 *  
 */

class MCP4725 : public DACInterface
{
    public:
    /** The device supports two types of power modes: normal and power-down. In normal mode the device
     * operates a normal digital to analog conversion. In power-down mode all digitial to analog  
     * conversion is stopped, resulting in the device using less power (typically 60nA). Also, in power 
     * down mode Vout will be pulled to ground using either a 1k, 100k or 500k ohm internal resistors. */
    enum PowerMode {
        /** In normal mode the device operates a normal D2A conversion. */
        Normal=0,
        /** Enter the device into a power down mode, and pull Vout to ground using an internal 1k resistor. */
        PowerDown1k=1,
        /** Enter the device into a power down mode, and pull Vout to ground using an internal 100k resistor. */
        PowerDown100k=2,
        /** Enter the device into a power down mode, and pull Vout to ground using an internal 500k resistor. */
        PowerDown500k=3
    };
    
    /** The device supports 3 different I2C bus frequencies.*/
    enum BusFrequency {
        /** Standard 100kHz bus. */
        Standard100kHz,
        /** Fast 400kHz bus. */
        Fast400kHz,
        /** High Speed 3.4Mhz bus. WARNING: the test suite fails for the mbed LPC1768 when this frequency is selected - not tested on other platforms.*/
        HighSpeed3_4Mhz
    };
    
    /** Create an mcp4725 I2C interface
     * 
     * @param i2cinterface pointer to the  MBED I2C interface 
     * @param scl I2C clock line pin
     * @param bus_frequency the frequency at which the I2C bus is running.
     * @param device_address_bits The 3bit address bits of the device.
     */
    MCP4725(I2CInterface* i2cinterface,  int device_address_bits, float Vdd_);
    
    
    /* Update the power mode and DAC value.
     *
     * @param   mode The Power mode to set the device into.
     * @param   dac_value The 12bit dac register data value.
     * @param   writeToEeprom True if the config is to be stored 
     *          in eeprom, otherwise false
     * 
     * @returns 
     *       0 on success,
     *   non-0 on failure
     */
    int write(enum PowerMode mode, int dac_value, bool writeToEeprom);
    
        
    /** Read the contents of the dac register, the contents of eeprom, and if an eeprom write is currently active.
     * 
     * @param   mode Pointer to variable to store the current power mode.
     * @param   mode_eeprom Pointer to variable to store the power mode as is stored in eeprom.
     * @param   dac_value Pointer to variable to store the current dac value.
     * @param   dac_value_eeprom Pointer to variable to store the dac value as is stored in eeprom.
     * @param   eeprom_write_in_progress Pointer to variable to store the current eeprom write status.
     *
     * @returns 
     *       0 on success,
     *   non-0 on failure
     */
    int read(enum PowerMode& mode, enum PowerMode& mode_eeprom, int& dac_value, int& dac_value_eeprom, bool& eeprom_write_in_progress);
    
    
    /** no action as read always read from the device so returns always 0 */
    virtual int update() { return 0;}  
    virtual int getDACvalue(int &value, int ch=0);
    virtual int setVoltage (float voltage, int ch=0);
    virtual int setDACvalue( int value, int ch=0);
    virtual int getVoltage(float &voltage, int ch=0);
    //** set dac value permanent to eeprom TEST 
    int setDACvalue_ep( int value, int ch=0);
    protected:
    /** mbed I2C interface driver. */
    I2CInterface* _i2c_interface;
    /** The full i2c device address. */
    int _device_address;
    float Vdd;
};

#endif
