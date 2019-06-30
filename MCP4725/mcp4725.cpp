#include "mcp4725.h"


/* 
 *  For info see mcp4725.h 
 *
 *  This file make part of the PeriperalDevice package see repository  
 *  https://github.com/wimbeaumont/PeripheralDevices
 *
 *  (C) Wim Beaumont Universiteit Antwerpen 2015  2019 
 *  License see
 *  https://github.com/wimbeaumont/PeripheralDevices/blob/master/LICENSE
 *  
 */


#define VERSION_MCP4725_SRC "0.40"

MCP4725::MCP4725(I2CInterface* i2cinterface, int device_address_bits, float Vdd_):

    getVersion( VERSION_MCP4725_HDR,VERSION_MCP4725_SRC, __TIME__, __DATE__), _i2c_interface(i2cinterface)

{
    Vdd=Vdd_;
    // Assemble the full I2C device address.
    _device_address = 0xC0; // Prime the full device address with the device code.
    _device_address |= (device_address_bits<<1);
}

int MCP4725::getDACvalue(int& value, int ch){
    // has to limit to reading only the dac value not the other items. 
    enum PowerMode mode; enum PowerMode mode_eeprom; int dac_value; int dac_value_eeprom; bool eeprom_write_in_progress;
    int status = read( mode, mode_eeprom,  dac_value, dac_value_eeprom,  eeprom_write_in_progress);
    value=dac_value;
    return status;
}


int MCP4725::getVoltage(float &voltage, int ch){
  int value;
  int status =getDACvalue(value);
  voltage  =  Vdd * (float) value / 4096;
  return status; 
 }   


int MCP4725::read(enum PowerMode& mode, enum PowerMode& mode_eeprom, int& dac_value, int& dac_value_eeprom, bool& eeprom_write_in_progress)
{
    char data[5];
    int result;
    
    // Read the raw data from the device.
    result = _i2c_interface->read(_device_address, data, sizeof(data)/sizeof(*data), false);
    
    // Parse the raw data, extracting our fields. Refer to MCP4725 ref manual, section 6.2
    if (result == 0)
    {
        eeprom_write_in_progress = (data[0] & 0x80)? false:true;
        
        mode = (enum PowerMode) ((data[0] & 0x06)>>1);
        
        dac_value  = (data[1]<<4) + (data[2]>>4);
        
        mode_eeprom = (enum PowerMode)((data[3] & 0x60)>>5);
        
        dac_value_eeprom = ((data[3] & 0x0F) <<8) + data[4];
    }
     
    return result;
}


int MCP4725::setDACvalue( int value, int ch){
    return write( Normal, value, false);
}

int MCP4725::setDACvalue_ep( int value, int ch){
    return write( Normal, value, true);
}

int MCP4725::setVoltage (float voltage, int ch){
    int value = 4096 * voltage /Vdd; 
    return write( Normal, value, false);
}    
    




int MCP4725::write(enum PowerMode mode, int dac_value, bool writeToEeprom)
{
    char data[3]={0};
    int write_command;
    
    //Which write command are we to use?
    if (writeToEeprom == true)
    {
        //Write DAC Register and EEPROM
        write_command = 3;
    }
    else
    {
        //Write DAC Register
        write_command = 2;
    }
    
    //Assemble our three bytes of data - Refer to MCP4725 ref manual, section 6.
    data[0] = (write_command <<5) | ((int)mode<<1);
    data[1] = (dac_value>>4);
    data[2] = (dac_value<<4);
    
    // Now write them to the device.
    return _i2c_interface->write(_device_address, data, sizeof(data)/sizeof(*data), false);
}
