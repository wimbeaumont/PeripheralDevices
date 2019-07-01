#include "mcp23009.h"
//#include "mbed.h"  // should not be in only for printf debug ?


#define VERSION_MCP23009_SRC  "0.20"  

// definition registers 

#define IODIR   0
#define IPOL    1 
#define GPINTEN 2
#define DEFVAL 3
#define INTCON 4
#define IOCON 5
#define GPPU 6
#define INTF 7 
#define INTCAP 8
#define GPIO 9 
#define OLAT 10

MCP23009::MCP23009(I2CInterface* i2cinterface, int device_address_bits  ): 
    getVersion( VERSION_MCP23009_HDR,VERSION_MCP23009_SRC, __TIME__, __DATE__),_i2c_interface(i2cinterface) {
    int _adrbytes= device_address_bits & 7;
    // Assemble the full I2C device address.
    _device_address = 0x40; // Prime the full device address with the device code.
    _device_address |= (_adrbytes<<1);
   
    data[0]=IOCON; data[1]=0x20; // set no increase addr pointer, all intterupt off 
    _i2c_interface->write(_device_address, data,2);     
}

int MCP23009::outp_status( int pinnr){
    return status_gen(pinnr,OLAT);
}

int MCP23009::status( int pinnr){
    return status_gen(pinnr,GPIO);
}    

int MCP23009::status_gen( int pinnr , char reg){
    int result=0;
    data[0]=reg;
    result= _i2c_interface->write(_device_address, data,1); // set reg
    if ( result ) return -1;
    char rb[1];
    result = _i2c_interface->read(_device_address,rb,1); // read the current status 
    if ( result ) return -2;
    if ( pinnr != 8) {
        rb[0] = rb[0] >> pinnr;
        rb[0] = 1 &rb[0];
    }
    return rb[0];
}    
    
int MCP23009::set( int pinnr , int value){
    int result=0;
    data[0]=OLAT;
    if ( pinnr != 8) {
        bool vb=false;
        if ( value > 0) {vb=true;}
        result=pinmanipulation( vb , pinnr);         
    }
    else {
        data[1]= (unsigned char) value ;
        result = _i2c_interface->write(_device_address, data,2);     
    }
    return result;
}    

 int MCP23009::set_as_input(int pinnr,int pullup,  int polarity  ){
     return  set_io_pin(true , pinnr, pullup,   polarity  );
}
 int MCP23009::set_as_output(int pinnr,int pullup,  int polarity  ){
     return  set_io_pin(false , pinnr, pullup,   polarity  );
}

int MCP23009::set_io_pin(bool input ,int pinnr,int pullup,  int polarity  ){
    int result=0;
    if ( pinnr >8 || pinnr < 0 ) return -20;
    
    data[0]=IODIR;  
    result=pinmanipulation( input , pinnr); 
    if ( result ) return result;
    if (pullup >= 0) {
        data[0]=GPPU;
        bool pullup_b=false; 
        if ( pullup > 0) pullup_b =true;
        result=pinmanipulation( pullup_b , pinnr); 
        if ( result ) return result;
    }
    if (polarity >= 0) {
        data[0]=IPOL;
        bool pol_b=false; 
        if ( polarity > 0) pol_b =true;
        result=pinmanipulation( pol_b , pinnr); 
        if ( result ) return result;
    }
    return 0;
}    
    
    
int MCP23009::pinmanipulation ( bool active , int pinnr ){
    int result=-10;
    if (pinnr < 8 ) {
        result= _i2c_interface->write(_device_address, data,1); // set to ioreg
        if ( result ) return -1;
        char rb[1];
        result = _i2c_interface->read(_device_address, rb,1); // read the current status 
        if ( result ) return -2;
        data[1]= 1<<pinnr;
        if ( active ) { data[1] |= rb[0]; } // new reg value for  pinnr input      
        else {      data[1] = ~data[1];
                    data[1] &= rb[0]; // new reg value for  pinnr ouput      
        }         
        if ( rb[0] != data[1]) { // only write if different                    
            result=_i2c_interface->write(_device_address, data,2);     
        }        
    }
    else { // change all pins
      if ( active) data[1]=0xFF; else data[1]=0;
      result=_i2c_interface->write(_device_address, data,2);                    
    }
    return result;
}

