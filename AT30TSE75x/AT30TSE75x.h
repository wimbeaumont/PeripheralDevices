#ifndef AT30TSE75x_H
#define AT30TSE75x_H
#include "getVersion.h"

#include "stdbool.h"


#include "dev_interface_def.h"
#include "I2CInterface.h" 


#define VERSION_AT30TSE75x_HDR "0.95"

/** AT30TSE75x class.
 *  Used for interfacing with a AT30TSE75x temperature sensor and ee-prom  
 *  For version 0.1 inital 
 *  It has to be used with the  https://developer.mbed.org/users/wbeaumont/code/DevInterfaces/ package 
 *  This includes the "virtual" I2CInterface class that is the interface to the I2C device 
 *  An implementation of the I2Cinterface class for the MBED can be found at 
 *  https://developer.mbed.org/users/wbeaumont/code/I2Cinterfaces/ 
 *  ee-prom set / read not tested / implemented. 
 *  version 0.8  : read of temperature  12 bits, fixed configuration
 *                  read /write ee-prom 
 *  version 0.86 : added ee-prom write protect mode tested  ( un-protect not tested)
 *  version 0.90 : added all set and get functions for reading the config register(s)
 *                  most of the functions are not tested . 
 *  version 0.95 : found error in set temperature limit registers , corrected set and get limit register 
 *                  function calls               
 *  this code is only tested with the AT30TSE752  version ( 2 KBit version) 
 *  v 
 *  processor board  for the testing : FRDM-KL25Z 
 * 
 * (C) Wim Beaumont Universiteit Antwerpen 2016, 2017
 *  
 */
class AT30TSE75x : public virtual getVersion {
   uint8_t buffer[4];
   int Taddr, Eaddr;  // base address for temperature and eeprom 
   int Esize;
   void set_LimitRegister(int &error ,uint8_t reg  ,float temperature, int Nonvolatile);
   float  get_LimitRegister(int &error ,uint8_t reg  , int Nonvolatile);
   void CopyRegisters( int &error , uint8_t reg);
   float  convert_temperature( uint16_t datain);
   uint16_t  set_temperature(float temperature );
   int initstatus;
   protected:
   struct configreg {
        bool oneshot;
        uint8_t resolution;
        uint8_t faultTolQueue;
        bool alertpol;
        bool alarmMode;
        bool shutdownMode;
        bool NVregBusy;
        bool RegisterLockDown;
        bool RegisterLock;
    };
    struct configreg  CregNV, Creg;
    /** pointer to the I2C interface driver. */
    I2CInterface* _i2c;
   public:
   
   enum { active_mode=0, disabled_mode=1 } shutdownmodes;
   enum { comparator_mode=0 , interrupt_mode=1 } alarmthermostatmodes;
   AT30TSE75x (I2CInterface* i2cinterface,  int device_address_bits, int eepromsize=2);
    int getInitStatus(void) { return initstatus;}
    int getTaddr(){ return Taddr;};
    int getEaddr(){ return Eaddr;};
    uint16_t get_temperature_register(int &error);
    float get_temperature(int &error);
    // write the status of the configreg  structure to the register ( NV or volatile ) r
    // @error , report error I2C error or 43 in case the register is locked permanent and writing to nonvolatile register
    //            error is 44 when register lock bit  is active (in the structure) no update is done 
    // @param Nonevolatile  , write to Nonvolatile register if 1 ( so no effect )
    //                          write to volatile config register  so effect 
    void set_config( int &error, int Nonvolatile=0 );
    uint16_t read_config(int &error,  int Nonvolatile=0  );
    // set the one_shot bit in the (Volatile) config register and force a temperture read during shutdown mode 
    // it reads first the config register and then set the bit and writes the config register
    // @return 0 
    void activate_oneshot(int &error  ); 
    // reads the resolution bits from the sconfig structure
    // @param  Nonvolatile  if 1 read from the Nonvolatile register 
    // @param update  if 1  reads first the register and fills the structure
    // @param error is  0 if success else error ( I2C error in case update =1) 
    // @return  the resolution bits 
    int get_resolution(int &error,  int Nonvolatile=0, bool update=0 );
    int get_FaultTollerantQueue(int &error, int Nonvolatile=0, bool update=0 );
    int get_AlertPinPolarity(int &error,  int Nonvolatile=0, bool update=0 );
    int get_AlarmThermostateMode(int &error,  int Nonvolatile=0, bool update=0 );
    int get_ShutdownMode(int &error,  int Nonvolatile=0, bool update=0 );
    // reads the resolution bits from the sconfig structure
    // @param  Nonvolatile  has to be 1 , in case of 0 no read is done and error is set to none zero
    // @param update  if 1  reads first the register and fills the structure
    // @param error is  0 if success else error ( I2C error in case update =1) 
    // @return  the  Register lock status 
    int get_RegisterLock (int &error,  int Nonvolatile=1, bool update=0 );
    int get_RegisterLockdown (int &error,  int Nonvolatile=1, bool update=0 );
    // reads the config register updates the reg structure  and reports the  Nonvolatile busy bit 
    int get_NonevolatileBussy(int &error);
    
    // reads the resolution bits from the sconfig structure
    // @resolution to be set can be either bit value 0 .. 3  or 9,10, 11, 12 for bit resolution
    // @param  Nonvolatile  if 1 set the Nonvolatile register 
    // @param write if 0 only the  structure is updated and no write to the config register 
    // @param update  if 1  reads first the register and fills the structure
    // @param error is  0 if success else error ( I2C error in case update =1) 
    void set_resolution(int resolution , int &error,  int Nonvolatile=0,bool write=0, bool update=0 );
    void set_FaultTollerantQueue(int ftq, int &error, int Nonvolatile=0, bool write=0, bool update=0 );
    void set_FaultTollerantQueue(char nrfaults, int &error, int Nonvolatile=0,bool write=0,  bool update=0 );
    void set_AlertPinPolarity(int pol, int &error,  int Nonvolatile=0,bool write=0, bool update=0 );
    //void set_AlarmThermostateMode(AT30TSE75x::alarmthermostatmodes mode , int &error,  int Nonvolatile=0, bool update=0 ){
     //   set_AlarmThermostateMode((int) mode , error,  Nonvolatile,  update=0 );}
    void set_AlarmThermostateMode(int mode , int &error,  int Nonvolatile=0, bool write=0,bool update=0 );
    //void set_ShutdownMode(AT30TSE75x::shutdownmodes mode , int &error,  int Nonvolatile=0, bool update=0 ){
     //    set_ShutdownMode((int) mode , error,  Nonvolatile, bool ) ;}
    void set_ShutdownMode(int mode , int &error,  int Nonvolatile=0,bool write=0, bool update=0 );
    // set the register loc bit reads nonvolitaile config register and update the structure CregNV
    // @param lock  locks status and limits registers ( None Volitaile and volitaile) if set to   1 or unlock ( lock=0) 
    // @param error is  0 if success else error ( I2C error in case update =1) 
    void set_RegisterLock (int &error, int lock   );
    // set the register loc bit reads nonvolitaile config register and update the structure CregNV
    // @param error  reports an I2C error or wrong condition.  Has to be 123 when called; 
    // use this with caution 
    //  error will be 35 if #define RegisterLocDownEnable 0 in the code file 
    //  error will be 36 if called with error an other value then 123 
    // most likely this function is not tested. 
    void set_RegisterLockdown (int &error, int lockpermanent  );
    
    // limit registers
    // set the low limit temperature register 
    //@param error  reports I2C error , in case the lock bit is set it error will  44 an no attempt is done to write 
    //                          if Nonvolatile=1 and the permanentlock bit is set error will be 43 
    //@param temperature   the temperature to be set in the register 
    //@param Nonvolatile   set the Nonvolataile register if set to none zero 
    void set_TLowLimitRegister(int &error ,  float temperture, int Nonvolatile=0) ;
    void set_THighLimitRegister(int &error ,  float temperture, int Nonvolatile=0) ;
    float get_THighLimitRegister(int &error ,   int Nonvolatile=0) ;
    float get_TLowLimitRegister(int &error ,  int Nonvolatile=0) ;
    
    // copies the Volatile registers to the NoneVolataile registers 
    // reads the volataile config register to check the NoneVolataile register status
    // if bussy waits for 100ms and tries again ( max 300ms wait) 
    // @param error   returns the I2C error , 
    //              returns 22 in case register stays busy 
    //              returns 43 if lock bit is set
    //              returns 44 if lockdown bit is set 
    void CopyVolatile2NoneVolatileRegisters( int &error); 
    void CopyNonVolatile2VolatileRegisters( int &error) ;
    
    //eeprom
    // max lenght =16 for 1 page.  If word =0 
    int read_eeprompage(char *data, uint8_t length, uint8_t word_addr, uint8_t page);
    int write_eeprompage(char *data, uint8_t length, uint8_t word_addr, uint8_t page);
    // read a single byte from ee prom
    int read_eeprombyte(char &data, uint8_t word_addr, uint8_t page);
    int write_eeprombyte(char data,  uint8_t word_addr, uint8_t page);
    // set ee-prom in read only  mode special voltage on A0 is needed  and A1 and A2 shoud be low.
    int protect_eeprom(void);
    // set ee-prom in read only  mode special voltage on A0 and  A1 should be high , A2 should be low
    int unprotect_eeprom(void);
    // returns I2C err ( NACK) in case protected  , else ACK ( no error =0) 
    int get_eeprom_protec(void);
    
 };
 
 #endif   