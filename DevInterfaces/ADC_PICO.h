#ifdef  __PICO__
#ifndef __ADCPICOCLASS__
#define __ADCPICOCLASS__
/*  this interface is to have an integrated ADC of a PICO using 
    the same interface as for the I2C devices 
    so this will only work for PICO hardware 
    
    (c) Wim Beaumont  2025 
*   Ver 0.1  inital version
    Ver 0.2  with mutex to lock the ADC for both cores
*/	    


#define VERSION_ADCPICO_HDR "0.2"
#include "ADCInterface.h"
#include "getVersion.h"
#include "hardware/adc.h"  //pico lib
#include "DevErrorReporter.h"
#include <pico/mutex.h>

class ADC_PICO: public ADCInterface , public DevErrorReporter  {
private : 

		float VrefADC;
		 mutex_t			ADCMutex;					// mutex to lock ADC , all channels !! 
public :
	//initialize the ADC channels, if no connection is made to the ADC ref this is 3.3V ( pico voltage ) 
    ADC_PICO( int* pinlist ,int nrpins,float VrefADCin=3.3  ):
	               getVersion( VERSION_ADCPICO_HDR ,VERSION_ADCPICO_HDR , __TIME__, __DATE__)   {    
		adc_init();
		mutex_init( &ADCMutex );	// create a mutex to lock ADC
		for( int pin;pin<nrpins;pin ++)	{
			if( *pinlist == 30) {
				adc_set_temp_sensor_enabled (true); //power on temperature sensor	
				
			}
			adc_gpio_init(*pinlist++);    // pico lib remove pullup etc.	
			
		}
		VrefADC=VrefADCin; // voltage reference pico ADC 
    };
    
	virtual int  getADCvalue (int& adcvalue , int ch=0){
		mutex_enter_blocking(&ADCMutex);
					adc_select_input(ch); 
					sleep_ms(50);
					adcvalue=adc_read();
		mutex_exit(&ADCMutex);
					return 0 ;  // no error check
					// return (float)adc_read() * VrefADC/(1<<12);
	}
  
	virtual int    getVoltage (float& voltage, int ch=0){ 
		int adcvalue;
		int retvalue= getADCvalue(adcvalue,ch);
			voltage= (float)adcvalue* VrefADC/(1<<12);
			return  retvalue;		
	}
	virtual int     update() {notsupported=true;return -1;};  // general update  by example readout all registers in the device to the class storage. 

	virtual int     startConversion(int ch=0){notsupported=true;return -1;};
	virtual int     statusConversion( int& status, int ch=0){status=1; notsupported=true;return -1;};
	virtual int     getFullRange( ){notsupported=false;return 4096;} 
	virtual int		get_reference_voltage(float &refvolt ){notsupported=false;refvolt=VrefADC ;return 0;}     

    
}; 

#endif    
// so if not __PICO__ this is an empty file 
#endif
