#ifndef __DIGITALOUTPICO_H
#define __DIGITALOUTPICO_H
/** 
 * pico digital output  
 * GPIO_SLEW_RATE_FAST / GPIO_SLEW_RATE_SLOW
 * GPIO_DRIVE_STRENGTH_2MA  ,4,8,12MA
 * 
 * V0.1 20250821  initial (quick )development irq is not supported 
 * V0.2 20251030  basic is working tested with the HVON 
 * 
 */ 
#include "hardware/gpio.h"

class DigitalOutPico {
	protected: 
	int lcval;
	int pinnr;
	public:
	enum PULLUPP_DEFS{PULL_UP=0,PULL_DOWN=1,PULL_NONE=3};
	DigitalOutPico(int iopinnr=1 , int pull_set=PULL_UP ,
		   gpio_drive_strength drivestrengt=GPIO_DRIVE_STRENGTH_4MA  , 
		    gpio_slew_rate  slew_rate=GPIO_SLEW_RATE_FAST ) {   
		pinnr=iopinnr;
		gpio_init(pinnr);  //set pin function to GPIO_FUNC_SIO
		gpio_set_dir(pinnr, GPIO_OUT);
		gpio_set_slew_rate (pinnr,slew_rate);
		gpio_set_drive_strength (pinnr,drivestrengt);
		if (pull_set == PULL_UP)  {gpio_set_pulls (pinnr, true , false);  // up= true, down=false
		} else if ( pull_set == PULL_DOWN) { gpio_set_pulls (pinnr, false , true);  // up= false, down=true
		} else {	gpio_set_pulls (pinnr, false , false);  // up= falsee, down=false
		}
	}
	 void write(int inp )  { if (inp) inp=1; else inp=0; gpio_put(pinnr, inp );}
	 int read() { return gpio_get_out_level (pinnr);}
	 int is_connected() { if ( gpio_get_function(pinnr) == GPIO_FUNC_SIO ) return pinnr ; else return -1;} 
	//virtual DigitalOutInterface& operator= 	( 	int  	inp 	) {if (inp) inp=1 else inp=0; gpio_put(pinnr, inp ); lcval=value; return *this; }
	//virtual DigitalOutInterface& operator= 	( 	DigitalOutInterface &  	rhs	) 	{ lcval = rhs.read() ; return *this;}
	//virtual operator int 	( 		) 	{ return gpio_get_out_level (pinnr);}
	
};



#endif
