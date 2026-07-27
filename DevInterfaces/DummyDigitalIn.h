#ifndef __DUMMYDIGITALIN_H
#define __DUMMYDIGITALIN_H

class DummyDigitalIn {
	
	int lcval;
	public:
	DummyDigitalIn(int value=0 ) { lcval=value;};
	int read(void) { return lcval;}
	int is_connected() { return 1;} 
	DummyDigitalIn& operator= 	( 	DummyDigitalIn &  	rhs	) 	{ lcval = rhs.read() ; return *this;}
	operator int 	( 		) 	{ return lcval;}
	
};



#endif
