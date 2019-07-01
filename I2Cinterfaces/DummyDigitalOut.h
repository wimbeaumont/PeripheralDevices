#ifndef __DUMMYDIGITALOUT_H
#define __DUMMYDIGITALOUT_H

class DummyDigitalOut {
	
	int lcval;
	public:
	DummyDigitalOut(void) {};
	void write(int inp )  { lcval= inp;}
	int read(void) { return lcval;}
	int is_connected() { return 1;} 
	DummyDigitalOut& operator= 	( 	int  	value	) { lcval=value; return *this; }
	DummyDigitalOut& operator= 	( 	DummyDigitalOut &  	rhs	) 	{ lcval = rhs.read() ; return *this;}
	operator int 	( 		) 	{ return lcval;}
	
};



#endif
