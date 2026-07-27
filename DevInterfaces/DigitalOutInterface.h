#ifndef __DIGITALOUTINTERFACE_H
#define __DIGITALOUTINTERFACE_H
// this class is the interface forsingle pin IO  in development don't use 

class DigitalOutInterface {
	protected: 
	int lcval;
	int pinnr;
	public:
	DigitalOutInterface(int iopinnr=1) {pinnr=iopinnr};
	virtual void write(int inp )  { lcval= inp;}
	virtual int read() { return lcval;}
	virtual int is_connected() { return pinnr;} 
	virtual DigitalOutInterface& operator= 	( 	int value	) { lcval=value; return *this; }
	virtual DigitalOutInterface& operator= 	( 	DigitalOutInterface &  	rhs	) 	{ lcval = rhs.read() ; return *this;}
	virtual operator int 	( 		) 	{ return lcval;}
	
};



#endif
