/*
 * can drive whith servo angle turr can move
 **/


#include "simpletools.h"                      // Include simple tools
#include "abdrive.h"
//#include "adcDCpropab.h"
#include "fdserial.h"
#include "servo.h"

void drive();   //Functions
void trigger();

  fdserial *xbee;

  static volatile char xchar;
  static volatile int l, r, c;
  static volatile int ud, lr, lrdiff;
  static volatile int glob, ignore;
  static volatile int bo, sc, boom, scoop;
 

int main()                                    // Main function
{
  // Add startup code here.
  glob = 0;
  ignore = 0;
  
  
  simpleterm_close();      //Don't print in int main
  xbee = fdserial_open(9, 8, 0, 9600);              //Initializing Communication
  pause(1000);                                      // Wait 1 s for Terminal app
  
  sc = 900;
  bo = 900;
  
  cog_run(drive, 128);
  
  drive_setRampStep(50);
  
  low(26);
  
  while(1)
  {
    fdserial_rxFlush( xbee ); 
    xchar = fdserial_rxChar( xbee );
    ud = fdserial_rxTime( xbee, 1 );
    lr = fdserial_rxTime( xbee, 1 );
    xchar = fdserial_rxTime( xbee, 1 );
    xchar = fdserial_rxTime( xbee, 1 );
    xchar = fdserial_rxTime( xbee, 1 );
    xchar = fdserial_rxTime( xbee, 1 );
    xchar = fdserial_rxTime( xbee, 1 );
    xchar = fdserial_rxTime( xbee, 1 );
    
    xchar = fdserial_rxTime( xbee, 1 );
    xchar = fdserial_rxTime( xbee, 1 );
    
    glob = 1;
    pause(45);
  }  
}

void drive(void *par)
{
  simpleterm_open();
  while(1)
  {
    while( glob == 1 )
    {
      glob = 0;
      c = 0.418 * ud - 56.7;
      lrdiff = -0.418 * lr + 56.7;
      l = 900 + c - lrdiff;
      r = 900 - c - lrdiff;
      if( abs(l-900) < 10 ) l = 900; 
      if( abs(r-900) < 10 ) r = 900;
      //drive_speed(l+900, r+900);                    //pin 13 for left servo
      servo_angle(14, l);
      servo_angle(15, r);
      //print( " l = %d     r = %d\n", l, r);
      print( " ud = %d     lr = %d   l = %d  r = %d\n", ud, lr, l, r);
      /*
      if( abs(bo-125) > 10 ) boom = boom  - (bo - 125)/2;
      if( abs(sc-125) > 10 ) scoop = scoop  - (sc - 125);
      if( boom < 750 ) boom = 750;
      if( boom > 1550 ) boom = 1550;
      if( scoop < 300 ) scoop = 300;
      if( scoop > 1800 ) scoop = 1800;
      print( " ud = %d    lr = %d   boom = %d  scoop = %d\n", ud, lr);
      servo_angle(16, boom);
      servo_angle(11, scoop);
      */
      pause(25);                            
    }
  }     
}