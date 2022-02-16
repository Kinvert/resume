/*
 * can drive whith servo angle turr can move
 **/


#include "simpletools.h"                      // Include simple tools
#include "abdrive.h"
//#include "adcDCpropab.h"
#include "fdserial.h"
#include "servo.h"

void drive();   //Functions

  fdserial *xbee;

  static volatile char xchar;
  static volatile int l, r, c;
  static volatile int ud, lr, lrdiff;
  static volatile int glob, ignore;
  static volatile int bu, buck;
 

int main()                                    // Main function
{
  // Add startup code here.
  glob = 0;
  ignore = 0;
  
  
  simpleterm_close();      //Don't print in int main
  xbee = fdserial_open(9, 8, 0, 9600);              //Initializing Communication
  pause(1000);                                      // Wait 1 s for Terminal app
  
  buck = 0;
  
  cog_run(drive, 128);
  
  drive_setRampStep(50);
  
  low(26);
  
  while(1)
  {
    fdserial_rxFlush( xbee ); 
    xchar = fdserial_rxChar( xbee );   
    xchar = fdserial_rxTime( xbee, 1 );
    xchar = fdserial_rxTime( xbee, 1 );
    xchar = fdserial_rxTime( xbee, 1 );
    xchar = fdserial_rxTime( xbee, 1 );
    xchar = fdserial_rxTime( xbee, 1 );
    xchar = fdserial_rxTime( xbee, 1 );
    ud = fdserial_rxTime( xbee, 1 );
    lr = fdserial_rxTime( xbee, 1 );
    
    xchar = fdserial_rxTime( xbee, 1 );
    bu = fdserial_rxTime( xbee, 1 );
    
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
      l = c - lrdiff;
      r = c + lrdiff;
      if( abs(l) < 10 ) l = 0; 
      if( abs(r) < 10 ) r = 0;
      drive_speed(l, r);

      if ( bu == 0 && buck < 590)
      {
        buck = buck + 10;
      }
      else if (bu == 255 && buck > 10)
      {
        buck = buck - 10;
      }
      if ( buck > 600 ) buck = 600;
      if ( buck < 10 ) buck = 10;

      print( " ud = %d    lr = %d   buck = %d\n", ud, lr, buck);
      servo_angle(16, buck);
      pause(5);                            
    }
  }     
}