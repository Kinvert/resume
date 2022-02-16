#include "simpletools.h"                      // Include simple tools
#include "abdrive.h"
#include "adcDCpropab.h"
#include "fdserial.h"
#include "servo.h"

void drive();
void ping();
void hit();

  fdserial *xbee;

  static volatile char ud1, ud2, ud3, ud4;
  static volatile char xchar;
  static volatile int ud;
  static volatile int xint;
  static volatile char lr1, lr2, lr3, lr4;
  static volatile int l;
  static volatile int r;
  static volatile int c;
  static volatile int lrc, lr;
  static volatile int glob;
 
  static volatile char blueh, bluet;
  static volatile int irLeft;

int main()                                    // Main function
{
  // Add startup code here.
  simpleterm_close();
  xbee = fdserial_open(9, 8, 0, 9600);
  pause(1000);                                      // Wait 1 s for Terminal app
  adc_init(21, 20, 19, 18);                         // CS=21, SCL=20, DO=19, DI=18
  
  cog_run(drive, 128);
  cog_run(ping, 128);
  cog_run(hit, 128);
  
  drive_setRampStep(20);
  
  blueh = 255;
  
  low(26);
  
  while(1)
  {
    l = 0;
    r = 0;
    //ud = fdserial_rxChar( xbee );
    //lr = fdserial_rxTime( xbee, 10 );
    ud4 = -1;
    lr4 = -1;
    bluet = 0;
    fdserial_rxFlush( xbee );
    ud1 = fdserial_rxChar( xbee );
    ud2 = fdserial_rxChar( xbee );
    ud3 = fdserial_rxChar( xbee );
    ud4 = fdserial_rxTime( xbee, 1 );
    xchar = fdserial_rxChar( xbee );
    xchar = fdserial_rxChar( xbee );
    xchar = fdserial_rxChar( xbee );
    xchar = fdserial_rxTime( xbee, 1 );
    lr1 = fdserial_rxChar( xbee );
    lr2 = fdserial_rxChar( xbee );
    lr3 = fdserial_rxChar( xbee );
    lr4 = fdserial_rxTime( xbee, 1 );
    xchar = fdserial_rxChar( xbee );
    xchar = fdserial_rxChar( xbee );
    xchar = fdserial_rxChar( xbee );
    xchar = fdserial_rxTime( xbee, 1 );
    
    bluet = fdserial_rxChar( xbee );    //get blue turret
    xchar = fdserial_rxChar( xbee );    //skip orange turret
    
    fdserial_txChar( xbee, blueh );     //send blue health
    xchar = fdserial_rxTime( xbee, 10 );//skip orange health
    
    glob = 1;
    pause(40);
  }  
}

void ping(void *par)
{

  while(1)
  {
    if( bluet == 116 )
    {
      high(27);
      freqout( 11, 10, 38000);
      pause(1000);
      low(27);
    }      
    
  }    
}

void hit(void *par)
{
  while(1)
  {
    irLeft = input(10);
    if ( irLeft == 0 )
    {
      blueh--;
      pause(1000);
    }      
    if ( blueh < 252 ) high(26);
  }    
}  

void drive(void *par)
{
  simpleterm_open();
  while(1)
  {
    while( glob == 1)
    {
      glob = 0;
      
      print("%c", CLS);
      //print( "ud4 = %d ud3 = %d ud2 = %d ud1 = %d\n", ud4, ud3, ud2, ud1 );
      if( ud4 >= 48 && ud4 <= 57 )
      {
        ud = (ud1-48)*1000 + (ud2-48)*100 + (ud3-48)*10 + (ud4-48);
      }      
      else if( ud4 < 48 || ud4 > 57 )
      {
        ud = (ud1-48)*100 + (ud2-48)*10 + (ud3-48);
      }
      if( lr4 >= 48 && lr4 <= 57 )
      {
        lrc = (lr1-48)*1000 + (lr2-48)*100 + (lr3-48)*10 + (lr4-48);
      }      
      else
      {
        lrc = (lr1-48)*100 + (lr2-48)*10 + (lr3-48);
      }
      
      
      c = .0667*ud - 145.071;
      lr = .0667*lrc - 145.071;
      if( ud > 2000 && ud < 2200 ) c = 0;
      if( lrc > 2000 && lrc < 2150 ) lr = 0;
      l = c + lr;
      r = c - lr;
      
      print( "blueh = %d IRLEFT = %d\n", blueh , irLeft);
      print( " ud = %d lr = %d butt = %d\n ", c, lr, bluet );
      if( bluet == 116   )
      {
        print("FIRE\n");
        pause(1000);
      }  

      drive_rampStep( l,r );
    }     
  }     
}