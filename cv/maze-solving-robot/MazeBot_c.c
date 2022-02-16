/**
 * This is the main Brian MMA7455 Accelerometer program file.
 */

#include "simpletools.h"
#include "abdrive.h"
#include "fdserial.h"

fdserial *comms;

//#include <math.h>

//void blink();

unsigned char reply;

static volatile float max = 196.00;

char acc = 1;
char ch = 0;
int mv = 0;
int butt = 0;

int main()
{
  drive_setMaxSpeed(96);
  drive_setRampStep(10);
  
  low(26);
  low(27);
  
  comms = fdserial_open( 31, 30, 0, 9600);
  ch = fdserial_rxChar(comms);
  
  int num = (ch*2-3)*2;
  
  char val[num];
  
  for( int i = 0; i < num; i++ )
  {
    val[i] = fdserial_rxChar(comms);
  }    
  
  while( !butt )
  {
    butt = input(0);
    pause(250);
  }
  
  for( int i = 0; i < num; i++ )
  {
    if(val[i] == 1) //Drive
    {
      i++;
      mv = val[i] * 2;
      drive_goto(mv,mv);
    }
    if(val[i] == 2) //Turn Right
    {
      i++;
      mv = val[i] / 1.75;
      drive_goto(mv,-mv);
      pause(200);
    }
    if(val[i] == 3) //Turn Left
    {
      i++;
      mv = (val[i] / 7) + 0.5; // 45/1.75~26 for 90deg
      drive_goto(-mv,mv);
      pause(200);
    }
  }
  high(27);
}