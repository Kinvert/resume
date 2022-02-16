{{
Keith Young - kyoung21@vols.utk.edu
Updated - Jul 31 2013
Copyright (c) 2012 Keith Young
Terms of use at end of file         
Sound http://obex.parallax.com/object/329
}}
CON

  _clkmode  = xtal1 + pll16x
  '_xinfreq  = 5_000_000
  _clkfreq = 80_000_000                                                       

  ADC_dpin  = 19'1   'Sets ADC Pins
  ADC_cpin  = 20'2
  ADC_spin  = 21'0
  
                 'Sets ADC Channels
  udbuc     = 0    'up/down bulldozer channel
  lrbuc     = 1    'left/right bulldozer channel
  udloc     = 2    'up/down loader channel
  lrloc     = 3    'left/right loader channel
  boloc     = 4    'boom loader channel                                                            
  scloc     = 5    'scoop loader channel
  uddtc     = 6    'up/down dump truck channel
  lrdtc     = 7    'left/right dump truck channel
    

  baud      = 115200  'communication speed for computer to device

  XB_RX     = 7       'receiving pin for XBEE on centcom
  XB_TX     = 6       'transmiting pin for XBEE
  XB_Mode   = 0        'sets mode to zero
  XB_Baud   = 9600     'communication speed for XBEE

  lPin = 26
  rPin = 27
  

  doPin = 12
  clkPin = 13
  diPin = 14
  csPin = 15

  wpPin = -1
  cdPin = -1
                                   
OBJ
                                      
  AD    : "MCP3208_fast_ADC"     'introduces other programs        
  DB    : "FullDuplexSerial"
  XB    : "XBee_Object_2"
  'WA    : "V2-WAV_DACEngine.spin"  
                           
VAR
          
  long Shared
  long stack[256]    'memory for each cog
  byte Cog           'which cog is being used
  byte command       
  byte junk
  word udbu, lrbu, udlo, lrlo, bolo, sclo, uddt, lrdt      'variables for speed
  'byte udC, swingC, boomC, buckC
                                                     
PUB Start(Pos) : Pass 
                                              
  AD.Start(ADC_dpin, ADC_cpin, ADC_spin, 0)    'Start ADC
  DB.Start( 31, 30, 0, baud)                   'Start Parallax Serial Terminal communication
  XB.Start( XB_RX, XB_TX, XB_Mode, XB_Baud)
  'if(WA.begin(lPin, rPin, doPin, clkPin, diPin, csPin, wpPin, cdPin))
  '  DB.Str(string("Start: Success"))
  'else
  '  DB.Str(string("Start: Failure"))
  cognew(Check, @stack[0])                     'Value Update Loop (starting new cog and running check function (methods))

  dira[27]~~                              'pin 27 is an output
  outa[27]~                               'send zero voltage out of pin 27 (basically turning off LED)
  
  repeat                   
          
    XB.Tx(udbu >> 4)    'XB.TX     means transmit
    XB.Tx(lrbu >> 4)    '  udb    udo etc is the variable we want to send
    XB.Tx(udlo >> 4)    '  >> 4     is what is shrinking the 12 bit number to an 8 bit number
    XB.Tx(lrlo >> 4)    '   >> 4 means "Shift Bits Right 4 spaces"
    XB.Tx(bolo >> 4)
    XB.Tx(sclo >> 4)
    XB.Tx(uddt >> 4)
    XB.Tx(lrdt >> 4)
    
    if INA[0] == 0    'dozer scoop up
      XB.Tx(255)                              
    elseif INA[1] == 0
      XB.Tx(0)        'dozer scoop down
    else
      XB.Tx(128)       'dozer scoop stop

    if INA[3] == 0    'Dump Bucket up
      XB.Tx(255)                              
    elseif INA[4] == 0
      XB.Tx(0)        'Dump bucket down
    else
      XB.Tx(128)       'Dump bucket stop


      
    if INA[2] == 0 
      XB.Tx(255)                           
    else
      XB.Tx(0)       
    waitcnt(clkfreq/100 + cnt)

    waitcnt(clkfreq/10 + cnt)

Pub Listen

  repeat
    command := DB.rx
    if command == $6C
      outa[27]~~
    else
      outa[27]~

Pub Check
                                              'LED setup  
  dira[27]~~                                   'first setting as output pin, then setting pin 27, 26, 1 to 0 voltage
  outa[27]~
  dira[26]~~
  outa[26]~
  dira[1]~~
  outa[1]~
   
  repeat                              'Main user accessable loop
    waitcnt(clkfreq / 50 + cnt)   
    udbu   := AD.In(udbuc)            'up/down bulldozer
    if udbu < 256
      udbu := 256
    waitcnt(clkfreq / 200 + cnt)    
    lrbu   := AD.In(lrbuc)             'left/right bulldozer
    if lrbu < 256
      lrbu := 256
    waitcnt(clkfreq / 200 + cnt)
    udlo  := AD.In(udloc)              'up/down loader
    if udlo < 256
      udlo := 256
    waitcnt(clkfreq / 200 + cnt)
    lrlo := AD.In(lrloc)               'left/right loader
    if lrlo < 256
      lrlo := 256
    waitcnt(clkfreq / 200 + cnt)   
    bolo   := AD.In(boloc)             'boom loader
    if bolo < 256
      bolo := 256
    waitcnt(clkfreq / 200 + cnt) 
    sclo   := AD.In(scloc)             'scoop loader
    if sclo < 256
      sclo := 256
    waitcnt(clkfreq / 200 + cnt) 
    uddt   := AD.In(uddtc)             'up/down dump truck
    if uddt < 256
      uddt := 256
    waitcnt(clkfreq / 200 + cnt)
    lrdt   := AD.In(lrdtc)             'left/right dump truck
    if lrdt < 256
      lrdt := 256    

    waitcnt(clkfreq / 200 + cnt)
    if INA[8] == 0  'BLUE TRIGGER              'listen to pin 0, if voltage is high (1) then the trigger was pulled for blue robot
      junk := 255                               'if trigger pulled, transmit a 255
    elseif INA[9] == 0
      junk := 0
    else
      junk := 128
    waitcnt(clkfreq / 200 + cnt)
    if INA[10] == 0  'BLUE TRIGGER              'listen to pin 0, if voltage is high (1) then the trigger was pulled for blue robot
      junk := 255                               'if trigger pulled, transmit a 255
    elseif INA[11] == 0
      junk := 0
    else
      junk := 128

    'DB.Str(String(16)) 
    DB.Str(String("udb = "))        'print the values we're getting for up/down and left/right
    DB.Dec(udlo >> 4)
    DB.Str(String("    "))
    DB.Str(String("lrb = "))
    DB.Dec(udlo >> 4)
    DB.Str(String("    "))
    DB.Str(String("oturr = "))
    DB.Dec(udlo)
    DB.Str(String("    "))
    DB.Str(String("trig = "))
    if INA[0] == 1
      DB.Dec(255)
    else
      DB.Dec(0)
    if INA[2] == 1
      DB.Dec(255)
    else
      DB.Dec(0)
    DB.Str(String(13))             'set string to 13 (next line)
    {{
    if INA[0] == 1 and btime == 0
      result := \WA.play(string("Cannon.wav"))
      repeat until btime == 19
      
    if INA[1] == 1 and otime == 0
      result := \WA.play(string("Cannon.wav"))
      repeat until otime == 19
      
    if oran < 252 or blue < 252
      if oran < 250 or blue < 250
      result := \WA.play(string("Victory.wav"))
      waitcnt(clkfreq * 10 + cnt)
      
      outa[2]~~
      result := \WA.play(string("Alarm.wav"))
      if(WA.playErrorNum)
        DB.Str(string("WAV Error: "))
        DB.Str(result)
        DB.Tx(10)
        repeat          
    else
      outa[2]~
    }}

Pub Stop

  if Cog
    CogStop(Cog~ -1)    
                                     
DAT

{{
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                  TERMS OF USE: MIT License
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation
// files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy,
// modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
// Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
// ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}}                                               