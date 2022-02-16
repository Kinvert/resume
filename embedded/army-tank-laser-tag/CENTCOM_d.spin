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

  rudderc   = 0   'Sets ADC Channels
  aileronc  = 1
  elevatorc = 2
  throttlec = 3

  baud      = 115200

  XB_RX     = 7
  XB_TX     = 6
  XB_Mode   = 0
  XB_Baud   = 9600

  lPin = 26
  rPin = 27

  doPin = 12
  clkPin = 13
  diPin = 14
  csPin = 15

  wpPin = -1
  cdPin = -1
                                   
OBJ
                                      
  AD    : "MCP3208_fast_ADC"          
  DB    : "FullDuplexSerial"
  XB    : "XBee_Object_2"
  WA    : "V2-WAV_DACEngine.spin"  
                           
VAR
          
  long Shared
  word rudder, aileron, elevator, throttle
  long stack[256]
  byte Cog
  byte command
  byte junk
  byte oran, blue, otime, btime
                                                     
PUB Start(Pos) : Pass 
                                              
  AD.Start(ADC_dpin, ADC_cpin, ADC_spin, 0)    'Start ADC
  DB.Start( 31, 30, 0, baud)                   'Start Parallax Serial Terminal communication
  XB.Start( XB_RX, XB_TX, XB_Mode, XB_Baud)
  if(WA.begin(lPin, rPin, doPin, clkPin, diPin, csPin, wpPin, cdPin))
    DB.Str(string("Start: Success"))
  else
    DB.Str(string("Start: Failure"))
  cognew(Check, @stack[0])                     'Value Update Loop

  dira[27]~~
  outa[27]~

  blue := 255
  oran := 255
  otime := 0
  btime := 0
  
  repeat          
    XB.Dec(elevator)
    waitcnt(clkfreq/75 + cnt)
    XB.Dec(elevator)                  'SENDING WRONG SIGNAL TO 2nd BOT 
    waitcnt(clkfreq/75 + cnt)
    XB.Dec(aileron)
    waitcnt(clkfreq/75 + cnt)
    XB.Dec(aileron)                 'SENDING WRONG SIGNAL TO 2nd BOT 
    waitcnt(clkfreq/75 + cnt)
    if INA[0] == 1 and btime == 0                  'blue turret
      btime := 20
      XB.Str(String("t"))
    else
      XB.Str(String("0"))
    if INA[1] == 1 and otime == 0                 'orange turret
      otime := 20
      XB.Str(String("t"))
    else
      XB.Str(String("0"))   
    blue := XB.RxTime(10)
    oran := XB.RxTime(10)
    if btime > 0
      btime--
    if otime > 0
      otime--
    waitcnt(clkfreq/10 + cnt)

Pub Listen

  repeat
    command := DB.rx
    if command == $6C
      outa[27]~~
    else
      outa[27]~


Pub Check

  dira[27]~~                                   'LED setup
  outa[27]~
  dira[26]~~
  outa[26]~
  dira[1]~~
  outa[1]~ 

  repeat                              'Main user accessable loop
    waitcnt(clkfreq / 50 + cnt)       'Slow down long enough to display on PST     
    rudder   := AD.In(aileronc)        'Get ADC value of Rudder
    if aileron < 256
      aileron := 256
    waitcnt(clkfreq / 200 + cnt)
    aileron  := AD.In(aileronc)       'Get ADC value of Aileron
    waitcnt(clkfreq / 200 + cnt)
    elevator := AD.In(elevatorc)      'Get ADC value of Elevator
    if elevator < 256
      elevator := 256
    waitcnt(clkfreq / 200 + cnt)
    throttle := AD.In(elevatorc)      'Get ADC value of Throttle

    {{
    DB.Str(String(16))
    DB.Str(String("Buttons  = "))
    DB.Dec(Shared)                    'Display status of Buttons
    DB.Str(String(13))
    DB.Str(String("Rudder   = "))
    DB.Dec(rudder)                    'Display Rudder value
    DB.Str(String(13))
    DB.Str(String("Aileron  = "))
    DB.Dec(aileron)                   'Display Aileron value
    DB.Str(String(13))
    DB.Str(String("Elevator = ")) 
    DB.Dec(elevator)                  'Display Elevator value
    DB.Str(String(13))
    DB.Str(String("Throttle = "))
    DB.Dec(throttle)                  'Display Throttle value
    DB.Str(String(13))
    }}
    DB.Str(String("Blue = "))
    DB.Dec(blue)
    DB.Str(String("       "))
    DB.Dec(btime)
    DB.Str(String(13))
    DB.Str(String("Orange = "))
    DB.Dec(oran)
    DB.Str(String("       "))
    DB.Dec(otime)    
    DB.Str(String(13))
    
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