CON

    _clkmode        = xtal1 + pll8x
    _xinfreq        = 10_000_000

    SDDO            =  8    '20
    SDCLK           =  9    '21
    SDDI            = 10    '22
    SDCS            = 11    '23
                       '
    CS              = 12    '26
    DIO             = 15    '27
    CLK             = 14    '28
    ZER             = 13
    BUTT            =  5
    LIGHT           =  6
                                        
OBJ                                                 
        sdfat   : "fsrw"
        H48C    : "H48C Tri-Axis Accelerometer"
        num     : "Numbers_Keith"
                                       
VAR
        long i, j, name, drop
        long fname[256]
        long stack[128]
        long vref,x,y,z,ThetaA,ThetaB,ThetaC
          
PUB Start  | DD
                  
   
  H48C.Start(CS,DIO,CLK)               
  sdfat.mount_explicit(SDDO, SDCLK, SDDI, SDCS)
  Num.Init                        
  i := 0    
  name := 0
  repeat while i == 0
    i := 0
    j := 0                  
    name ++                
    DD := num.ToStr(name,Num#DEC5)   
    if name > 9999
      byte[@fname+j] := byte[DD+0]
      j ++  
    if name > 999  
      byte[@fname+j] := byte[DD+1]
      j ++
    if name > 99   
      byte[@fname+j] := byte[DD+2]
      j ++
    if name > 9    
      byte[@fname+j] := byte[DD+3]
      j ++
    byte[@fname+j] := byte[DD+4]
    j ++    
    byte[@fname+j]   := "."                           
    byte[@fname+j+1] := "t"
    byte[@fname+j+2] := "x"
    byte[@fname+j+3] := "t"
    byte[@fname+j+4] := 0            
    i := sdfat.popen(@fname, "r")
           
  j := 0
  DD := num.ToStr(name,Num#DEC5)   
  if name > 9999
    byte[@fname+j] := byte[DD+0]
    j ++    
  if name > 999  
    byte[@fname+j] := byte[DD+1]
    j ++  
  if name > 99   
    byte[@fname+j] := byte[DD+2]
    j ++  
  if name > 9    
    byte[@fname+j] := byte[DD+3]
    j ++
  byte[@fname+j] := byte[DD+4]
  j ++
      
  byte[@fname+j]   := "."                           
  byte[@fname+j+1] := "t"
  byte[@fname+j+2] := "x"
  byte[@fname+j+3] := "t"
  byte[@fname+j+4] := 0
  readwrite                 

PUB readwrite | DD
 
  dira[7]~~
  outa[7]~
  !outa[7]
  dira[LIGHT]~~
  outa[LIGHT]~
  sdfat.popen(@fname, "a")    
  repeat           
    !outa[LIGHT]   
    x := H48C.x
    y := H48C.y
    z := H48C.z
    drop := ina[ZER]
                              
    DD := num.ToStr(x,Num#DEC5)
    if x < 0
      SDwrite(string("-"))
    x := ||x  
    if x > 9999
      sdfat.pputc(byte[DD+0])    
    if x > 999  
      sdfat.pputc(byte[DD+1])  
    if x > 99   
      sdfat.pputc(byte[DD+2])
    if x > 9    
      sdfat.pputc(byte[DD+3]) 
    sdfat.pputc(byte[DD+4])      
    SDwrite(String(" "))
    
    DD := num.ToStr(y,Num#DEC5)
    if y < 0
      SDwrite(string("-"))
    y := ||y  
    if y > 9999 
      sdfat.pputc(byte[DD+0])    
    if y > 999  
      sdfat.pputc(byte[DD+1])  
    if y > 99  
      sdfat.pputc(byte[DD+2])
    if y > 9   
      sdfat.pputc(byte[DD+3]) 
    sdfat.pputc(byte[DD+4])      
    SDwrite(string(" "))
                          
    DD := num.ToStr(z,Num#DEC5)
    if z < 0
      SDwrite(string("-"))
    z := ||z  
    if z > 9999 'or z < -9999
      sdfat.pputc(byte[DD+0])    
    if z > 999  'or z < -999
      sdfat.pputc(byte[DD+1])  
    if z > 99   'or z < -99 
      sdfat.pputc(byte[DD+2])
    if z > 9    'or z < -9 
      sdfat.pputc(byte[DD+3]) 
    sdfat.pputc(byte[DD+4])      
    SDwrite(String(" "))
    if drop == 1
      SDwrite(String("F"))
    else
      SDwrite(String("G"))  
    
    SDwrite(String(","))
              
    if ina[BUTT] == 1    'END                     
      sdfat.pclose
      outa[LIGHT]~~                         
      waitcnt(clkfreq * 60 + cnt)
      waitcnt(clkfreq * 60 + cnt)
           
    waitcnt(clkfreq / 100 + cnt - clkfreq / 220)   

PUB SDwrite(strAddr )                                   ' Writes the string located at strAddr to the SD card

  repeat strsize(strAddr)                               ' loop for each character in string
    sdfat.pputc(byte[strAddr++])                        ' Write the character to the file that's open    
                               