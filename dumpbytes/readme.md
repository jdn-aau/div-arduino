## dumpbytes

Just a simple library for dumping memory  in either binary, hex or ascii format


long x = 0x12345678;
unsigned int xx = 0x0120;


dumpes as memory layout


     Serial.println("Little Endian so LSB first - as memory layouot");
    dumpBytes((void *)(&x), 4,'h');  // h/H  hex
    Serial.println("");
    dumpBytes((void *)(&x), 4,'b');  // b/B  bin
    Serial.println("");
    dumpBytes((void *)(&x), 4,'a');  //  a/A  ascii /.

The same in more "readable" format - MSB first - as we read from lefto to right :-)

    dumpBytesRev((void *)(&x), 4,'h');
    Serial.println("");
    dumpBytesRev((void *)(&x), 4,'b');
    Serial.println("");
    dumpBytesRev((void *)(&x), 4,'a'); 


"4" is just the number of bytes to be dumped from start of memeory location   ( here it is &x)



## Arduino

The code is generic but in dumpbytes.cpp there is ...

// ARDUINO
#include <dumpbytes.h>
#include <Arduino.h>"
// easy to change print around here 
#define PRNT(x) Serial.print(x)
#define PRNT2(x,y) Serial.print(x,y)
// end ARDUINO


which can be changed if you are running another system
