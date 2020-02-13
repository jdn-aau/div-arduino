// Jens Dalsgaard Nielsen, 2020 
// ARDUINO PART
#include <dumpbytes.h>
#include <Arduino.h>"

// easy to change print around here 
#define PRNT(x) Serial.print(x)
#define PRNT2(x,y) Serial.print(x,y)
// end ARDUINO


void dumpBytesRev(void *p, int antal, char dType)
{
  unsigned char *pp = (unsigned char*)p + (antal - 1);

  while (antal) {
  
    if ( (dType == 'b') || (dType == 'B') ) {
      for (int i = 7; 0 <= i; i--)  {
        if (*pp & (1 << i))
          PRNT("1");
        else
          PRNT("0");
        if (i == 4)
          PRNT(".");
      }
      PRNT(" ");
    }
    else if ( (dType == 'h') || (dType == 'H') ) {
      unsigned char tmp;
      tmp = (*pp >> 4);
      PRNT("0x");
      PRNT2(tmp, HEX);
      tmp = (*pp & 0x0f);
      PRNT2(tmp, HEX);
      PRNT(" ");
    }
    else if ( (dType == 'a') || (dType == 'A') ) {
      if (32 <= *pp && *pp <=126)
        PRNT(*pp);
      else
        PRNT(".");
    }
    
    pp--;
    antal--;
  }
}



void dumpBytes(void *p, int antal, char dType)
{
  unsigned char *pp = (unsigned char*)p;
  while (antal) {
    
    if ((dType == 'b') || (dType == 'B') ) {
      for (int i = 7; 0 <= i; i--)  {
        if (*pp & (1 << i))
          PRNT("1");
        else
          PRNT("0");
        if (i == 4)
          PRNT(".");
      }
      PRNT(" ");
    } 
    else if ((dType == 'h') || (dType == 'H') ) {
      unsigned char tmp;
      tmp = (*pp >> 4);
      PRNT("0x");
      PRNT2(tmp, HEX);
      tmp = (*pp & 0x0f);
      PRNT2(tmp, HEX);
      PRNT(" ");

    }
    else if ( (dType == 'a') || (dType == 'A') ) {
      if (32 <= *pp && *pp <=126)
        PRNT((char)*pp);
      else
        PRNT(".");
    }
    
    pp++;
    antal--;
  }
}





 
