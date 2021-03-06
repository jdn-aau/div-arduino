/* 
* COBS  - Consistent Overhead Byte Stuffing
* idea from  https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing 
*
* Limitations
* max lenght of userdata is 254 (COBSMAX)
*
* Jens Dalsgaard Nielsen (C) - beerlicense 
*/

 /*
* Encodes a up to 254
* NB d array shall be 3 Bytes longer than your data
*/

#include <cobs.h>

int cobsEncode(char *d, char *dst, int nB)
{
    if (nB < 1 || COBSMAX < nB)
    {
        return -1; // error length(nBytes) of org packet outside range	[1..254]
    }
    if (dst == 0)
    	dst = d;

    char nextZero=1; // next zero is zero delim (EndOfFrame)

    //  nb backwards loop !
    for (int i = nB-1 ; 0 <= i ; i--)
    {
        if (d[i] == 0)
        {
            dst[i] = nextZero;
            nextZero = 1;
        }
        else
        {
            nextZero++;
        }
    }
    // move to make place to startdelim, first offset and stop delim
    for (int i=nB+1;  2 <= i; i--) 
    {
      dst[i] = dst[i-2];
    }
    dst[nB+2] = 0; // stop delim
    dst[0] = 0; // startdelim
    dst[1] = (char)(nextZero);

    return nB+3;  // new lgt 
}

/*
* take a full cobs package incl start and stop delim
* returns 0 if alles went ok
* the array now holds decoded data shifted to left so d[0] = first databyte
* Returns number of data bytes
*/
int cobsDecode(char * d, char *dst, int nB)
{
    // Full package incl start and stop delim 
    // first copy incl leading start delim and first zero ref and incl term zero
    int chg;
     
    if (nB < 3 || (COBSMAX+3) < nB) // 257: 254 + lead_0+firstCob + term 0
        return -1; // error

    if (d[0] != 0 || d[nB-1] != 0 )
        return -2; // bad pkg format delimiters 
        
    if (d[1] == 0 ) // first zero delim must not be zero
      return -3;

    if (dst == 0)
       dst = d;
       
    chg = d[1]; // first ref to 0

    for (int i=2; i < nB-1; i++)
    {
        dst[i-2] = d[i];  // shift pkg 2 to left
        chg--;
        if (chg == 0)
        {
            chg = d[i];
            dst[i-2] = 0;
        }
    }
    return nB-3;  // decoded lgt
}

 
