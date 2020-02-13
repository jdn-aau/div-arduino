
/* 
* COBS  - Consistent Overhead Byte Stuffing
* idea from  https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing 
*
* Limitations
* max lenght of userdata is 254 (COBSMAX)
*
* Jens Dalsgaard Nielsen (C) - beerlicense 
*/

#ifndef COBS
#define COBS
 
#define COBSMAX 254

int cobsEncode(char *d, char *dst, int nB);
int cobsDecode(char * d, char *dst, int nB);

#endif

/* eof */
