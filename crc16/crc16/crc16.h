/*****************************************************
* CRC16  package                                     *
* "THE BEER-WARE LICENSE" (frit efter PHK)           *
* <jdn@es.aau.dk> wrote this file. As long as you    *
* retain this notice you can do whatever you want    *
* with this stuff. If we meet some day, and you think*
* this stuff is worth it ...                         *
*  you can buy me a beer in return :-)               *
* or if you are real happy then ...                  *
* single malt will be well received :-)              *
*                                                    *
* Use it at your own risk - no warranty              *
*****************************************************/
#ifndef CRC16
#define CRC16
 
#include <avr/pgmspace.h>  // bq we have crc16 table in prog mem space for saving RAM

 

void set_crc16_pol(int id);

uint16_t calc_crc16(unsigned char *buf,int l, unsigned char *c1, unsigned char *c2);
void add_crc16(unsigned char *buf, int l);
char chk_crc16(unsigned char *buf, int l);
#endif
