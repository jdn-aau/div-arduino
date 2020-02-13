#include <crc16.h>
#define BUFSIZE 20

// two last bytes for crc16
unsigned char inbuf[BUFSIZE] = {
  0xff, 0x23, 0xc0, 0x21, 0x01, 0xc9, 0x00, 0x0c, 0x01, 0x04,
  0x05, 0xdc, 0x28,  0x02, 0x07, 0xc9, 0x00, 0x0c, 0x0, 0x00
};

void setup()
{
  unsigned char c1=0, c2=0;
  Serial.begin(9600);

  delay(2000);
  Serial.println("start");
  
  add_crc16(inbuf, BUFSIZE);  // NB we use the two last bytes for CRC 

  calc_crc16(inbuf, BUFSIZE - 2, &c1, &c2); // or maybe more straight - the same 

  Serial.println(c1);
  Serial.println(inbuf[BUFSIZE-2]);

  if (c1 == inbuf[BUFSIZE - 2])
    Serial.println("first ok");
  else
    Serial.println("first bad");
  if (c2 == inbuf[BUFSIZE - 1])
    Serial.println("scnd ok");
  else
    Serial.println("scnd bad");

  if (0 == chk_crc16(inbuf, BUFSIZE))  // lgt of data incl 2 byte CRC
    Serial.print("crc16 ok");
  else
    Serial.print("crc16  not ok");

}

void loop()
{
}
 
