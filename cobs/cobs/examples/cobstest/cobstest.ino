#include <cobs.h>

 
void dmpAr(char * d, int l)
{
  Serial.print("hex: ");
  for (int i = 0; i < l; i++) {
    Serial.print(" ");
    Serial.print(d[i], HEX);
  }
  Serial.println("");
}
 
char d[200] = {1, 2, 0, 0, 3, 4, 5, 0, 5, 0, 22}; // 11 bytes data

void tester()
{
int lgt;
  Serial.println(" raw data");
  dmpAr(d,11);
  
  lgt = cobsEncode(d,NULL,11);
  Serial.print("full tlg length "); Serial.println(lgt);
  dmpAr(d,lgt);

  lgt = cobsDecode(d,NULL,lgt);
  Serial.print("decoded tlg length "); Serial.println(lgt);
  dmpAr(d,lgt);
}
 

//  decode encode test
void setup()
{
  Serial.begin(9600);
  delay(1000);
  tester();
}
 


void loop()
{
}

