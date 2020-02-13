#include <dumpbytes.h> 

void setup()
{
  Serial.begin(9600);
}

long x = 0x12345678;
unsigned int xx = 0x0120;

void loop()
{
Serial.println(">>>>>>>>>>>");
  for (int i = 1; i <= 4 ; i++) {
    Serial.println("Little Endian so LSB first - as memory layouot");
    dumpBytes((void *)(&x), i,'h');  // h/H  hex
    Serial.println("");
    dumpBytes((void *)(&x), i,'b');  // b/B  bin
    Serial.println("");
    dumpBytes((void *)(&x), i,'a');  //  a/A  ascii /.


    Serial.println("");
    Serial.println("Reverse so M endian first");
    dumpBytesRev((void *)(&x), i,'h');
    Serial.println("");
    dumpBytesRev((void *)(&x), i,'b');
    Serial.println("");
    dumpBytesRev((void *)(&x), i,'a');
     
    
    Serial.println("\n\n");
  }

  delay(1000);


}
