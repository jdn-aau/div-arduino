 #include <cobs.h>


// testing on MEGA

// short circuti tx and rx on serial 1 :-)
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



void txPk(char *d, int lgt)
{
  while (lgt --)
    Serial1.print(*(d++));
}

int rxPkDelim()
{
  char b;
  Serial1.setTimeout(500);
   // try to read 1 byte
  if (0 < Serial1.readBytes(&b, 1)) {
    if (b == 0)
      return 1;
    else
      return 0;
  }
  return 0;
}

int rxPk(char *b, int maxL)
{
  int l = 0;
  while (1) {
    Serial1.setTimeout(3 + 2); //  9600 baud we set max 3 msec pr byte
    // so we assume sender do it fast
    // try to read one byte
    if (0 < Serial1.readBytes(b, 1)) {
      l++;
      if (*b == 0) {
        return l + 1; // found eop +1 bq we have not read first zero
        // but first zero is manual put'd in buffer
      }
    }
    else {
      return -l;  // timout, ret value eq neg val off rcvd data
    }

    if (maxL <= l) {
      // max found
      return -l; // max exceeded you can find out by cmp to maxL
    }
    b++; // incr data pointer where you put rcv'd bytes
  }
}


void tester()
{
  int lgt;
  Serial.println(" raw data");
  dmpAr(d, 11);

  lgt = cobsEncode(d,NULL, 11);
  Serial.print("full tlg length "); Serial.println(lgt);
  dmpAr(d, lgt);
  // tx
  txPk(d, lgt);
  Serial.println("after tx");
  //rx
  while (0 == rxPkDelim())
    Serial.print(".");

  Serial.println("found start");
  d[0] = 0;
  lgt = rxPk(d + 1, 25);
  // chk lgt osv nb remember that lgt is excl term 0 so
  Serial.println("after rxPk - lgt: ");
  Serial.println(lgt);
  dmpAr(d, lgt);
  if (d[lgt] == 0)
  {
    // delim ok
  }

  lgt = cobsDecode(d, NULL, lgt);
  Serial.print("decoded tlg length "); Serial.println(lgt);
  dmpAr(d, lgt);
}

//  decode encode test
void setup()
{
  Serial.begin(9600);
  Serial1.begin(9600);
  delay(1000);
  tester();
}



void loop()
{
}

