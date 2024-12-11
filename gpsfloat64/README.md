# A GPS module for NMEA handling

- NMEA 0183
- GPGGA packages
- on ublox neo 6m and 7m all tx is disabled except for GGA 
-- by use of ublox propritary PUBX messages
- Implementation for Arduino (Mega)
- Tested with two GPS, SW has capability for 3
- Tested with 7h without loosing packages
- Using 64 bit float by use of "fp64lib" works nice
-- 32 bit float is to low in resolution


No guarantee what so ever


JEns

pls see my license in the code


## output in GPGGA mode

2 1 1 130635.000 57.01465383333333 9.986309500000001 0.92 11 
1 1 1 130635.000 57.01454416666667 9.986270166666667 0.91 10 
                                                          ^-- int:   nr of sats within range
                                                       ^----- float: DHOP q index . lower eq better -  should be less than 1.0
                                        ^-------------------- float: lat dd.ddddd...
                         ^----------------------------------- float: lon dd.ddddd...
          ^-------------------------------------------------- float: utc time  hhmms.sss
    ^-------------------------------------------------------- int:   pack type  1:GGA, 2: GLL,-- se source
  ^---------------------------------------------------------- int:  fix  1 == fix  0 == nofix
^-------------------------------------------------------------int:  GPS rcv 1,2,3    

# Code 4 arduino

``` C

loop ()
{
  /*
     if (0 < Serial2.available()) {
     Serial.write((unsigned char)(Serial2.read()));
     }
     return;
   */
  char status;
  if (getNmeaPkg (Serial2, &rx2))
    {
      // startT = millis();
      dumpNmeaPkg (&rx2);	// all
      //Serial.println(), Serial.println(millis() - startT);

      // or dumpRawPkg(&rx2);
    }

  if (getNmeaPkg (Serial1, &rx1))
    {
      dumpNmeaPkg (&rx1);
      // or dumpRawPkg(&rx1);
    }
}
```
