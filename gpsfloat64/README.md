# A GPS module for NMEA handling

under construction  /Jens

- NMEA 0183 arduino module
- GPGGA and GPGLL packages
- GPVTG on way
- on ublox neo 6m and 7m all tx is
-- by use of ublox propritary PUBX messages out is configurable
- Implementation for Arduino (Mega)
- Tested with two GPS, SW has capability for 3
- Tested with 7h without loosing packages
- Using 64 bit float by use of "fp64lib" works nice
-- 32 bit float is to low in resolution


No guarantee what so ever


JEns

pls see my license in the code


## output in GPGGA mode

```
2 1 1 130635.000 57.01465383333333 9.986309500000001 0.92 11 
1 1 1 130635.000 57.01454416666667 9.986270166666667 0.91 10 
                                                          ^-- int:   nr of sats within range
                                                       ^----- float: DHOP q index . lower eq better -  should be less than 1.0
                                        ^-------------------- float: lat dd.ddddd...
                         ^----------------------------------- float: lon dd.ddddd...
          ^-------------------------------------------------- float: utc time  hhmms.sss
    ^-------------------------------------------------------- int:   pack type  1:GGA, 2: GLL,-- se source
  ^---------------------------------------------------------- int:   fix  1 == fix  0 == nofix
^-------------------------------------------------------------int:   GPS rcv 1,2,3    
```

# Code 4 arduino

``` C

loop ()
{
  /*  JUST FOR SHOW ALL INFO FROM A DEVICE - here Serial2
     if (0 < Serial2.available()) {
     Serial.write((unsigned char)(Serial2.read()));
     }
     return;
   */
   
  char status;
  if (getNmeaPkg (Serial2, &rx2))   // returns true if nmea pkg has been received with correct cheksum
    {                               // can be found in struct rx1 - see code
      // startT = millis();
      dumpNmeaPkg (&rx2);	       // parse packet and dump on Serial - see format above can dump GGA and GLL pkgs
      //Serial.println(), Serial.println(millis() - startT); // just for mea timeconsump: On mega [4;10] msec

      // or dumpRawPkg(&rx2);    // just dump raw packet
    }

  if (getNmeaPkg (Serial1, &rx1))  // same for other iface
    {
      dumpNmeaPkg (&rx1);
      // or dumpRawPkg(&rx1);
    }
}
```

# Config
``` C
void
setup ()
{

  Serial.begin (57600);		// at least 4 times larger than serial 1 and 2 do avoid tx buffer overflow
  Serial1.begin (9600);
  Serial2.begin (9600);

  delay (50);

  while (!Serial);
  while (!Serial1);
  while (!Serial2);
  delay (50);

  // stop all tx
  bloxCfgDisableAll (Serial1); // send commands to both GPS om do not TX anythong
  bloxCfgDisableAll (Serial2);

  // enable only for gga packages
  // 
  Serial1.print (ENGGA);  // enable or FPGGA pkg for GPS on Serial1
  Serial2.print (ENGGA);
  activateNmeaType (GGA); // activate parsning of GPGGA - a only once command - reset to come back to init state
                          // activateNmeaType is for all interfaces iff you have send the activate strings as just above


  // Serial1.print(ENGLL);
  //Serial2.print(ENGLL);
  //Serial2.print(ENVTG);  //  VTG not impl only tracked in parser 

  //Serial1.print(ENVTG);
  //activateNmeaType(GLL);  // for GPL packets

  //activateNmeaType(VTG);  // fo VTG packets  ONLY SKELETON CODE so activatting it do not give anythong
  //activateNmeaType(GLL);
  delay (50);
}
```

# Timing and how it runs

- GPS tx to the serial interface 
- loop take a char from each iface (getNmeaPkg) when looping
- RX buffer is 64 bytes so dont be away too much time
- getNmeaPkg(Serial<ifacenr>,<buffer) returns 1 when a full correct pkg is received
- dumpNmeaPkg convert the packet - read the nmea string(saved in rx2) and save lat,lon etc in rx2 as well
- dumpNmeaPkg takes 4-10 msec



```
void loop()
{
  if (getNmeaPkg (Serial2, &rx2))
    {
      // startT = millis();
      dumpNmeaPkg (&rx2);	
      ...
    }
    if (getNmeaPkg(Serial1, &rx1))
    ...
 ```


NO warranty whatsoever - tested stabel for +8 hours

happy hacking


Jens



 
