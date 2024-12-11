


/***
   "THE BEER-WARE LICENSE" (frit efter PHK)
   <jdn@es.aau.dk/jensd@jensd.dk> wrote this file. As long as you
   retain this notice you can do whatever you want
   with this stuff. If we meet some day, and you think
   this stuff is worth it ...
   you can buy me a beer in return :-)
   or if you are real happy then ...
   single malt will be well received :-)

   Use it at your own risk - no warranty
   
   Jens Dalsgaard Nielsen, AAU, Aalborg, Denmark
   
   dec 2024
   vrs 0.92
***/


#include <fp64lib.h>

#define NMEAMAXLGT 100

#define LF 0X0A
#define CR 0x0D
#define GPSOK 0

#define NRTP 2

#define PKGRDY 99
#define PKGEMPTY 0
#define PKGRCV 1

/* See format for GGA and GLL pkgs in bottom of this file
*/



// startdelim is normmaly $ but some weird gps use ! -- in encapsulated msg's
#define STARTDELIM '$'

// must in sync with location on nmeaNames array
#define GGA 1
#define GLL 2
#define VTG 3
char *nmeaNames[] = { "SKALVAERETOM", "GPGGA", "GPGLL", "GPVTG", NULL };

// array is used to activate tlg types
unsigned char nmeaActives[] = { 0, 0, 0, 0, 0 };

typedef struct {
  unsigned char ifaceID, state, fix, gg, cnt, nmeaId;
  int nrSat;
  float hdop, utc, alt;  //s, chkSum;
  char rx[NMEAMAXLGT];   // received nmea pkg
  char nmeaName[8];
  float64_t lat64, lon64;

} nmeaPkgTp;

// the three interface/GPS - you need to do a Serial<x>.begin(9600); for each of them in setup - if you use them
nmeaPkgTp
  rx1 = {
    .ifaceID = 1,
    .state = PKGEMPTY,
    .fix = 0,
  },
  rx2 = {
    .ifaceID = 2,
    .state = PKGEMPTY,
    .fix = 0,
  },
  rx3 = {
    .ifaceID = 2,
    .state = PKGEMPTY,
    .fix = 0,
  };


// strings for holding extracted info from nmea pkg
// for gga pkg
#define GGAELM 20

char
  utcS[GGAELM],
  latS[GGAELM], latSgnS[3], latDS[GGAELM], latFS[GGAELM],
  lonS[GGAELM], lonSgnS[3], lonDS[GGAELM], lonFS[GGAELM],
  fixS[3], hdopS[6], nrSatS[4], altS[6];


// PUX strings for ublox config
// **********************
// * blox NEO-6M.txt and 7
// **********************
// NMEA  disable & enable strings(ublox)
#define DISGGA "$PUBX,40,GGA,0,0,0,0,0,0*5A\r\n"
#define DISGLL "$PUBX,40,GLL,0,0,0,0,0,0*5C\r\n"
#define DISGSA "$PUBX,40,GSA,0,0,0,0,0,0*4E\r\n"
#define DISGSV "$PUBX,40,GSV,0,0,0,0,0,0*59\r\n"
#define DISRMC "$PUBX,40,RMC,0,0,0,0,0,0*47\r\n"
#define DISVTG "$PUBX,40,VTG,0,0,0,0,0,0*5E\r\n"
#define DISZDA "$PUBX,40,ZDA,0,0,0,0,0,0*44\r\n"

#define ENGGA "$PUBX,40,GGA,0,1,0,0,0,0*5B\r\n"
#define ENGLL "$PUBX,40,GLL,0,1,0,0,0,0*5D\r\n"
#define ENGSA "$PUBX,40,GSA,0,1,0,0,0,0*4F\r\n"
#define ENGSV "$PUBX,40,GSV,0,1,0,0,0,0*58\r\n"
#define ENRMC "$PUBX,40,RMC,0,1,0,0,0,0*46\r\n"
#define ENVTG "$PUBX,40,VTG,0,1,0,0,0,0*5F\r\n"
#define ENZDA "$PUBX,40,ZDA,0,1,0,0,0,0*45\r\n"

// TEST STRENGE
unsigned char
  *tlg = "$GPGGA,134830.00,5700.88614,N,00959.19274,E,1,05,2.44,40.8,M,42.4,M,,*65",
  *t2 = "$GPGGA,134826.00,,,,,0,06,12.74,,,,,,*6A";

//-------------------------------------------------------------------------------------

// switch off all "known" messages from PUBX

void bloxCfgDisableAll(HardwareSerial &serPort) {
  serPort.print(DISGSV);
  serPort.print(DISGLL);
  serPort.print(DISRMC);
  serPort.print(DISGSA);
  serPort.print(DISVTG);
  serPort.print(DISGGA);
}

int okChkSum(char *tlg) {
  unsigned char c = 0, ch, cl, *pC = tlg;
  unsigned char cnt = 0;
  pC++;                 // skip $
  while (*pC != '*') {  // chk end is *
    cnt++;
    if (100 < cnt)
      return -2;
    c ^= *pC;
    pC++;
  }

  pC++;  // point to first of two chars in chksum
  if (*pC <= '9')
    ch = *pC - 0x30;  // '0'
  else
    ch = *pC - 0x37;  //(0x37 ='A' -10dec)

  pC++;
  if (*pC <= '9')
    cl = *pC - 0x30;  // '0'
  else
    cl = *pC - 0x37;  //(0x37 ='A' -10dec)


  return (c == ((ch << 4) | (cl & 0x0F)));  // returns 1 is chksum is ok !
}

unsigned char getNmeaId(nmeaPkgTp *buf) {
  unsigned char indx = 1;
  while (NULL != nmeaNames[indx]) {
    if (5 < indx) return 0;
    if (0 == strcmp(buf->nmeaName, nmeaNames[indx])) {
      if (nmeaActives[indx]) {
        return indx;
      } else {
        return 0;
      }
    }
    indx++;
  }
  return 0;  // not found
}

//pick out degr part  ddmm.mmm
//dd: degrees
// mm.mm   minutes  /1m = 1/60 deg

void getDegFrac(char *d, char *f, char *pkg) {
  char *p = pkg;
  // deg part in degr
  while ('.' != *(p + 2)) {
    *d = *p;
    d++;
    p++;
  }
  *d = 0x00;

  // frac part - mm.mmm... i 1/60
  while (0x00 != *(p)) {
    *f = *p;
    f++;
    p++;
  }
  *f = 0x00;
}

// make float 64 bit from .. dd.dddd  from ddd and mm.mmm mm is 1/60 deg
float64_t makeFloat64(char *d, char *f, int sign)  // 2 lat, 4 lon i gga
{
  float64_t tmp;

  tmp = fp64_add(
    fp64_atof(d),
    fp64_div(
      fp64_atof(f),
      fp64_atof("60.0")));

  if (sign == -1)
    return fp64_neg(tmp);
  else
    return tmp;
}

// read nmeas until next separator (,) Returns ref to elem AFTER next comma
char *nextComma(char *s) {
  while ((',' != *s) && ('*' != *s))
    s++;
  s++;  // to first ch in next field
  return s;
}

// as nextComma but do also copy field we are passing through
char *nextCommaCopy(char *s, char *dst) {
  char *ss = dst;
  while ((',' != *s) && ('*' != *s)) {  // * is delimiter of last field bef checksum
    *dst = *s;
    s++;
    dst++;
  }
  s++;
  *dst = 0x00;  // terminate string extracted
  return s;
}


void doLatLon(nmeaPkgTp *buf) {
  char sgn;
  // get sign for lat and lon
  if ('N' == latSgnS[0])
    sgn = 1;
  else
    sgn = -1;
  getDegFrac(latDS, latFS, latS);
  buf->lat64 = makeFloat64(latDS, latFS, sgn);

  if ('E' == lonSgnS[0])
    sgn = 1;
  else
    sgn = -1;
  getDegFrac(lonDS, lonFS, lonS);
  buf->lon64 = makeFloat64(lonDS, lonFS, sgn);
}

void doGLLvars(nmeaPkgTp *buf) {
  char *pg = buf->rx;

  pg = nextComma(pg);  // skip to after next comma

  pg = nextCommaCopy(pg, latS);
  pg = nextCommaCopy(pg, latSgnS);
  pg = nextCommaCopy(pg, lonS);
  pg = nextCommaCopy(pg, lonSgnS);
  pg = nextCommaCopy(pg, utcS);
  pg = nextCommaCopy(pg, fixS);
  // utc time, fix,...
  // utc time, fix,...
  buf->utc = atof(utcS);

  if ('A' == fixS[0])  // A ok V no fix
    buf->fix = 1;
  else
    buf->fix = 0;

  doLatLon(buf);
}
 
void doGGAvars(nmeaPkgTp *buf) {
  char *pg = buf->rx;
  pg = nextComma(pg);  // skip to after next comma


  pg = nextCommaCopy(pg, utcS);
  pg = nextCommaCopy(pg, latS);
  pg = nextCommaCopy(pg, latSgnS);
  pg = nextCommaCopy(pg, lonS);
  pg = nextCommaCopy(pg, lonSgnS);
  pg = nextCommaCopy(pg, fixS);
  pg = nextCommaCopy(pg, nrSatS);
  pg = nextCommaCopy(pg, hdopS);
  pg = nextCommaCopy(pg, altS);

  // utc time, fix,...


  if ('0' == fixS[0])  // A ok V no fix
    buf->fix = 0;
  else
    buf->fix = 1;

  buf->utc = atof(utcS);
  buf->nrSat = atoi(nrSatS);

  buf->hdop = atof(hdopS);
  buf->alt = atof(altS);

  doLatLon(buf);
}

void doVTGvars(nmeaPkgTp *buf) {
  char *pg = buf->rx;
  /* TODO
  not updated
  pg = nextComma(pg);  // skip to after next comma
  pg = nextCommaCopy(pg, utcS);
  pg = nextCommaCopy(pg, latS);
  pg = nextCommaCopy(pg, latSgnS);
  pg = nextCommaCopy(pg, lonS);
  pg = nextCommaCopy(pg, lonSgnS);
  pg = nextCommaCopy(pg, fixS);
  pg = nextCommaCopy(pg, nrSatS);
  pg = nextCommaCopy(pg, hdopS);
  pg = nextCommaCopy(pg, altS);

  // utc time, fix,...
  buf->utc = atof(utcS);
  buf->nrSat = atoi(nrSatS);
  buf->fix = atoi(fixS);
  buf->hdop = atof(hdopS);
  buf->alt = atof(altS);

  doLatLon(buf);
  */
}

// Pulling serial line to avoid buffer overflow
// max one char each time passing here
char getNmeaPkg(HardwareSerial &ser, nmeaPkgTp *buf) {
  int r;
  unsigned char c, retcode = 0;

  if (0 >= ser.available())
    return 0;

  r = ser.read();

  if (0 > r)
    return 0;

  c = (char)r;

  switch (buf->state) {
    case PKGEMPTY:  // waiting for a $
      if (STARTDELIM == c) {
        buf->cnt = 0;
        buf->rx[buf->cnt++] = c;
        buf->state = PKGRCV;
        buf->rx[NMEAMAXLGT - 1] = 0x00;  // term string just in case
      }
      break;

    case PKGRCV:
      if (STARTDELIM == c) {  // reset
        buf->state = PKGEMPTY;
        break;  // restart
      }

      // buffer overflow
      if (NMEAMAXLGT - 2 < buf->cnt) {
        if (STARTDELIM == c) {
          buf->rx[buf->cnt++] = c;
          buf->state = PKGEMPTY;
        }
        break;
      }

      if (0 < buf->cnt && buf->cnt < 6)
        buf->nmeaName[buf->cnt - 1] = c;
      if (5 == buf->cnt) {
        buf->nmeaName[5] = 0x00;
      }

      buf->rx[buf->cnt++] = c;

      //finito
      if (LF == c) {
        buf->rx[buf->cnt] = 0x00;
        buf->state = PKGEMPTY;
        if (okChkSum(buf->rx)) {
          retcode = getNmeaId(buf);  // 1,2,3  - 0 means not found == not supported pkg
          buf->nmeaId = retcode;
        }
      }
      break;
    default:
      // bad bad to come here. we do a rcv reset;
      buf->state = PKGEMPTY;
  }
  return retcode;
}


// to avoid tx buffer overflow ard buffer is 64 bytes
// this setup has a pos dump to be max 48-49 bytes to two pos dump can overfill buffer
// Running 57k6~ 5k bytes pr sec. So it takes 50/5000  or approx 10 msec to tx a position

void chkSerialTX() {
  if (48 > Serial.availableForWrite()) {
    delay(4);
  }
}


void extractDumpGLL(nmeaPkgTp *buf) {

  doGLLvars(buf);

  Serial.print(buf->ifaceID);
  Serial.print(" ");
  Serial.print(buf->fix);
  Serial.print(" ");
  Serial.print(buf->nmeaId);
  Serial.print(" ");
  Serial.print(buf->utc, 3);
  Serial.print(" ");

  // no fix then stop  here
  if (0 == buf->fix) {
    Serial.println(" ");
    return -3;
  }


  Serial.print(fp64_to_string(buf->lat64, 17, 0));
  Serial.print(" ");

  Serial.print(fp64_to_string(buf->lon64, 17, 0));
  Serial.println(" ");
}

int extractDumpGGA(nmeaPkgTp *buf) {
  int sgn;

  doGGAvars(buf);

  Serial.print(buf->ifaceID);
  Serial.print(" ");
  Serial.print(buf->fix);
  Serial.print(" ");
  Serial.print(buf->nmeaId);
  Serial.print(" ");
  Serial.print(buf->utc, 3);
  Serial.print(" ");

  // no fix then stop  here
  if (0 == buf->fix) {
    Serial.println();
    return -3;
  }

  //dump pos
  Serial.print(fp64_to_string(buf->lat64, 17, 0));
  Serial.print(" ");

  Serial.print(fp64_to_string(buf->lon64, 17, 0));
  Serial.print(" ");

  Serial.print(buf->hdop);
  Serial.print(" ");
  Serial.print(buf->nrSat);

  Serial.println(" ");
  return 0;  // ok
}

int activateNmeaType(unsigned char id) {
  if (id < sizeof(nmeaActives)) {
    nmeaActives[id] = 1;
  }
}

int extractDumpVTG(nmeaPkgTp *buf) {
  int sgn;

  //doGVTGvars(buf);
  /*
  Serial.print(buf->ifaceID);
  Serial.print(" ");
  Serial.print(buf->fix);
  Serial.print(" ");
  Serial.print(buf->utc, 3);
  Serial.print(" ");

  // no fix then stop  here
  if (0 == buf->fix) {
    Serial.println();
    return -3;
  }

  //dump pos
  Serial.print(fp64_to_string(buf->lat64, 17, 0));
  Serial.print(" ");

  Serial.print(fp64_to_string(buf->lon64, 17, 0));
  Serial.print(" ");

  Serial.print(buf->hdop);
  Serial.print(" ");
  Serial.print(buf->nrSat);

  Serial.println(" ");
  */
  return 0;  // ok
}

void dumpNmeaPkg(nmeaPkgTp *buf) {
  // we know we have a pkg bq check sum is ok
  char id;
  id = getNmeaId(buf);
  buf->nmeaId = id;
  getNmeaId(buf);
  chkSerialTX();

  switch (id) {
    case GGA:
      extractDumpGGA(buf);
      break;
    case GLL:
      extractDumpGLL(buf);
      break;
    case VTG:
      Serial.println("VTG received");
      break;
      extractDumpVTG(buf);
      break;

    default:
      Serial.print(buf->ifaceID);
      Serial.print(": ");
      Serial.print(buf->rx);
  }
}

int dumpRawPkg(nmeaPkgTp *buf) {

  if (PKGRDY == buf->state) {  // ready to dump position ?
    chkSerialTX();             // if tx buffer is full then wait a little
    Serial.print(buf->ifaceID);
    Serial.print(": ");
    Serial.print(buf->rx);
    buf->state = PKGEMPTY;
  }
  return 0;
}

//-----------------------------------------------------------------

void setup() {

  Serial.begin(57600);  // at least 4 times larger than serial 1 and 2 do avoid tx buffer overflow
  Serial1.begin(9600);
  Serial2.begin(9600);

  delay(50);

  while (!Serial)
    ;
  while (!Serial1)
    ;
  while (!Serial2)
    ;
  delay(50);

  // stop all tx
  bloxCfgDisableAll(Serial1);
  bloxCfgDisableAll(Serial2);

  // enable only for gga packages
  //
  Serial1.print(ENGGA);
  Serial2.print(ENGGA);

  // Serial1.print(ENGLL);
  //Serial2.print(ENGLL);
  //Serial2.print(ENVTG);  // TOT VTGnot impl only tarcked in parser
  //Serial1.print(ENVTG);

  activateNmeaType(GGA);
  //activateNmeaType(GLL);
  //  activateNmeaType(VTG);
  //activateNmeaType(GLL);
  delay(50);
}

//#define TIMETEST

// unsigned startT;

// my little non preempt kernel
void loop() {
  /*
  if (0 < Serial2.available()) {
    Serial.write((unsigned char)(Serial2.read()));
  }
  return;
  */
  char status;
  if (getNmeaPkg(Serial2, &rx2)) {
    // startT = millis();
    dumpNmeaPkg(&rx2);  // all
    //Serial.println(), Serial.println(millis() - startT);

    // or dumpRawPkg(&rx2);
  }

  if (getNmeaPkg(Serial1, &rx1)) {
    dumpNmeaPkg(&rx1);
    // or dumpRawPkg(&rx1);
  }
}

//----------------------------------
//-----------------------------------------------------------------
//-----------------------------------------------------------------
//-----------------------------------------------------------------


/*
#ifdef NEVER

ID NAVIGATION SATELLITE SYSTEM RECEIVERS REGION / COUNTRY GA European Global Navigation System(Galileo)
Europe
  GB BeiDou Navigation Satellite
  System(BDS)
China
  GI Navigation Indian
  Constellatiozn(NavIC)
Indi·a
  GL Globalnaya Navigazionnaya Sputnikovaya
  Sistema(GLONASS)
Russia
    GN Global Navigation Satellite
    System(GNSS)
  * multiple
  GP Global Positioning System(GPS)
US
    GQ Quasi
  - Zenith Satellite System(QZSS) 	Japan



 

ID NAVIGATION SATELLITE SYSTEM RECEIVERS REGION / COUNTRY GA European Global Navigation System(Galileo)
Europe
  GB BeiDou Navigation Satellite
  System(BDS)
China
  GI Navigation Indian
  Constellatiozn(NavIC)
India
  GL Globalnaya Navigazionnaya Sputnikovaya
  Sistema(GLONASS)
Russia
    GN Global Navigation Satellite
    System(GNSS)
  * multiple
  GP Global Positioning System(GPS)
US
    GQ Quasi
  - Zenith Satellite System(QZSS) 	Japan

https://docs.novatel.com/OEM7/Content/Logs/GPGGA.htm

GGA message fields
Field 	Meaning
0 	Message ID $GPGGA
1 	UTC of position fix
2 	Latitude
3 	Direction of latitude:

N: North
S: South
4 	Longitude
5 	Direction of longitude:

E: East
W: West
6 	GPS Quality indicator:
  0: Fix not valid
  1: GPS fix
  2: Differential GPS fix (DGNSS), SB$AS, OmniSTAR VBS, Beacon, RTX in GVBS mode
  3: Not applicable
  4: RTK Fixed, xFill
  5: RTK Float, OmniSTAR XP/HP, Location RTK, RTX
  6: INS Dead reckoning

7 	Number of SVs (space vehicles:sats) in use, range from 00 through to 24+
8 	HDOP - Horizontal dilution of precision.
9 	Orthometric height (MSL reference)
10 	M: unit of measure for orthometric height is meters
11 	Geoid separation
12 	M: geoid separation measured in meters
13 	Age of differential GPS data record, Type 1 or Type 9. Null field when DGPS is not used.
14 	Reference station ID, range 0000 to 4095. A null field when any reference station ID is selected and no corrections are received. See table below for a description of the field values.
15 	The checksum data, always begins with *

---------------------------------------------
GLL GLL
https://receiverhelp.trimble.com/alloy-gnss/en-us/NMEA-0183messages_GLL.html
0 	Message ID $GPGLL
1 	Latitude in dd mm,mmmm format (0-7 decimal places)
2 	Direction of latitude N: North S: South
3 	Longitude in ddd mm,mmmm format (0-7 decimal places)
4 	Direction of longitude E: East W: West
5 	UTC of position in hhmmss.ss format
6 	Status indicator:
  A: Data valid
  V: Data not valid

This value is set to V (Data not valid) for all Mode Indicator values except A (Autonomous) and D (Differential)
7 	The checksum data, always begins with *

Mode indicator:
A: Autonomous mode
D: Differential mode
E: Estimated (dead reckoning) mode
M: Manual input mode
S: Simulator mode
N: Data not valid


---------------------------------
VTG VTG 
https://receiverhelp.trimble.com/alloy-gnss/en-us/NMEA-0183messages_VTG.html

0 	Message ID $GPVTG
1 	Track made good (degrees true)
2 	T: track made good is relative to true north
3 	Track made good (degrees magnetic)
4 	M: track made good is relative to magnetic north
5 	Speed, in knots
6 	N: speed is measured in knots
7 	Speed over ground in kilometers/hour (kph)
8 	K: speed over ground is measured in kph
9 	Mode indicator:
    A: Autonomous mode
    D: Differential mode
    E: Estimated (dead reckoning) mode
    M: Manual Input mode
    S: Simulator mode
    N: Data not valid
10 	The checksum data, always begins with *
#endif
*/
