/*
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
   vrs 0.91
*/
#include <fp64lib.h>

#define NMEAMAXLGT 100

#define LF 0X0A
#define CR 0x0D
#define GPSOK 0

#define NRTP 2
#define GGA 0
#define GLL 1


// See format for GGA and GLL pkgs in bottom of this file

unsigned char *nmeaTp[] = { "GGA", "GLL", NULL };

// ce data type
typedef struct {
  unsigned char fix, state, cnt, ifaceID;
  int nrSat;
  float hdop, utc, alt;
  float64_t lat64, lon64;
  char rx[NMEAMAXLGT + 2];  // received nmea pkg
} nmeaPkgTp;

// the two interface/GPS
nmeaPkgTp
  rx1 = { .fix = 0, .state = 0, .cnt = 0, .ifaceID = 1, .nrSat = 0, .hdop = 0.0, .utc = 0.0 },
  rx2 = { .fix = 0, .state = 0, .cnt = 0, .ifaceID = 2, .nrSat = 0, .hdop = 0.0, .utc = 0.0 },
  rx3 = { .fix = 0, .state = 0, .cnt = 0, .ifaceID = 2, .nrSat = 0, .hdop = 0.0, .utc = 0.0 };


// strings for holding extracted info from nmea pkg
// for gga pkg
#define GGAELM 20

char
  utcS[GGAELM],
  latS[GGAELM], latSgnS[3], latDS[GGAELM], latFS[GGAELM],
  lonS[GGAELM], lonSgnS[3], lonDS[GGAELM], lonFS[GGAELM],
  fixS[3], hdopS[6], nrSatS[4], altS[6];


// PUX strings for ublox config
/**********************
* blox NEO-6M.txt and 7
**********************/
// disable strings
#define DISGGA "$PUBX,40,GGA,0,0,0,0,0,0*5A\r\n"
#define DISGLL "$PUBX,40,GLL,0,0,0,0,0,0*5C\r\n"
#define DISGSA "$PUBX,40,GSA,0,0,0,0,0,0*4E\r\n"
#define DISGSV "$PUBX,40,GSV,0,0,0,0,0,0*59\r\n"
#define DISRMC "$PUBX,40,RMC,0,0,0,0,0,0*47\r\n"
#define DISVTG "$PUBX,40,VTG,0,0,0,0,0,0*5E\r\n"
#define DISZDA "$PUBX,40,ZDA,0,0,0,0,0,0*44\r\n"

// enable strings
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

// fct forward dcl

int badChkSum(char *tlg);
int notNMEA(char *t, int tp);

// switch off all "known" messages from PUBX

void bloxCfgDisableAll(HardwareSerial &serPort) {
  serPort.print(DISGSV);
  serPort.print(DISGLL);
  serPort.print(DISRMC);
  serPort.print(DISGSA);
  serPort.print(DISVTG);
  serPort.print(DISGGA);
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

  // mm.mmm... i 1/60
  while (0x00 != *(p)) {
    *f = *p;
    f++;
    p++;
  }
  *f = 0x00;
}

// make a dd.dddd  from ddd and mm.mmm mm is 1/60 deg
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
  *dst = 0x00;  // terminate string
  return s;
}


void extractGGL(char *pgg) {
  char *pg = pgg;
  pg = nextComma(pg);  // skip to after next comma
  pg = nextCommaCopy(pg, latS);
  pg = nextCommaCopy(pg, latSgnS);
  pg = nextCommaCopy(pg, lonS);
  pg = nextCommaCopy(pg, lonSgnS);
  pg = nextCommaCopy(pg, utcS);
  pg = nextCommaCopy(pg, fixS);
}


void extractGGA(char *pgg) {
  char *pg = pgg;
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
}

int chkAndDumpGGA(HardwareSerial &Serial,nmeaPkgTp *buf) {
  int sgn;

  if (badChkSum(buf->rx))
    return -1;

  if (notNMEA(buf->rx, GGA))
    return -2;

  extractGGA(buf->rx);

  // utc time, fix,...
  buf->utc = atof(utcS);
  buf->nrSat = atoi(nrSatS);
  buf->fix = atoi(fixS);

  Serial.print(buf->ifaceID);
  Serial.print(" ");
  Serial.print(buf->fix);
  Serial.print(" ");
  Serial.print(buf->utc, 3);
  Serial.print(" ");

  // no fix then stop  here
  if (0 == buf->fix) {
    Serial.println(" ");
    return -3;
  }

  // from here we have a ok GGA pkg we assume
  buf->hdop = atof(hdopS);

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

// Pulling serial line to avoid buffer overflow
// max one char each time passing here
char chkGPS(HardwareSerial &ser, nmeaPkgTp *buf) {
  int r;
  char c;

  if (0 >= ser.available())
    return -1;

  r = ser.read();

  if (0 > r)
    return -2;

  c = (char)r;

  switch (buf->state) {

    case 0:  // waiting for a $
      if ('$' == c) {
        buf->cnt = 0;
        buf->rx[buf->cnt++] = c;
        buf->state = 1;
      }
      break;

    case 1:
      if ('$' == c) {  // reset state 1 to state 0
        buf->cnt = 0;
        buf->rx[buf->cnt++] = c;
        buf->state = 1;
        break;
      }

      // space in buf ? not so reset to start(0)
      if (NMEAMAXLGT < buf->cnt) {
        buf->cnt = 0;  // reset
        buf->state = 0;
        if ('$' == c) {
          buf->rx[buf->cnt++] = c;
          buf->state = 1;
        }
        break;
      }

      buf->rx[buf->cnt++] = c;

      //finito
      if (LF == c) {
        buf->state = 2;
      }
      break;

    case 2:
      break;
    default:;
  }
  return buf->state;
}

// to avoid tx buffer overflow ard buffer is 64 bytes
// this setup has a pos dump to be max 48-49 bytes to two pos dump can overfill buffer
// Running 57k6~ 5k bytes pr sec. So it takes 50/5000  or approx 10 msec to tx a position

void chkSerialTX() {
  if (48 > Serial.availableForWrite()) {
    delay(4);
  }
}


int dumpGGA(nmeaPkgTp *buf) {
  if (2 == buf->state) {  // ready to dump position ?
    chkSerialTX();        // if tx buffer is full then wait a little
    chkAndDumpGGA(Serial,buf);
    buf->state = 0;
  }
  return 0;
}

int dumpAll(nmeaPkgTp *buf) {

  if (2 == buf->state) {  // ready to dump position ?
    chkSerialTX();        // if tx buffer is full then wait a little
    Serial.print(buf->ifaceID);
    Serial.print(" ");
    Serial.print(buf->rx);
    buf->state = 0;
  }
  return 0;
}


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
  delay(500);

  // stop all tx
  bloxCfgDisableAll(Serial1);
  bloxCfgDisableAll(Serial2);

  // enable only for gga packages
  Serial1.print(ENGGA);
  Serial2.print(ENGGA);
  delay(50);
}

// my little non preempt kernel
void loop() {
  if (2 == chkGPS(Serial2, &rx2)) {
    dumpGGA(&rx2);
    //dumpAll(&rx2);
  }

  if (2 == chkGPS(Serial1, &rx1)) {
    dumpGGA(&rx1);
    //dumpAll(&rx1);
  }
}

//----------------------------------

int badChkSum(char *tlg) {
  char c = 0, ch, cl, *pC = tlg;
  int cnt = 0;
  pC++;                 // skip $
  while (*pC != '*') {  // chk end is *
    cnt++;
    if (100 < cnt)
      return 2;
    c ^= *pC;
    pC++;
  }

  // make it visible
  ch = c >> 4;
  cl = c & 0x0F;
  if (ch < 10)
    ch += '0';
  else
    ch += 'A' - 10;

  if (cl < 10)
    cl += '0';
  else
    cl += 'A' - 10;

  pC++;
  if (ch == *pC++ && cl == *pC) {
    return 0;  //
  } else {
    return 1;  // not ok
  }
}

int notNMEA(char *t, int tp) {
  unsigned char *cc = nmeaTp[tp];

  if (NRTP <= tp)
    return -1;

  t++;
  if ('G' != *t) return -2;
  t++;
  if ('P' != *t) return -3;
  t++;
  if (*cc != *t) return -4;
  t++;
  cc++;
  if (*cc != *t) return -5;
  t++;
  cc++;
  if (*cc != *t) return -6;

  return 0;  // ok
}

/*

ID	NAVIGATION SATELLITE SYSTEM RECEIVERS	REGION/COUNTRY
GA	European Global Navigation System (Galileo)	Europe
GB	BeiDou Navigation Satellite System (BDS) 	China
GI	Navigation Indian Constellatiozn (NavIC) 	India
GL	Globalnaya Navigazionnaya Sputnikovaya Sistema (GLONASS)	Russia
GN	Global Navigation Satellite System (GNSS)	*multiple 
GP	Global Positioning System (GPS) 	US
GQ	Quasi-Zenith Satellite System (QZSS) 	Japan

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
  2: Differential GPS fix (DGNSS), SBAS, OmniSTAR VBS, Beacon, RTX in GVBS mode
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
*/
