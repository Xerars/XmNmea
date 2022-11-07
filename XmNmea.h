/*
 SPDX-License-Identifier: MIT
 
 XmNmea 1.0.0
 Copyright (c) 2022 - 2022 William Xiao
 
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
*/

#ifndef __XMNMEA_H__    //__XMNMEA_H__
#define __XMNMEA_H__    //__XMNMEA_H__

//Macro Declare

//Standard Character
#define NMEA_START      '$'           //Nmea Start Character
#define NMEA_CHKSTART   '*'           //Nmea Check Sum Start
#define NMEA_END        '\n'          //Nmea End Character
#define NMEA_END2       '\r'          //Nmea End Character "<CR><LF>"
#define NMEA_SEP        ','           //Nmea Seperate Character

//Standard NMEA 0183
#define NMEA_GGA        "GGA"         //GGA Title
#define NMEA_GSA        "GSA"         //GSA Title
#define NMEA_GSV        "GSV"         //GSV Title
#define NMEA_RMC        "RMC"         //RMC Title
#define NMEA_VTG        "VTG"         //VTG Title
#define NMEA_GLL        "GLL"         //GLL Title
#define NMEA_ZDA        "ZDA"         //ZDA Title

//Max NMEA Data Size
#define GGA_MAXLEN      72            //GGA Max Length
#define GSA_MAXLEN      65            //GSA Max Length
#define GSV_MAXLEN      210           //GSV Max Length
#define RMC_MAXLEN      70            //RMC Max Length
#define VTG_MAXLEN      34            //VTG Max Length
#define GLL_MAXLEN      47            //GLL Max Length
#define ZDA_MAXLEN      37            //ZDA Max Length

#define NMEA_TYPELEN    5             //NMEA Type Length(With GP)
#define NMEA_TYPELEN2   3             //NMEA Type Length(Without GP)
#define NMEA_PRELEN     1             //NMEA Ignore $ Start Length

#define GPS_MAXBUFFER   255           //GPS Max Buffer



//Enum Declare

//Struct Declare

//GGA:Global Positioning System Fix Data.Time,Position and Fix Related Data for a GPS Receiver
typedef struct _GGA
{
  char Tag[5];              //<00>:
  float UTCTime;            //<01>:
  float Latitude;           //<02>:Latitude(DDMM.MMMM)
  char LatiMark;            //<03>:Latitude Mark('N'/'S')
  float Longitude;          //<04>:Longitude(DDDMM.MMMM)
  char LongMark;            //<05>:Longitude Mark('E'/'W')
  int GPSQuality;           //<06>:GPS Quality
  //(0:Fix not Valid/1:GPS Fix/2:Differential/3:Not Applicable/4:RTK Fixed/5:RTK Float/6:Ins Dead)
  int SatelliateNum;        //<07>:Number of SV(00~12)
  float HDOP;               //<08>:HDOP(0.5~99.9)
  float Altitude;           //<09>:Orthometric Height(-9999.9~99999.9)
  char AltitudeUnit;        //<10>:Orthometric Height Unit(M)
  float GeoidHeight;        //<11>:Geoid Separation
  char GeoidHightUnit;      //<12>:Geoid Separation Unit(M)
  float DiffTime;           //<13>:Differential Time
  int DiffID;               //<14>:Differential Station ID Number(0000~1023)
} GGA;


//GSA:DOP and Active Satellites
typedef struct _GSA
{
  char Tag[5];              //<00>:$GPGSA
  char PosMode;             //<01>:Select Mode(A:Auto/M:Manual)
  int PosType;              //<02>:Mode(1:UnPosition/2:Auto Select/3:Manual Select)
  int PRNCode[12];          //<03-14>:1~12 PRN Code(GPS:01~32 / SBAS:33~64 / GLONASS:64+ / 00)
  float PDOP;               //<15>:PDOP(0.5~99.9)
  float HDOP;               //<16>:HFOP(0.5~99.9)
  float VDOP;               //<17>:VDOP(0.5~99.9)
} GSA;


//GSV:Satellites in View
typedef struct _GSV
{
  char Tag[5];              //<00>:$GPGSV
  int TotalMsg;             //<01>:Total GSV Sentence(1~3)
  int MsgNum;               //<02>:Message Numeric in Sentence(1~3)
  int SatelliteNum;         //<03>:Satellite Numeric(0~12)
  //Field Start(4 Field)
  int PRNCode[3][4];        //<04>:PRN Code(01~32)
  int ElevationAngle[3][4]; //<05>:Satellite Elevation Angle(00~90)
  int AzimuthAngle[3][4];   //<06>:Satellite Azimth Angle(00~90)
  //Field End
  int SNRatio[3][12];       //<07>:SN Ratio(00~99) dbHz
} GSV;


//RMC:Recommanded Minimum Navigation Information
typedef struct _RMC
{
  char Tag[5];              //<00>:$GPRMC
  float UTCTime;            //<01>:UTC Time(HHMMSS.SSS)
  char Status;              //<02>:Status(A:Data Valid/V:Data InValid)
  float Latitude;           //<03>:Latitude(DDMM.MMMM)
  char LatiMark;            //<04>:Latitude Mark('N'/'S')
  float Longitude;          //<05>:Longitude(DDDMM.MMMM)
  char LongMark;            //<06>:Longitude Mark('E'/'W')
  float Speed;              //<07>:Speed Over Ground(Knots)
  float TrackDeg;           //<08>:Track Degree
  int UTCDate;              //<09>:UTC Date(DDMMYY)
  float MagneticAngel;      //<10>:Magnetic Angel(000-180)
  char MagneticDirect;      //<11>:Magnetic Direction
  char Mode;                //<12>:Mode(A:Auto/D:Diff/E:Estimate:N:None)
} RMC;


//VTG:Trake Mode Good and Ground Speed
typedef struct _VTG
{
  char Tag[5];              //<00>:$GPVTG
  float TrueDegree;         //<01>:True Angle Degree(000~359)
  char TrueSys;             //<02>:True Reference System(T)
  float MagneticDegree;     //<03>:Magnetic Angle Degree(000~359)
  char MagneticSys;         //<04>:Magnetic Referenct System(M)
  float SpeedKnot;          //<05>:Speed and Unit is Knot(0.00)
  char SpeedKnotUnit;       //<06>:Speed Unit(N:Knots)
  float SpeedKm;            //<07>:Speed and Unit is Kilometer(0.00)
  char SpeedKmUnit;         //<08>:Speed Unit(K:Km)
} VTG;


//GLL:Geographic Position-Latitude/Longitude
typedef struct _GLL
{
  char Tag[5];              //<00>:$GPGLL
  float Latitude;           //<01>:Latitude(DDMM.MMMM)
  char LatiMark;            //<02>:Latitude Mark('N'/'S')
  float Longitude;          //<03>:Longitude(DDDMM.MMMM)
  char LongMark;            //<04>:Longitude Mark('E'/'W')
  float UTCTime;            //<05>:UTC Time(HHMMSS.SSS)
  char Status;              //<06>:Status(A:Data Valid/V:Data InValid)
} GLL;


//ZDA:Time and Date-Day,Month,Year and Local Time Zone
typedef struct _ZDA
{
  char Tag[5];              //<00>:$GPZDA
  float UTCTime;            //<01>:UTC Time(HHMMSS.SSS)
  int Day;                  //<02>:Day(01~31)
  int Month;                //<03>:Month(01~12)
  int Year;                 //<04>:Year
  int HourZoneOffset;       //<05>:Local Time Zone Offset from GMT Hour(00~+/-13)
  int MinuteZoneOffset;     //<06>:Local Time Zone Offset from GMT Minute(00~59)
} ZDA;


typedef struct _NMEA
{
  GGA* pGGA;                //GGA Sentence
  GSA* pGSA;                //GSA Sentence
  GSV* pGSV;                //GSV Sentence
  RMC* pRMC;                //RMC Sentence
  VTG* pVTG;                //VTG Sentence
  GLL* pGLL;                //GLL Senetnce
  ZDA* pZDA;                //ZDA Sentence
} NMEA;



//Prototype Declare

//Initial
void NMEAInit(void);
//Get Data
NMEA* GetNMEA(void);

//Input Tool
void XmNmeaBufferGet(const char c);
void XmNmeaParse(const char* data);

//Parser Tool
GGA* XmGGAParse(const char* data);
GSA* XmGSAParse(const char* data);
GSV* XmGSVParse(const char* data);
RMC* XmRMCParse(const char* data);
VTG* XmVTGParse(const char* data);
GLL* XmGLLParse(const char* data);
ZDA* XmZDAParse(const char* data);

//Conversion Tool
int ContentBetween(const char* src,char* dst,int From);
int Str2Int(const char* str);
double Str2Double(const char* str);

#endif    //__XMNMEA_H__