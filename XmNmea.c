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

#include "XmNmea.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

//Macro Declare
#define TRUE        1         //True Condition
#define FALSE       0         //False Condition

#define XMERROR     1         //Error Condition
#define XMSUCCESS   0         //Success Condition

#ifdef DEBUG
  #define XMDEBUG   printf
#else
  #define XMDEBUG
#endif


//Prototype Declare



//Private Variable

//Buffer to Save Data Read from UART
static int CurIdx=0;
static char GPSBuffer[GPS_MAXBUFFER]={0};   //GPSBuffer[128]
NMEA* GPS;


//Function:NMEAInit
//Descript:NMEA Initial and Alloc Memory for NMEA Structure
void NMEAInit(void)
{
  //Alloc Memory
  GPS=(NMEA*)malloc(sizeof(NMEA));
  GPS->pGGA=(GGA*)malloc(sizeof(GGA));
  GPS->pGSA=(GSA*)malloc(sizeof(GSA));
  GPS->pGSV=(GSV*)malloc(sizeof(GSV));
  GPS->pRMC=(RMC*)malloc(sizeof(RMC));
  GPS->pVTG=(VTG*)malloc(sizeof(VTG));
  GPS->pGLL=(GLL*)malloc(sizeof(GLL));
  GPS->pZDA=(ZDA*)malloc(sizeof(ZDA));

  //Clean GPS Buffer
  for(int i=0;i<GPS_MAXBUFFER;i++)
    GPSBuffer[i]=0;
  //memset(GPSBuffer,0,sizeof(GPSBuffer));
}

//Function:GetNMEA
//Descript:Get Global NMEA Value
NMEA* GetNMEA(void)
{
  return GPS;
}


//Function:XmNmeaBufferGet
//Descript:Get NMEA Buffer By Character(Usually Use in Uart)
void XmNmeaBufferGet(const char c)
{
  //Ignore Char When the Buffer Empty and the Char is not Start Char
  if((CurIdx==0) && (c!=NMEA_START))
    return;

  //Buffer is Full and can not Parse it. Regard it as Other Protocal
  if(CurIdx >= GPS_MAXBUFFER)   //over 255
  {
    CurIdx=0;
    return;
  }

  //Append Data to Global
  GPSBuffer[CurIdx++]=c;

  //At the End of Data,Parse it
  if(c==NMEA_END)   //Equal End Character
  {
    //Check Prefix and Suffix Content
    if(GPSBuffer[0]==NMEA_START)    //Check '$'
      if((GPSBuffer[CurIdx-2]==NMEA_END2) && (GPSBuffer[CurIdx-1]==NMEA_END))   //Check '\r' and '\n'
        XmNmeaParse(GPSBuffer);
    CurIdx=0;
  }
}

//Function:XmNmeaParse
//Descript:NMEA Parser Selector
void XmNmeaParse(const char* data)
{
  int i;
  char* type=(char*)malloc(NMEA_TYPELEN2);     //Alloc Memory for NMEA Type
  
  //memcpy(type,data+NMEA_PRELEN,NMEA_TYPELEN); //Memcpy cost too much Time
  for(i=NMEA_PRELEN+2;i<(NMEA_PRELEN+NMEA_TYPELEN);i++)   //+2 Ignore GP
    type[i-3]=data[i];

  type[i-3]='\0';
  XMDEBUG("%s\n",type);

  if(!strcmp(type,NMEA_GGA))       //Parse GGA
  {
    GGA* pGGA=XmGGAParse(data);
    memcpy(GPS->pGGA,pGGA,sizeof(GGA));
    free(pGGA);
  }
  else if(!strcmp(type,NMEA_GSA))  //Parse GSA
  {
    GSA* pGSA=XmGSAParse(data);
    memcpy(GPS->pGSA,pGSA,sizeof(GSA));
    free(pGSA);
  }
  else if(!strcmp(type,NMEA_GSV))  //Parse GSV
  {
    GSV* pGSV=XmGSVParse(data);
    memcpy(GPS->pGSV,pGSV,sizeof(GSV));
    free(pGSV);
  }
  else if(!strcmp(type,NMEA_RMC))  //Parse RMC
  {
    RMC* pRMC=XmRMCParse(data);
    memcpy(GPS->pRMC,pRMC,sizeof(RMC));
    free(pRMC);
  }
  else if(!strcmp(type,NMEA_VTG))  //Parse VTG
  {
    VTG* pVTG=XmVTGParse(data);
    memcpy(GPS->pVTG,pVTG,sizeof(VTG));
    free(pVTG);
  }
  else if(!strcmp(type,NMEA_GLL))  //Parse GLL
  {
    GLL* pGLL=XmGLLParse(data);
    memcpy(GPS->pGLL,pGLL,sizeof(GLL));
    free(pGLL);
  }
  else if(!strcmp(type,NMEA_ZDA))  //Parse ZDA
  {
    ZDA* pZDA=XmZDAParse(data);
    memcpy(GPS->pZDA,pZDA,sizeof(ZDA));
    free(pZDA);
  }
  free(type);
}

//Function:XmGGAParse
//Descript:NMEA-0183 GGA Sentence Parse
GGA* XmGGAParse(const char* data)
{
  //Global Positioning System Fix Data.Time,Position and Fix Related Data for a GPS Receiver
  //$GPGGA,172814.000,3723.4658,N,12202.2695,W,2,6,1.2,18.893,M,-25.669,M,2.0 0031*4F
  //$GPGGA,092204.999,4250.5589,S,14718.5084,E,1,04,24.4,12.2,M,19.7,M,,0000*1F
  //$GPGGA,<1>,<2>,<3>,<4>,<5>,<6>,<7>,<8>,<9>,<10>,<11>,<12>,<13>,<14>,<15>,<CR>,<LF>
  //<1>:UTC Time(HHMMSS.SSS)
  //<2><3>:Latitude(DDMM.MMMM)   / 'N' or 'S'
  //<4><5>:Longitude(DDDMM.MMMM) / 'E' or 'W'
  //<6>:GPS Quality(0:Fix not Valid/1:GPS Fix/2:Differential/3:Not Applicable/4:RTK Fixed/5:RTK Float/6:Ins Dead)
  //<7>:Number of satellites in view(00~12)
  //<8>HDOP(0.5~99.9)
  //<9><10>:Orthometric Height(-9999.9~99999.9)(M)
  //<11><12>:Geoid Separation(M)
  //<13>:Differential Time
  //<14>:Differential Station ID Number(0000~1023)
  //<15>:CheckSum
  int ContentSize;
  int i;

  XMDEBUG("%s\n",data);
  GGA* pGGA=(GGA*)malloc(sizeof(GGA));      //Alloc Memory with GGA Struct
  char* tmp=(char*)malloc(GGA_MAXLEN);      //Alloc Memory with GGA Max Size

  //<0>
  ContentSize=ContentBetween(data,tmp,0); //Get <0>
  for(i=0;i<ContentSize;i++)
    pGGA->Tag[i]=tmp[i];      //Copy Tag
  pGGA->Tag[i]='\0';
  XMDEBUG("[%s] ",pGGA->Tag);
  //<1>
  ContentSize=ContentBetween(data,tmp,1); //Get <1>
  pGGA->UTCTime=ContentSize>0?Str2Double(tmp):0;
  XMDEBUG("[%3.3f] ",pGGA->UTCTime);
  //<2>
  ContentSize=ContentBetween(data,tmp,2); //Get <2>
  pGGA->Latitude=ContentSize>0?Str2Double(tmp):0;
  XMDEBUG("[%4.4f] ",pGGA->Latitude);
  //<3>
  ContentSize=ContentBetween(data,tmp,3); //Get <3>
  pGGA->LatiMark=ContentSize>0?tmp[0]:'0';
  XMDEBUG("[%c] ",pGGA->LatiMark);
  //<4>
  ContentSize=ContentBetween(data,tmp,4); //Get <4>
  pGGA->Longitude=ContentSize>0?Str2Double(tmp):0;
  XMDEBUG("[%5.4f] ",pGGA->Longitude);
  //<5>
  ContentSize=ContentBetween(data,tmp,5); //Get <5>
  pGGA->LongMark=ContentSize>0?tmp[0]:'0';
  XMDEBUG("[%c] ",pGGA->LongMark);
  //<6>
  ContentSize=ContentBetween(data,tmp,6); //Get <6>
  pGGA->GPSQuality=ContentSize>0?Str2Int(tmp):0;  //Fix Not Valid
  XMDEBUG("[%d] ",pGGA->GPSQuality);
  //<7>
  ContentSize=ContentBetween(data,tmp,7); //Get <7>
  pGGA->SatelliateNum=ContentSize>0?Str2Int(tmp):0;     //00
  XMDEBUG("[%d] ",pGGA->SatelliateNum);
  //<8>
  ContentSize=ContentBetween(data,tmp,8); //Get <8>
  pGGA->HDOP=ContentSize>0?Str2Double(tmp):0;
  XMDEBUG("[%2.1f] ",pGGA->HDOP);
  //<9>
  ContentSize=ContentBetween(data,tmp,9); //Get <9>
  pGGA->Altitude=ContentSize>0?Str2Double(tmp):0;
  XMDEBUG("[%2.1f] ",pGGA->Altitude);
  //<10>
  ContentSize=ContentBetween(data,tmp,10); //Get <10>
  pGGA->AltitudeUnit=ContentSize>0?tmp[0]:'M';  //InValid
  XMDEBUG("[%c] ",pGGA->AltitudeUnit);
  //<11>
  ContentSize=ContentBetween(data,tmp,11); //Get <11>
  pGGA->GeoidHeight=ContentSize>0?Str2Double(tmp):0;
  XMDEBUG("[%5.1f] ",pGGA->GeoidHeight);
  //<12>
  ContentSize=ContentBetween(data,tmp,12); //Get <12>
  pGGA->GeoidHightUnit=ContentSize>0?tmp[0]:'M';
  XMDEBUG("[%c] ",pGGA->GeoidHightUnit);
  //<13>
  ContentSize=ContentBetween(data,tmp,13); //Get <13>
  pGGA->DiffTime=ContentSize>0?Str2Double(tmp):0;
  XMDEBUG("[%2.1f] ",pGGA->DiffTime);
  //<14>
  ContentSize=ContentBetween(data,tmp,14); //Get <14>
  pGGA->DiffID=ContentSize>0?Str2Int(tmp):0;  //InValid
  XMDEBUG("[%d] ",pGGA->DiffID);
  free(tmp);
  return pGGA;
}

//Function:XmGSAParse
//Descript:NMEA-0183 GSA Sentence Parse
GSA* XmGSAParse(const char* data)
{
  //GPS DOP and Active Satellites
  //$GNGSA,A,3,21,5,29,25,12,10,26,2,,,,,1.2,0.7,1.0*27
  //$GNGSA,A,3,65,67,80,81,82,88,66,,,,,,1.2,0.7,1.0*20
  //$GNGSA,<1>,<2>,<3>,<4>,<5>,<6>,<7>,<8>,<9>,<10>,<11>,<12>,<13>,<14>,<15>,<16>,<17>,<18>,<CR>,<LF>
  //<1>:Select Mode(A:Auto/M:Manual)
  //<2>:Mode(1:UnPosition/2:Auto Select/3:Manual Select)
  //<3>~<14>:<03-14>:1~12 PRN Code(GPS:01~32 / SBAS:33~64 / GLONASS:64+ / 00)
  //<15>:PDOP:(0.5~99.9)
  //<16>:HDOP:(0.5~99.9)
  //<17>:VDOP:(0.5~99.9)
  //<18>:CheckSum
  int ContentSize;
  int i;

  XMDEBUG("%s\n",data);
  GSA* pGSA=(GSA*)malloc(sizeof(GSA));      //Alloc Memory with GSA Struct
  char* tmp=(char*)malloc(GSA_MAXLEN);      //Alloc Memory with GSA Max Size

  //<0>
  ContentSize=ContentBetween(data,tmp,0); //Get <0>
  for(i=0;i<ContentSize;i++)
    pGSA->Tag[i]=tmp[i];      //Copy Tag
  pGSA->Tag[i]='\0';
  XMDEBUG("[%s] ",pGSA->Tag);
  //<1>
  ContentSize=ContentBetween(data,tmp,1); //Get <2>
  pGSA->PosMode=ContentSize>0?tmp[0]:'0';  //InValid
  XMDEBUG("[%c] ",pGSA->PosMode);
  //<2>
  ContentSize=ContentBetween(data,tmp,2); //Get <2>
  pGSA->PosType=ContentSize>0?Str2Int(tmp):1;  //InValid
  XMDEBUG("[%d] ",pGSA->PosType);
  //<3>~<14>
  for(int i=0;i<12;i++)
  {
    ContentSize=ContentBetween(data,tmp,i+3);
    pGSA->PRNCode[i]=ContentSize>0?Str2Int(tmp):0;
    XMDEBUG("[%d] ",pGSA->PRNCode[i]);
  }
  //<15>
  ContentSize=ContentBetween(data,tmp,15); //Get <15>
  pGSA->PDOP=ContentSize>0?Str2Double(tmp):0;
  XMDEBUG("[%2.1f] ",pGSA->PDOP);
  //<16>
  ContentSize=ContentBetween(data,tmp,16); //Get <16>
  pGSA->HDOP=ContentSize>0?Str2Double(tmp):0;
  XMDEBUG("[%2.1f] ",pGSA->HDOP);
  //<17>
  ContentSize=ContentBetween(data,tmp,17); //Get <17>
  pGSA->VDOP=ContentSize>0?Str2Double(tmp):0;
  XMDEBUG("[%2.1f] ",pGSA->VDOP);
  free(tmp);
  return pGSA;
}

//Function:XmGSVParse
//Descript:NMEA-0183 GSV Sentence Parse
GSV* XmGSVParse(const char* data)
{
  //Satellites in View
  //$GPGSV,8,1,25,21,44,141,47,15,14,049,44,6,31,255,46,3,25,280,44*75
  //$GPGSV,8,2,25,18,61,057,48,22,68,320,52,27,34,268,47,24,32,076,45*76
  //$GPGSV,8,3,25,14,51,214,49,19,23,308,46*7E
  //$GPGSV,<1>,<2>,<3>,(<4>,<5>,<6>,<7>)...,<CR>,<LF>
  //<1>:Total GSV Sentence(1~3)
  //<2>:Current in Total GSV Sentence(1~3)
  //<3>:Satellite Numeric(0~12)
  //<4>:PRN Code(01~32)
  //<5>:Satellite Elevation Angle(00~90)
  //<6>:Satellite Azimth Angle(00~90)
  //<7>:SN Ratio(00~99) dbHz

  int ContentSize;
  int i;

  XMDEBUG("%s\n",data);
  GSV* pGSV=(GSV*)malloc(sizeof(GSV));      //Alloc Memory with GSV Struct
  char* tmp=(char*)malloc(GSV_MAXLEN);      //Alloc Memory with GSV Max Size

  //<0>
  ContentSize=ContentBetween(data,tmp,0); //Get <0>
  for(i=0;i<ContentSize;i++)
    pGSV->Tag[i]=tmp[i];      //Copy Tag
  pGSV->Tag[i]='\0';
  XMDEBUG("[%s] ",pGSV->Tag);
  //<1>
  ContentSize=ContentBetween(data,tmp,1); //Get <1>
  pGSV->TotalMsg=ContentSize>0?Str2Int(tmp):0;
  XMDEBUG("[%d] ",pGSV->TotalMsg);
  //<2>
  ContentSize=ContentBetween(data,tmp,2); //Get <2>
  pGSV->MsgNum=ContentSize>0?Str2Int(tmp):0;
  XMDEBUG("[%d] ",pGSV->MsgNum);
  //<3>
  ContentSize=ContentBetween(data,tmp,3); //Get <3>
  pGSV->SatelliteNum=ContentSize>0?Str2Int(tmp):0;
  XMDEBUG("[%d] ",pGSV->SatelliteNum);

  int MsgNum=(pGSV->MsgNum)-1;
  //<4>..<7>
  for(i=1;i<=4;i++)
  {
    //<4>
    ContentSize=ContentBetween(data,tmp,4*i);     //Get <4><8><12><16>
    pGSV->PRNCode[MsgNum][i*4]=ContentSize>0?Str2Int(tmp):0;
    XMDEBUG("[%d] ",pGSV->PRNCode[MsgNum][i*4]);
    //<5>
    ContentSize=ContentBetween(data,tmp,(4*i)+1); //Get <5><9><13><17>
    pGSV->ElevationAngle[MsgNum][i*4+1]=ContentSize>0?Str2Int(tmp):0;
    XMDEBUG("[%d] ",pGSV->ElevationAngle[MsgNum][i*4+1]);
    //<6>
    ContentSize=ContentBetween(data,tmp,(4*i)+2); //Get <6><10><14><18>
    pGSV->AzimuthAngle[MsgNum][i*4+2]=ContentSize>0?Str2Int(tmp):0;
    XMDEBUG("[%d] ",pGSV->AzimuthAngle[MsgNum][i*4+2]);
    //<7>
    ContentSize=ContentBetween(data,tmp,(4*i)+3); //Get <7><11><15><19>
    pGSV->SNRatio[MsgNum][i*4+3]=ContentSize>0?Str2Int(tmp):0;
    XMDEBUG("[%d] ",pGSV->SNRatio[MsgNum][i*4+3]);
  }
  free(tmp);
  return pGSV;
}

//Function:XmRMCParse
//Descript:NMEA-0183 GPRMC Sentenct Parse
RMC* XmRMCParse(const char* data)
{
  //Position,Velocity and Time
  //$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
  //$GPRMC,<1>,<2>,<3>,<4>,<5>,<6>,<7>,<8>,<9>,<10>,<11>,<12>,<13>,<CR>,<LF>
  //<1>:UTC Time(HHMMSS.SSS)
  //<2>:Status(A:Data Valid / V:Data InValid)
  //<3><4>:Latitude(DDMM.MMMM)   / 'N' or 'S'
  //<5><6>:Longitude(DDDMM.MMMM) / 'E' or 'W'
  //<7>:Speed over the Ground in Knots
  //<8>:Track Angle in Degree
  //<9>:UTC Date(DDMMYY)
  //<10>:Megnetic Variation in Degree
  //<11>:Direction of Megnetic Variation('E' or 'W')
  //<12>:Mode(A:Auto / D:Diff / E:Estimate / N:Data None)
  //<13>:CheckSum

  int ContentSize;
  int i;

  XMDEBUG("%s\n",data);
  RMC* pRMC=(RMC*)malloc(sizeof(RMC));      //Alloc Memory with RMC Struct
  char* tmp=(char*)malloc(RMC_MAXLEN);      //Alloc Memory with RMC Max Size

  //<0>
  ContentSize=ContentBetween(data,tmp,0); //Get <0>
  for(i=0;i<ContentSize;i++)
    pRMC->Tag[i]=tmp[i];      //Copy Tag
  pRMC->Tag[i]='\0';
  XMDEBUG("[%s] ", pRMC->Tag);
  //<1>
  ContentSize=ContentBetween(data,tmp,1); //Get <1>
  pRMC->UTCTime=ContentSize>0?Str2Double(tmp):0;
  XMDEBUG("[%3.3f] ",pRMC->UTCTime);
  //<2>
  ContentSize=ContentBetween(data,tmp,2); //Get <2>
  pRMC->Status=ContentSize>0?tmp[0]:'V';  //InValid
  XMDEBUG("[%c] ",pRMC->Status);
  //<3>
  ContentSize=ContentBetween(data,tmp,3); //Get <3>
  pRMC->Latitude=ContentSize>0?Str2Double(tmp):0;
  XMDEBUG("[%4.4f] ",pRMC->Latitude);
  //<4>
  ContentSize=ContentBetween(data,tmp,4); //Get <4>
  pRMC->LatiMark=ContentSize>0?tmp[0]:'0';
  XMDEBUG("[%c] ",pRMC->LatiMark);
  //<5>
  ContentSize=ContentBetween(data,tmp,5); //Get <5>
  pRMC->Longitude=ContentSize>0?Str2Double(tmp):0;
  XMDEBUG("[%5.4f] ",pRMC->Longitude);
  //<6>
  ContentSize=ContentBetween(data,tmp,6); //Get <6>
  pRMC->LongMark=ContentSize>0?tmp[0]:'0';
  XMDEBUG("[%c] ",pRMC->LongMark);
  //<7>
  ContentSize=ContentBetween(data,tmp,7); //Get <7>
  pRMC->Speed=ContentSize>0?Str2Double(tmp):0;
  XMDEBUG("[%2.2f] ",pRMC->Speed);
  //<8>
  ContentSize=ContentBetween(data,tmp,8); //Get <8>
  pRMC->TrackDeg=ContentSize>0?Str2Double(tmp):0;
  XMDEBUG("[%2.2f] ",pRMC->TrackDeg);
  //<9>
  ContentSize=ContentBetween(data,tmp,9);//Get <9>
  pRMC->UTCDate=ContentSize>0?Str2Int(tmp):0;
  XMDEBUG("[%d] ",pRMC->UTCDate);
  //<10>
  ContentSize=ContentBetween(data,tmp,10);//Get <10>
  pRMC->MagneticAngel=ContentSize>0?Str2Double(tmp):0;
  XMDEBUG("[%3.2f] ",pRMC->MagneticAngel);
  //<11>
  ContentSize=ContentBetween(data,tmp,11);//Get <11>
  pRMC->MagneticDirect=ContentSize>0?tmp[0]:'0';
  XMDEBUG("[%c] ",pRMC->MagneticDirect);
  //<12>
  ContentSize=ContentBetween(data,tmp,12);//Get <12>
  pRMC->Mode=ContentSize>0?tmp[0]:'N';    //N:Data Valid
  XMDEBUG("[%c] ",pRMC->Mode);
  free(tmp);
  return pRMC;
}

//Function:XmVTGParse
//Descript:NMEA-0183 VTG Sentence Parse
VTG* XmVTGParse(const char* data)
{
  //Track Made Good and Speed Over Ground
  //$GPVTG,89.68,T,,M,0.00,N,0.0,K*5F
  //$GPVTG,140.88,T,,M,8.04,N,14.89,K,D*05
  //$GPVTG,<1>,<2>,<3>,<4>,<5>,<6>,<7>,<8>,<9>,<CR>,<LF>
  //<1>:True Angle Degree(000~359)
  //<2>:True Reference System(T)
  //<3>:Magnetic Angle Degree(000~359)
  //<4>:Magnetic Referenct System(M)
  //<5>:Speed and Unit is Knot(0.00)
  //<6>:Speed Unit(N:Knots)
  //<7>:Speed and Unit is Kilometer(0.00)

  int ContentSize;
  int i;

  XMDEBUG("%s\n",data);
  VTG* pVTG=(VTG*)malloc(sizeof(VTG));      //Alloc Memory VTG RMC Struct
  char* tmp=(char*)malloc(VTG_MAXLEN);      //Alloc Memory VTG RMC Max Size

  //<0>
  ContentSize=ContentBetween(data,tmp,0); //Get <0>
  for(i=0;i<ContentSize;i++)
    pVTG->Tag[i]=tmp[i];      //Copy Tag
  pVTG->Tag[i]='\0';
  XMDEBUG("[%s] ",pVTG->Tag);
  //<1>
  ContentSize=ContentBetween(data,tmp,1); //Get <1>
  pVTG->TrueDegree=ContentSize>0?Str2Double(tmp):0;
  XMDEBUG("[%4.2f] ",pVTG->TrueDegree);
  //<2>
  ContentSize=ContentBetween(data,tmp,2); //Get <2>
  pVTG->TrueSys=ContentSize>0?tmp[0]:'T';
  XMDEBUG("[%c] ",pVTG->TrueSys);
  //<3>
  ContentSize=ContentBetween(data,tmp,3); //Get <3>
  pVTG->MagneticDegree=ContentSize>0?Str2Double(tmp):0;
  XMDEBUG("[%4.2f] ",pVTG->MagneticDegree);
  //<4>
  ContentSize=ContentBetween(data,tmp,4); //Get <4>
  pVTG->MagneticSys=ContentSize>0?tmp[0]:'M';
  XMDEBUG("[%c] ",pVTG->MagneticSys);
  //<5>
  ContentSize=ContentBetween(data,tmp,5); //Get <5>
  pVTG->SpeedKnot=ContentSize>0?Str2Double(tmp):0;
  XMDEBUG("[%2.2f] ",pVTG->SpeedKnot);
  //<6>
  ContentSize=ContentBetween(data,tmp,6); //Get <6>
  pVTG->SpeedKnotUnit=ContentSize>0?tmp[0]:'N';
  XMDEBUG("[%c] ",pVTG->SpeedKnotUnit);
  //<7>
  ContentSize=ContentBetween(data,tmp,7); //Get <7>
  pVTG->SpeedKm=ContentSize>0?Str2Double(tmp):0;
  XMDEBUG("[%2.2f] ",pVTG->SpeedKm);
  //<8>
  ContentSize=ContentBetween(data,tmp,8); //Get <8>
  pVTG->SpeedKmUnit=ContentSize>0?tmp[0]:'K';
  XMDEBUG("[%c] ",pVTG->SpeedKmUnit);
  free(tmp);
  return pVTG;
}

//Function:XmGLLParse
//Descript:NMEA-0183 GLL Sentence Parse
GLL* XmGLLParse(const char* data)
{
  //Position Data,Position Fix,Time of Position Fix and Status
  //$GPGLL,3953.88008971,N,10506.75318910,W,034138.00,A,D*7A
  //$GPGLL,<1>,<2>,<3>,<4>,<5>,<6>,<7>,<CR>,<LF>
  //<1><2>:Latitude(DDMM.MMMM)   / ('N' or 'S')
  //<3><4>:Longitude(DDDMM.MMMM) / ('E' or 'W')
  //<5>:UTC Time(HHMMSS.SSS)
  //<6>:Status(A:Data Valid / V:Data InValid)
  //<7>:CheckSum
  int ContentSize;
  int i;

  XMDEBUG("%s\n",data);
  GLL* pGLL=(GLL*)malloc(sizeof(GLL));      //Alloc Memory with GLL Struct
  char* tmp=(char*)malloc(GLL_MAXLEN);      //Alloc Memory with GLL Max Size

  //<0>
  ContentSize=ContentBetween(data,tmp,0); //Get <0>
  for(i=0;i<ContentSize;i++)
    pGLL->Tag[i]=tmp[i];      //Copy Tag
  pGLL->Tag[i]='\0';
  XMDEBUG("[%s] ",pGLL->Tag);
  //<1>
  ContentSize=ContentBetween(data,tmp,1); //Get <1>
  pGLL->Latitude=ContentSize>0?Str2Double(tmp):0;
  XMDEBUG("[%4.4f] ",pGLL->Latitude);
  //<2>
  ContentSize=ContentBetween(data,tmp,2); //Get <2>
  pGLL->LatiMark=ContentSize>0?tmp[0]:'0';
  XMDEBUG("[%c] ",pGLL->LatiMark);
  //<3>
  ContentSize=ContentBetween(data,tmp,3); //Get <3>
  pGLL->Longitude=ContentSize>0?Str2Double(tmp):0;
  XMDEBUG("[%5.4f] ",pGLL->Longitude);
  //<4>
  ContentSize=ContentBetween(data,tmp,4); //Get <4>
  pGLL->LongMark=ContentSize>0?tmp[0]:'0';
  XMDEBUG("[%c] ",pGLL->LongMark);
  //<5>
  ContentSize=ContentBetween(data,tmp,5); //Get <5>
  pGLL->UTCTime=ContentSize>0?Str2Double(tmp):0;
  XMDEBUG("[%3.3f] ",pGLL->UTCTime);
  //<6>
  ContentSize=ContentBetween(data,tmp,6); //Get <6>
  pGLL->Status=ContentSize>0?tmp[0]:'V';  //InValid
  XMDEBUG("[%c] ",pGLL->Status);
  free(tmp);
  return pGLL;
}

//Function:XmZDAParse
//Descript:NMEA-0183 ZDA Sentence Parse
ZDA* XmZDAParse(const char* data)
{
  //UTC day,Month, Year and Local Time Zone Offset
  //$GPZDA,172809.456,12,07,1996,00,00*45
  //$GPZDA,<1>,<2>,<3>,<4>,<5>,<6>,<7>,<CR>,<LF>
  //<1>:
  //<2><3><4>:Day,Month,Year(01~31,01~12,Year)
  //<5>:Local Time Zone Offset from GMT(00~+/-13)
  //<6>:Local Time Zone Offset from GMT(00~59)
  //<7>:CheckSum
  int ContentSize;
  int i;

  XMDEBUG("%s\n",data);
  ZDA* pZDA=(ZDA*)malloc(sizeof(ZDA));      //Alloc Memory with ZDA Struct
  char* tmp=(char*)malloc(ZDA_MAXLEN);      //Alloc Memory with ZDA Max Size

  //<0>
  ContentSize=ContentBetween(data,tmp,0); //Get <0>
  for(i=0;i<ContentSize;i++)
    pZDA->Tag[i]=tmp[i];      //Copy Tag
  pZDA->Tag[i]='\0';
  XMDEBUG("[%s] ",pZDA->Tag);
  //<1>
  ContentSize=ContentBetween(data,tmp,1); //Get <1>
  pZDA->UTCTime=ContentSize>0?Str2Double(tmp):0;
  XMDEBUG("[%3.3f] ",pZDA->UTCTime);
  //<2>
  ContentSize=ContentBetween(data,tmp,2); //Get <2>
  pZDA->Day=ContentSize>0?Str2Int(tmp):0;
  XMDEBUG("[%d] ",pZDA->Day);
  //<3>
  ContentSize=ContentBetween(data,tmp,3); //Get <3>
  pZDA->Month=ContentSize>0?Str2Int(tmp):0;
  XMDEBUG("[%d] ",pZDA->Month);
  //<4>
  ContentSize=ContentBetween(data,tmp,4); //Get <4>
  pZDA->Year=ContentSize>0?Str2Int(tmp):0;
  XMDEBUG("[%d] ",pZDA->Year);
  //<5>
  ContentSize=ContentBetween(data,tmp,5); //Get <5>
  pZDA->HourZoneOffset=ContentSize>0?Str2Int(tmp):0;
  XMDEBUG("[%d] ",pZDA->HourZoneOffset);
  //<6>
  ContentSize=ContentBetween(data,tmp,6); //Get <6>
  pZDA->MinuteZoneOffset=ContentSize>0?Str2Int(tmp):0;
  XMDEBUG("[%d] ",pZDA->MinuteZoneOffset);
  free(tmp);
  return pZDA;
}

//Function:ContentBetween
//Descript:Find the Specific Field of GPS Data
int ContentBetween(const char* src,char* dst,int From)
{
  int count=0;
  int To=From+1;
  int FindFrom=FALSE;   //Find From Flag
  int FindTo=FALSE;     //Find To Flag
  int FromIdx=0;        //From Index
  int ToIdx=0;          //To Index
  int FindSize=0;       //Find Size
  int i;
  //Becaue I Remove "int To" from Input Data.
  //Currently,"To" Value must be Greater than "From"
  //if(To<From)
  //  return 0;

  for(i=0;src[i]!=NMEA_END;i++)       //Find Data to End('/n')
  {
    if((src[i]==NMEA_SEP) || (src[i]==NMEA_CHKSTART))    //Find ',' or '*'
      count++;
    if((count==From) && !FindFrom)    //Find From Position
      FromIdx=i, FindFrom=TRUE;
    if((count==To) && !FindTo)        //Find To Position
    {
      ToIdx=i, FindTo=TRUE;
      break;
    }
  }

  //Can not Find the Content Between the Range
  if(!FindTo)
    return 0;
  FindSize=ToIdx-FromIdx-1;   //Get Find Size
  if(FindSize==0)
    return 0;

  //memcpy(dst,src+FromIdx+1,FindSize);
  for(i=0;i<FindSize;i++)
    dst[i]=src[FromIdx+1+i];
  dst[i]='\0';                //Add String End
  return FindSize;
}

//Function:Str2Uint
//Descript:String Convert to Integer
int Str2Int(const char* str)
{
  int i=0;
  int Num=0;
  int Negative=1;     //Negative Flag

  if(str[i]=='-')     //Negative Sign
    Negative=-1, i++;
  else if(str[i]=='+')  //Positive Sign
    Negative=1, i++;

  while(str[i])
  {
    if(str[i]>='0' && str[i]<='9')   //0~9
      Num=Num*10+(str[i]-'0');
    else
      return 0;
    i++;
  }

  Num*=Negative;        //Negative Calculate
  return Num;
}

//Function:Str2Float
//Descript:String Covert to Float
double Str2Double(const char* str)
{
  int i=0;
  int Dot=FALSE;        //Dot Flag
  int Negative=1;       //Negative Flag
  double Num=0;          //Output Numeric
  double Decimal=0;      //Decimal
  double DotDigit=0;

  while(str[i])
  {
    if((str[i]>='0') && (str[i]<='9') && (Dot==TRUE))    //Numeric and After Dot
    {
      //Num=Num+((float)(str[i]-'0')*((float)DotDigit)),DotDigit*=0.100000;
      Decimal=Decimal+(float)(str[i]-'0')*DotDigit;
      DotDigit*=0.1000;
    }
    else if((str[i]>='0') && (str[i]<='9') && (Dot==FALSE)) //Numeric and Before Dot
      Num=Num*10+(str[i]-'0');
    else if(str[i]=='.')    //Dot
      Dot=TRUE,DotDigit=0.1000;
    else if(str[i]=='-')    //Negative
      Negative=-1;
    else
      return 0;
    i++;
  }

  Num+=Decimal;
  Num*=Negative;    //Negative Calculate
  return Num;
}

#ifdef XMAIN

//Main Code
int main(void)
{
  XMDEBUG("Start of XmNmea Sample...(%d)\n",123);

  NMEAInit();   //NMEA Initial

  //Str2Unit Function Test
  XMDEBUG("123      = %d\n",Str2Int("123"));
  XMDEBUG("-123     = %d\n",Str2Int("-123"));
  XMDEBUG("1-23     = %d\n",Str2Int("1-23"));

  //Str2Float Function Test
  XMDEBUG("12.34    = %3.2f\n",Str2Double("12.34"));
  XMDEBUG("123.456  = %3.3f\n",Str2Double("123.456"));
  XMDEBUG("-123.456 = %3.3f\n",Str2Double("-123.456"));
  XMDEBUG("12=3.456 = %3.3f\n",Str2Double("12=3.456"));

  //Parse GPS NMEA Sentence

  char GGA_Str[]="$GPGGA,172814.000,3723.4658,N,12202.2695,W,2,6,1.2,18.893,M,-25.669,M,2.0,0031*4F";
  char GSA_Str[]="$GPGSA,A,3,21,5,29,25,12,10,26,2,,,,,1.2,0.7,1.0*27";
  char GSV_Str[]="$GPGSV,8,1,25,21,44,141,47,15,14,049,44,6,31,255,46,3,25,280,44*75";
  char RMC_Str[]="$GPRMC,024813.640,A,3158.4608,N,11848.3737,E,10.05,324.27,150706,,,A*50";
  char VTG_Str[]="$GPVTG,140.88,T,,M,8.04,N,14.89,K,D*05";
  char GLL_Str[]="$GPGLL,3953.88008971,N,10506.75318910,W,034138.00,A,D*7A";
  char ZDA_Str[]="$GPZDA,172809.456,12,07,1996,00,00*45";

  GGA *pGGA=XmGGAParse(GGA_Str);  XMDEBUG("\n\n");
  GSA *pGSA=XmGSAParse(GSA_Str);  XMDEBUG("\n\n");
  GSV *pGSV=XmGSVParse(GSV_Str);  XMDEBUG("\n\n");
  RMC *pRMC=XmRMCParse(RMC_Str);  XMDEBUG("\n\n");
  VTG *pVTG=XmVTGParse(VTG_Str);  XMDEBUG("\n\n");
  GLL *pGLL=XmGLLParse(GLL_Str);  XMDEBUG("\n\n");
  ZDA *pZDA=XmZDAParse(ZDA_Str);  XMDEBUG("\n\n");

  XMDEBUG("GGA Size:%d\n",sizeof(pGGA));
  XMDEBUG("GSA Size:%d\n",sizeof(pGSA));
  XMDEBUG("GSV Size:%d\n",sizeof(pGSV));
  XMDEBUG("RMC Size:%d\n",sizeof(pRMC));
  XMDEBUG("VTG Size:%d\n",sizeof(pVTG));
  XMDEBUG("GLL Size:%d\n",sizeof(pGLL));
  XMDEBUG("ZDA Size:%d\n",sizeof(pZDA));

  //XmNmeaParse Sentence Parse
  XmNmeaParse(GGA_Str);  XMDEBUG("\n\n");
  XmNmeaParse(GSA_Str);  XMDEBUG("\n\n");
  XmNmeaParse(GSV_Str);  XMDEBUG("\n\n");
  XmNmeaParse(RMC_Str);  XMDEBUG("\n\n");
  XmNmeaParse(VTG_Str);  XMDEBUG("\n\n");
  XmNmeaParse(GLL_Str);  XMDEBUG("\n\n");
  XmNmeaParse(ZDA_Str);  XMDEBUG("\n\n");

  NMEA* _GPS=GetNMEA();

  XMDEBUG("%3.2f",_GPS->pGGA->Latitude);
  XMDEBUG("%c\n",_GPS->pGLL->Status);
  XMDEBUG("%2.1f\n",_GPS->pGSA->HDOP);
  XMDEBUG("%d\n",_GPS->pGSV->MsgNum);
  XMDEBUG("%5.4f",_GPS->pRMC->Longitude);
  XMDEBUG("%3.2f\n",_GPS->pVTG->SpeedKnot);
  XMDEBUG("%d\n",_GPS->pZDA->Month);

  const char ZDA_Str2[]="$GPZDA,333333.333,12,07,1996,00,00*45\r\n";
  int i;

  for(i=0;ZDA_Str2[i]!='\0';i++)
    XmNmeaBufferGet(ZDA_Str2[i]);

  _GPS=GetNMEA();
  XMDEBUG("[%6.4f]\n",_GPS->pZDA->UTCTime);

  free(_GPS);
  XMDEBUG("End of XmNmea Sample...(%d)\n",456);
  return 0;
}

#endif