# XmNmea

XmNmea is a Follows NMEA-0183 Standard NMEA Parser Library is Written in C Language
Xm Represents a Medium Level Library Made by William XIAO.

## Feature

1. LightWeight(Only 2 Files)
2. Simple API
3. C99 Compatible
4. Code about 800 Line(Will Exceed 1000 Lines in the Future)
5. Suitable for Embedded Use
* Xs Level Code Under 1000 Lines 
* Xm Level Code Between 1000 ~ 5000 Lines
* Xl Level Code Between 5000 ~ 10000 Lines
* Xe Level Code Over 10000 Lines

## Install

* make Sample
  * Execute all the Following Function
* make Clean
  * Clean all Generate File

```c
//If you want to Execute Sample Code,You need to Add Main
#define MAIN

//If you want to Reading Debug Console,You need to Add DEBUG
#define DEBUG
```

## Usage

* The Following Four Functions are Particularly Important,while Others are Sub Function.
1. NMEAInit() : Make Sure to Use this Function First, It can Initialize your Structure Memory.
2. GetNMEA() : It can Get Parsed NMEA-0183 Data.
3. XmNmeaBufferGet() : Parser NMEA Data with NMEA Character.(Designed for GPS Uart)
4. XmNmeaParse() : Parser NMEA Data with NEMA Sentence.

### How to Use

XmNmeaParse() Method

```c
NMEAInit();
char VTG_Str[]="$GPVTG,140.88,T,,M,8.04,N,14.89,K,D*05";
XmNmeaParse(VTG_Str);

NMEA* _GPS=GetNMEA();
```

XmNmeaBufferGet() Method

```c
NMEAInit();
char VTG_Str[]="$GPVTG,140.88,T,,M,8.04,N,14.89,K,D*05";
for(i=0;ZDA_Str2[i]!='\0';i++)
  XmNmeaBufferGet(ZDA_Str2[i]);

NMEA* _GPS=GetNMEA();
```

You can Create a Thread to Collect GPS Data

```c
char c;

while(1)
{
  ret=read(fd,c,1)
  XmNmeaBufferGet(ZDA_Str2[i]);
}
```

## Future

I will Complete all NMEA-0183 Sentences. First, I am going to Add Garmin's Sentence.

## LICENSE

[The MIT License (MIT)](http://opensource.org/licenses/mit-license.php)
