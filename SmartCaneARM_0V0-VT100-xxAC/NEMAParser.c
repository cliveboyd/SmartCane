/*

	NMEAParser.c

	Author CS Boyd 
	January 2016
	
	Implementation of the NMEAParser to suit A2035H GPS Module
	In theis instance the GPS Module is configured as SPI and subsequently
	passes a string to the parser below.
	
	The NEMA parser is adapted from Windows Visual Studio Code and modified
	to suit a Nordic nRF51822 ARM processor running softcore S110.
	
	NOTE::: The following prototype routines have NOT been debuged....			ToDo !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
*/

#include <stdbool.h> 
#include <stdint.h>  					// for uint32_t etc.
#include <stdio.h>
#include <string.h>  					// for memset()
#include <stdlib.h>
#include <ctype.h>

#include "NEMAParser.h"
#include "global.h"

/////////////////////////////////////////
//		Construction/Destruction
/////////////////////////////////////////

															// ToDo Sort Out Entry String
//NMEAParser(LPCTSTR outputFileName) {
//  m_logging = TRUE;
//  m_outputFile.Open(outputFileName, CFile::modeCreate | CFile::modeWrite );
//}

static GPSInfo_t 			m_GPSInfo;
static NMEAParserState_t	m_State = SearchForSOS;


int axtoi( const char *hexStg ) {
	int n = 0;												// Position in string
	int m = 0;         										// Position in digit[] to shift
	int count;         										// Loop index
	int intValue = 0;										// Integer value of hex string
	int digit[5];											// Hold values to convert
	while (n < 4)   {
		if (hexStg[n]=='\0') break;
		if (hexStg[n] > 0x29 && hexStg[n] < 0x40 )			// If 0 to 9
			digit[n] = hexStg[n] & 0x0f;					// Convert to int
		else if (hexStg[n] >='a' && hexStg[n] <= 'f')		// If a to f
			digit[n] = (hexStg[n] & 0x0f) + 9;				// Convert to int
		else if (hexStg[n] >='A' && hexStg[n] <= 'F')		// If A to F
			digit[n] = (hexStg[n] & 0x0f) + 9;				// Convert to int
		else break;
    n++;
	}
	count = n;
	m = n - 1;
	n = 0;
	while(n < count)   {
		intValue = intValue | (digit[n] << (m << 2));
		m--;												// Adjust the position to set
		n++;												// Next digit to process
	}
	return (intValue);
}

void Parse(const char *buf, const uint16_t bufSize) {		// CSB---> Main recursive parser entry. Feed text *buf into here

//
	for( uint16_t i = 0; i < bufSize; i++ )
		ParseRecursive(buf[i]);

//	load up Global Buffers and direct up to main()
	main_latitude=m_GPSInfo.m_latitude;						// Pass back the values of m_GPSInfo up to main global variables
	main_longitude=m_GPSInfo.m_longitude;
	main_altitude=m_GPSInfo.m_altitude;
	main_nSentences=m_GPSInfo.m_nSentences;
//	main_signalQuality=m_GPSInfo.m_signalQuality;			// CSB CSB TEST TEST TEST
	main_signalQuality=main_signalQuality+1;				// CSB CSB TEST TEST TEST
	main_satelitesInUse=m_GPSInfo.m_satelitesInUse;
	
	main_year=m_GPSInfo.m_year;
	main_month=m_GPSInfo.m_month;
	main_day=m_GPSInfo.m_day;
	main_hour=m_GPSInfo.m_hour;
	main_minute=m_GPSInfo.m_minute;
	main_second=m_GPSInfo.m_second;
}

void ParseRecursive(const char ch) {

	static const uint16_t ADDRESS_FIELD_MAX_LENGTH = 10;
	static const uint16_t NMEA_SEQUENCE_MAX_LENGTH = 81;
	
	m_State = SearchForSOS;
	
	static uint16_t m_CalcChecksum;
	static char m_Checksum[3];
	static char m_NMEASequence[NMEA_SEQUENCE_MAX_LENGTH];
	static uint16_t m_NMEASequenceIndex;
	static char m_AddressField[ADDRESS_FIELD_MAX_LENGTH];
	static uint16_t m_AddressFieldIndex;
						 
	switch( m_State ) {
		
	case SearchForSOS:
	{
		if( ch == '$' ) {
			m_AddressFieldIndex = 0;
			m_NMEASequenceIndex = 0;
			m_CalcChecksum = 0;
			m_State = RetrieveAddressField;
		}
		break;
	}

	case RetrieveAddressField: 
	{
	  if( m_NMEASequenceIndex == NMEA_SEQUENCE_MAX_LENGTH - 1 )
			m_State = SearchForSOS;
	  else {
		m_NMEASequence[m_NMEASequenceIndex++] = ch;
		m_CalcChecksum ^= ch;
		if( ch == ',' ) {
			m_AddressField[m_AddressFieldIndex] = '\0';
			m_State = ReceiveSentenceData;
		}
		else if( m_AddressFieldIndex == ADDRESS_FIELD_MAX_LENGTH - 1 ||
				 !isalpha(ch) || islower(ch) )										// ERROR NO Definition for islower and isalpha ????????????
			m_State = SearchForSOS;
		else
			m_AddressField[m_AddressFieldIndex++] = ch;
	  }
	  break;
	}

	case ReceiveSentenceData: 
	{
		if( m_NMEASequenceIndex == NMEA_SEQUENCE_MAX_LENGTH - 1 ) m_State = SearchForSOS;
		else {
		m_NMEASequence[m_NMEASequenceIndex++] = ch;
		if( ch == '*' )		m_State = GetFirstChecksumCharacter;
		else if( ch == 10 )	m_State = WaitForST;
		else if( ch == 13 ) {
			m_NMEASequence[m_NMEASequenceIndex++] = ch;
			m_NMEASequence[m_NMEASequenceIndex] = '\0';
			ParseNMEASentence( m_AddressField, m_NMEASequence, m_NMEASequenceIndex );
			m_State = SearchForSOS;
		}
		else
			m_CalcChecksum ^= ch;
		}
		break;
	}

	case GetFirstChecksumCharacter:
	{
		if( m_NMEASequenceIndex == NMEA_SEQUENCE_MAX_LENGTH - 1 ||
			( !isdigit(ch) && ( ch < 'A' || ch > 'F' ) ) )
			m_State = SearchForSOS;
		else {
			m_NMEASequence[m_NMEASequenceIndex++] = ch;
			m_Checksum[0] = ch;
			m_State = GetSecondChecksumCharacter;
		}
		break;
	}

    case GetSecondChecksumCharacter: 
	{
	if( m_NMEASequenceIndex == NMEA_SEQUENCE_MAX_LENGTH - 1 ||
	  ( !isdigit(ch) && ( ch < 'A' || ch > 'F' ) ) )
			m_State = SearchForSOS;
	else {
		m_NMEASequence[m_NMEASequenceIndex++] = ch;
		m_Checksum[1] = ch;
		m_Checksum[2] = '\0';
		uint16_t iChecksum = axtoi( m_Checksum );
		if( iChecksum == m_CalcChecksum )
			m_State = WaitForST;
		else
			m_State = SearchForSOS;
	}
	break;
	}

	case WaitForST:
	{
		if( m_NMEASequenceIndex == NMEA_SEQUENCE_MAX_LENGTH - 1 ||
		  (ch != 10 && ch != 13) )
			m_State = SearchForSOS;
		else if(ch == 13)
		{
			m_NMEASequence[m_NMEASequenceIndex++] = ch;
			m_NMEASequence[m_NMEASequenceIndex] = '\0';
			ParseNMEASentence( m_AddressField, m_NMEASequence, m_NMEASequenceIndex );
			m_State = SearchForSOS;
		}
	  break;
	}

	default:
		break;
  }

}

void ParseNMEASentence(const char *addressField,
								   const char *buf, const uint16_t bufSize)
{
	if		( strcmp(addressField, "GPGGA") == NULL ) ProcessGPGGA(buf, bufSize);		// $GPGGA - Global Positioning System Fix Data
	else if	( strcmp(addressField, "GPGSA") == NULL ) ProcessGPGSA(buf, bufSize);		// $GPGSA - GPS DOP and active satellites 
	else if	( strcmp(addressField, "GPGSV") == NULL ) ProcessGPGSV(buf, bufSize);		// $GPGSV - GPS Satellites in view
	else if	( strcmp(addressField, "GPRMB") == NULL ) ProcessGPRMB(buf, bufSize);		// $GPRMB - Recommended minimum navigation info
	else if	( strcmp(addressField, "GPRMC") == NULL ) ProcessGPRMC(buf, bufSize);		// $GPRMC - Recommended minimum specific GPS/Transit data
	else if	( strcmp(addressField, "GPZDA") == NULL ) ProcessGPZDA(buf, bufSize);		// $GPZDA - Date & Time


// GPSInfo & GetActualGPSInfo() {
//	return m_GPSInfo;
}


/*
GPGGA Sentence format
$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M, ,*47
   |   |	  |			 |			 | |  |	  |		  |      | | 
   |   |	  |			 |			 | |  |	  |		  |		 | checksum data
   |   |	  |			 |			 | |  |	  |		  |		 |
   |   |	  |			 |			 | |  |	  |		  |		 empty field
   |   |	  |			 |			 | |  |	  |		  |
   |   |	  |			 |			 | |  |	  |		  46.9,M Height of geoid (m) above WGS84 ellipsoid
   |   |	  |			 |			 | |  |	  |
   |   |	  |			 |			 | |  |	  545.4,M Altitude (m) above mean sea level
   |   |	  |			 |			 | |  |
   |   |	  |			 |			 | |  0.9 Horizontal dilution of position (HDOP)
   |   |	  |			 |			 | |
   |   |	  |			 |			 | 08 Number of satellites being tracked
   |   |	  |			 |			 |
   |   |	  |			 |			 1 Fix quality:	0 = invalid
   |   |	  |			 |							1 = GPS fix (SPS)
   |   |	  |			 |							2 = DGPS fix
   |   |	  |			 |							3 = PPS fix
   |   |	  |			 |							4 = Real Time Kinematic
   |   |	  |			 |							5 = Float RTK
   |   |	  |			 |							6 = estimated (dead reckoning) (2.3 feature)
   |   |	  |			 |							7 = Manual input mode
   |   |	  |			 |							8 = Simulation mode
   |   |	  |			 |
   |   |	  |			 01131.000,E Longitude 11 deg 31.000' E
   |   |	  |
   |   |	  4807.038,N Latitude 48 deg 07.038' N	
   |   |
   |   123519 Fix taken at 12:35:19 UTC
   |
   GGA Global Positioning System Fix Data

*/
void ProcessGPGGA(const char *buf, const uint16_t bufSize) {						// Global Positioning System Fix Data
	// To disable handling this sentence uncomment the next line --->
	// return;

	char auxBuf[10];
	const char *p1 = buf, *p2;

	// GGA
	if((uint16_t)(p1 - buf) >= bufSize) 	return;
	if(bufSize < 6) 						return;
	
	strncpy(auxBuf, buf, 5);
	auxBuf[5] = '\0';
	if(strcmp(auxBuf, "GPGGA") != 0 || buf[5] != ',') return;
	
	p1 += 6;

	// Time
	if((uint16_t)(p1 - buf) >= bufSize)		return;
	if((p2 = strchr(p1, ',')) == NULL)		return;
	
	uint16_t hour, min, sec;
	strncpy(auxBuf, p1, 2);
	auxBuf[2] = '\0';
	hour = atoi(auxBuf);
	p1 += 2;
	strncpy(auxBuf, p1, 2);
	auxBuf[2] = '\0';
	min = atoi(auxBuf);
	p1 += 2;
	strncpy(auxBuf, p1, 2);
	auxBuf[2] = '\0';
	sec = atoi(auxBuf);
	p1 = p2 + 1;

    // Latitude
	if((uint16_t)(p1 - buf) >= bufSize)		return;
	if((p2 = strchr(p1, ',')) == NULL)		return;
	strncpy(auxBuf, p1, p2 - p1);
	auxBuf[p2 - p1] = '\0';
	p1 = p2 + 1;
	
	if((p2 = strchr(auxBuf, '.')) == NULL) 	return;
	if(p2-auxBuf < 2)						return;
	double latitude = atof(p2 - 2) / 60.0;
	auxBuf[p2 - 2 - auxBuf] = '\0';
	latitude += atof(auxBuf);
	
	if((p2 = strchr(p1, ',')) == NULL)		return;
	if(p2 - p1 != 1)						return;
	if(*p1 == 'S') latitude = -latitude;
	else if(*p1 != 'N') 					return;
	p1 = p2 + 1;

    // Longitude
	if((uint16_t)(p1 - buf) >= bufSize)		return;
	if((p2 = strchr(p1, ',')) == NULL)		return;
	
	strncpy(auxBuf, p1, p2 - p1);
	auxBuf[p2 - p1] = '\0';
	p1 = p2 + 1;
	if((p2 = strchr(auxBuf, '.')) == NULL)	return;
	
	double longitude = atof(p2 - 2) / 60.0;
	auxBuf[p2 - 2 - auxBuf] = '\0';
	longitude += atof(auxBuf);
	if((p2 = strchr(p1, ',')) == NULL) 		return;
	if(p2 - p1 != 1)						return;
	
	if(*p1 == 'W') longitude = -longitude;
	else if(*p1 != 'E')						return;
	p1 = p2 + 1;

	// GPS quality
	if((uint16_t)(p1 - buf) >= bufSize)		return;
	if((p2 = strchr(p1, ',')) == NULL)		return;
	strncpy(auxBuf, p1, p2 - p1);
	auxBuf[p2 - p1] = '\0';
	p1 = p2 + 1;
	uint16_t quality = atoi(auxBuf);

	// Satellites in use
	if((uint16_t)(p1 - buf) >= bufSize)		return;
	if((p2 = strchr(p1, ',')) == NULL)		return;
	strncpy(auxBuf, p1, p2 - p1);
	auxBuf[p2 - p1] = '\0';
	p1 = p2 + 1;
	uint16_t satelitesInUse = atoi(auxBuf);
	
	// HDOP
	if((uint16_t)(p1 - buf) >= bufSize)		return;
	if((p2 = strchr(p1, ',')) == NULL)		return;
	strncpy(auxBuf, p1, p2 - p1);
	auxBuf[p2 - p1] = '\0';
	p1 = p2 + 1;
	double hdop = atoi(auxBuf);
	
	// Altitude
	if((uint16_t)(p1 - buf) >= bufSize)		return;
	if((p2 = strchr(p1, ',')) == NULL)		return;
	strncpy(auxBuf, p1, p2 - p1);
	auxBuf[p2 - p1] = '\0';
	p1 = p2 + 1;
	double altitude = atoi(auxBuf);
	if((p2 = strchr(p1, ',')) == NULL) 		return;
	if(p2 - p1 != 1)						return;
	if(*p1 != 'M')							return;
	p1 = p2 + 1;

	// Height of geoid
	if((uint16_t)(p1 - buf) >= bufSize)		return;
	if((p2 = strchr(p1, ',')) == NULL)		return;
	strncpy(auxBuf, p1, p2 - p1);
	auxBuf[p2 - p1] = '\0';
	p1 = p2 + 1;
	double heightGeoid = atoi(auxBuf);
	if((p2 = strchr(p1, ',')) == NULL) 		return;
//	if(p2 - p1 != 1) return; 
//	if(*p1 != 'M') return;
	p1 = p2 + 1;

	// Empty field
	if((uint16_t)(p1 - buf) >= bufSize)		return;
	if((p2 = strchr(p1, ',')) == NULL)		return;
	p1 = p2 + 1;

	// Last Empty field
	if((uint16_t)(p1 - buf) >= bufSize)		return;
	if((p2 = strchr(p1, ',')) != NULL)		return;
	if((p2 = strchr(p1, '*')) == NULL)		return;


	m_GPSInfo.m_latitude		= latitude;								// Set the values of m_GPSInfo
	m_GPSInfo.m_longitude		= longitude;
	m_GPSInfo.m_altitude		= altitude;

	m_GPSInfo.m_nSentences++;

m_GPSInfo.m_signalQuality	= quality;
	m_GPSInfo.m_satelitesInUse	= satelitesInUse;
//	m_GPSInfo.m_year			= year;
//	m_GPSInfo.m_month			= month;
//	m_GPSInfo.m_day				= day;
	m_GPSInfo.m_hour			= hour;
	m_GPSInfo.m_minute			= min;
	m_GPSInfo.m_second			= sec;

	
}

void ProcessGPGSA(const char *buf, const uint16_t bufSize) {			// GPS DOP and active satellites
/*
	GPS DOP and active satellites

eg1. $GPGSA,A,3,,,,,,16,18,,22,24,,,3.6,2.1,2.2*3C
eg2. $GPGSA,A,3,19,28,14,18,27,22,31,39,,,,,1.7,1.0,1.3*35


1    = Mode:
       M=Manual, forced to operate in 2D or 3D
       A=Automatic, 3D/2D
2    = Mode:
       1=Fix not available
       2=2D
       3=3D
3-14 = IDs of SVs used in position fix (null for unused fields)
15   = PDOP
16   = HDOP
17   = VDOP  */


}

void ProcessGPGSV(const char *buf, const uint16_t bufSize) {			// Satellites in View
/*
	GPS Satellites in view

eg. $GPGSV,3,1,11,03,03,111,00,04,15,270,00,06,01,010,00,13,06,292,00*74
    $GPGSV,3,2,11,14,25,170,00,16,57,208,39,18,67,296,40,19,40,246,00*74
    $GPGSV,3,3,11,22,42,067,42,24,14,311,43,27,05,244,00,,,,*4D

    $GPGSV,1,1,13,02,02,213,,03,-3,000,,11,00,121,,14,13,172,05*67

1    = Total number of messages of this type in this cycle
2    = Message number
3    = Total number of SVs in view
4    = SV PRN number
5    = Elevation in degrees, 90 maximum
6    = Azimuth, degrees from true north, 000 to 359
7    = SNR, 00-99 dB (null when not tracking)
8-11 = Information about second SV, same as field 4-7
12-15= Information about third SV, same as field 4-7
16-19= Information about fourth SV, same as field 4-7 	*/

}

void ProcessGPRMB(const char *buf, const uint16_t bufSize) {			// Recommended minimum navigation info
/*
	$GPRMB

Recommended minimum navigation information (sent by nav. receiver when a destination waypoint is active)

eg1. $GPRMB,A,0.66,L,003,004,4917.24,N,12309.57,W,001.3,052.5,000.5,V*0B


           A            Data status A = OK, V = warning
           0.66,L       Cross-track error (nautical miles, 9.9 max.),
                                steer Left to correct (or R = right)
           003          Origin waypoint ID
           004          Destination waypoint ID
           4917.24,N    Destination waypoint latitude 49 deg. 17.24 min. N
           12309.57,W   Destination waypoint longitude 123 deg. 09.57 min. W
           001.3        Range to destination, nautical miles
           052.5        True bearing to destination
           000.5        Velocity towards destination, knots
           V            Arrival alarm  A = arrived, V = not arrived
           *0B          mandatory checksum


eg2. $GPRMB,A,4.08,L,EGLL,EGLM,5130.02,N,00046.34,W,004.6,213.9,122.9,A*3D
            1   2  3   4    5    6     7   8      9   10    11    12   13

  
      1    A         validity
      2    4.08      off track
      3    L         Steer Left (L/R)
      4    EGLL      last waypoint
      5    EGLM      next waypoint
      6    5130.02   Latitude of Next waypoint
      7    N         North/South
      8    00046.34  Longitude of next waypoint
      9    W         East/West
      10   004.6     Range
      11   213.9     bearing to waypt.
      12   122.9     closing velocity
      13   A         validity
      14   *3D       checksum


eg3. $GPRMB,A,x.x,a,c--c,d--d,llll.ll,e,yyyyy.yy,f,g.g,h.h,i.i,j*kk
1    = Data Status (V=navigation receiver warning)
2    = Crosstrack error in nautical miles
3    = Direction to steer (L or R) to correct error
4    = Origin waypoint ID#
5    = Destination waypoint ID#
6    = Destination waypoint latitude
7    = N or S
8    = Destination waypoint longitude
9    = E or W
10   = Range to destination in nautical miles
11   = Bearing to destination, degrees True
12   = Destination closing velocity in knots
13   = Arrival status; (A=entered or perpendicular passed)
14   = Checksum */

}

/*
	Format
	$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
	 |	 |		| |			 |			 |	   |	 |		|	   |
	 |	 |		| |			 |			 |	   |	 |		|	   *6A Checksum data
	 |	 |		| |			 |			 |	   |	 |		|
	 |	 |		| |			 |			 |	   |	 |		003.1,W Magnetic Variation
	 |	 |		| |			 |			 |	   |	 |
	 |	 |		| |			 |			 |	   |	 230394 Date - 23rd of March 1994
	 |	 |		| |			 |			 |	   |
	 |	 |		| |			 |			 |	   084.4 Track angle in degrees
	 |	 |		| |			 |			 |	   
	 |	 |		| |			 |			 022.4 Speed over the ground in knots
	 |	 |		| |			 |
	 |	 |		| |			 01131.000,E Longitude 11 deg 31.000' E
	 |	 |		| |
	 |	 |		| 4807.038,N Latitude 48 deg 07.038' N
	 |	 |		|
	 |	 |		A Status A=active or V=Void
	 |	 |
	 |	 123519 Fix taken at 12:35:19 UTC
	 |
	 RMC Recommended Minimum sentence C

*/
void ProcessGPRMC(const char *buf, const uint16_t bufSize) {		// Recommended minimum specific GPS/Transit data
	char auxBuf[10];
	const char *p1 = buf, *p2;

	// GGA
	if((uint16_t)(p1 - buf) >= bufSize) 	return;
	if(bufSize < 6) 						return;
	strncpy(auxBuf, buf, 5);
	auxBuf[5] = '\0';
	if(strcmp(auxBuf, "GPRMC") != 0 || buf[5] != ',') return;
	p1 += 6;

	// Time
	if((uint16_t)(p1 - buf) >= bufSize) 	return;
	if((p2 = strchr(p1, ',')) == NULL) 		return;
	uint16_t hour, min, sec;
	strncpy(auxBuf, p1, 2);
	auxBuf[2] = '\0';
	hour = atoi(auxBuf);
	p1 += 2;
	strncpy(auxBuf, p1, 2);
	auxBuf[2] = '\0';
	min = atoi(auxBuf);
	p1 += 2;
	strncpy(auxBuf, p1, 2);
	auxBuf[2] = '\0';
	sec = atoi(auxBuf);
	p1 = p2 + 1;
	
	// Status 
	if((uint16_t)(p1 - buf) >= bufSize)		return;
	if((p2 = strchr(p1, ',')) == NULL) 		return;
	if(p2 == p1)							return;
//	if(*p1 != 'A') return;
	p1 = p2 + 1;

    // Latitude
	if((uint16_t)(p1 - buf) >= bufSize) 	return;
	if((p2 = strchr(p1, ',')) == NULL) 		return;
	strncpy(auxBuf, p1, p2 - p1);
	auxBuf[p2 - p1] = '\0';
	p1 = p2 + 1;
	if((p2 = strchr(auxBuf, '.')) == NULL) 	return;
	if(p2-auxBuf < 2) 						return;
	double latitude = atof(p2 - 2) 									/ 60.0;
	auxBuf[p2 - 2 - auxBuf] = '\0';
	latitude += atof(auxBuf);
	if((p2 = strchr(p1, ',')) == NULL) 		return;
	if(p2 - p1 != 1) 						return;
	if(*p1 == 'S') latitude = -latitude;
	else if(*p1 != 'N') 					return;
	p1 = p2 + 1;

    // Longitude
	if((uint16_t)(p1 - buf) >= bufSize) 	return;
	if((p2 = strchr(p1, ',')) == NULL) 		return;
	strncpy(auxBuf, p1, p2 - p1);
	auxBuf[p2 - p1] = '\0';
	p1 = p2 + 1;
	if((p2 = strchr(auxBuf, '.')) == NULL) 	return;
	double longitude = atof(p2 - 2) 								/ 60.0;
	auxBuf[p2 - 2 - auxBuf] = '\0';
	longitude += atof(auxBuf);
	if((p2 = strchr(p1, ',')) == NULL) 		return;
	if(p2 - p1 != 1) 						return;
	if(*p1 == 'W') longitude = -longitude;
	else if(*p1 != 'E') 					return;
	p1 = p2 + 1;

	// Ground speed
	if((uint16_t)(p1 - buf) >= bufSize) 	return;
	if((p2 = strchr(p1, ',')) == NULL) 		return;
	strncpy(auxBuf, p1, p2 - p1);
	auxBuf[p2 - p1] = '\0';
	p1 = p2 + 1;
	double groundSpeed = atof(auxBuf);

	// Course over ground (degrees) 
	if((uint16_t)(p1 - buf) >= bufSize) 	return;
	if((p2 = strchr(p1, ',')) == NULL) 		return;
	strncpy(auxBuf, p1, p2 - p1);
	auxBuf[p2 - p1] = '\0';
	p1 = p2 + 1;
	double courseOverGround = atof(auxBuf);

	// Date
	if((uint16_t)(p1 - buf) >= bufSize) 	return;
	if((p2 = strchr(p1, ',')) == NULL) 		return;
	uint16_t day, month, year;
	strncpy(auxBuf, p1, 2);
	auxBuf[2] = '\0';
	day = atoi(auxBuf);
	p1 += 2;
	strncpy(auxBuf, p1, 2);
	auxBuf[2] = '\0';
	month = atoi(auxBuf);
	p1 += 2;
	strncpy(auxBuf, p1, 2);
	auxBuf[2] = '\0';
	year = 2000 + atoi(auxBuf);
	p1 = p2 + 1;

	// Magnetic variation
	if((uint16_t)(p1 - buf) >= bufSize)		return;
	if((p2 = strchr(p1, ',')) == NULL)		return;
	strncpy(auxBuf, p1, p2 - p1);
	auxBuf[p2 - p1] = '\0';
	p1 = p2 + 1;
	double magneticVariation = atof(auxBuf);
	if((p2 = strchr(p1, '*')) == NULL)		return;
	if(p2 - p1 > 1)							return;
	
	if(*p1 == 'W') latitude = -latitude;
	else if(*p1 != 'E' && *p1 != '*')		return;
	
	m_GPSInfo.m_latitude		= latitude;								// Set the values of m_GPSInfo
	m_GPSInfo.m_longitude		= longitude;
	m_GPSInfo.m_nSentences++;
	
	m_GPSInfo.m_year			= year;
	m_GPSInfo.m_month			= month;
	m_GPSInfo.m_day				= day;
	m_GPSInfo.m_hour			= hour;
	m_GPSInfo.m_minute			= min;
	m_GPSInfo.m_second			= sec;	
}

void ProcessGPZDA(const char *buf, const uint16_t bufSize) { 			// Date & Time
/*	$GPZDA	Date & Time

	UTC, day, month, year, and local time zone.

	$--ZDA,hhmmss.ss,xx,xx,xxxx,xx,xx
	hhmmss.ss = UTC 
	xx = Day, 01 to 31 
	xx = Month, 01 to 12 
	xxxx = Year 
	xx = Local zone description, 00 to +/- 13 hours 
	xx = Local zone minutes description (same sign as hours) 	*/
	
}
