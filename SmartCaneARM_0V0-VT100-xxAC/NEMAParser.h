/*
	NMEAParser.h: Interface for the NMEAParser modified to suit nRF51 A2035H GPS module.
	Author CS Boyd Jan 2016
*/


#include <stdbool.h> 
#include <stdint.h>  					// for uint32_t etc.
#include <stdio.h>
#include <string.h>  					// String Handling
#include <stdlib.h>
#include <global.h>

typedef struct {
	double		m_latitude;
	double		m_longitude;
	double		m_altitude;
	long		m_nSentences;
	uint16_t	m_signalQuality;
	uint16_t	m_satelitesInUse;
} GPSInfo_t;


typedef	enum {
	SearchForSOS = 1,
	RetrieveAddressField,
	ReceiveSentenceData,
	GetFirstChecksumCharacter,
	GetSecondChecksumCharacter,
	WaitForST,
	ValidSentence 
} NMEAParserState_t;


void NMEAParser (void); 

//	NMEAParser(LPCTSTR outputFileName);
//	virtual ~NMEAParser();

void Parse(const char *buf, const uint16_t bufSize);

	//	GPSInfo& GetActualGPSInfo();

void ParseRecursive(const char ch);

void ParseNMEASentence(const char *addressField, 
					   const char *buf, const uint16_t bufSize);

void ProcessGPGGA(const char *buf, const uint16_t bufSize);
void ProcessGPGSA(const char *buf, const uint16_t bufSize);
void ProcessGPGSV(const char *buf, const uint16_t bufSize);
void ProcessGPRMB(const char *buf, const uint16_t bufSize);
void ProcessGPRMC(const char *buf, const uint16_t bufSize);
void ProcessGPZDA(const char *buf, const uint16_t bufSize);

//bool m_logging;


