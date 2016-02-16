/*	
Program...		SmartCane 1V0
Description...	The following includes definitions of global variables to be allocated within main() and any underlying function domains
Author...		Clive S Boyd
Date...			January 22, 2016
*/


#ifndef GLOBAL_H
#define GLOBAL_H

#include <stdint.h>  			// for uint32_t etc.
#include <stdbool.h>
#include <stdio.h>

extern float	roll;
extern float	pitch;
extern float	yaw;

extern int		TestFlags;

extern uint16_t SFLASH_GPS_flag;

extern double	main_latitude;
extern double	main_longitude;
extern double	main_altitude;
extern long		main_nSentences;
extern uint16_t	main_signalQuality;
extern uint16_t	main_satelitesInUse;

extern uint16_t	main_year;
extern uint16_t	main_month;
extern uint16_t	main_day;
extern uint16_t	main_hour;
extern uint16_t	main_minute;
extern uint16_t	main_second;

#endif
