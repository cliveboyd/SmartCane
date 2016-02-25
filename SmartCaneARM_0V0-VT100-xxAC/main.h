/*
	Description		Function Definitions main.h
	Author			Clive S. Boyd

*/


/******************************************************************************/
/* Include Files                                                              */
/******************************************************************************/
#include <stdint.h>  // for uint32_t etc.
#include <stdbool.h>



/******************************************************************************/
/* Functions Prototypes                                                       */
/******************************************************************************/

int Print_byte_as_binary(int x);

int printASMReg(void);

int32_t Read32_ALL_IO(void);

int Grab_EEPROM_ID(char* name, int len);

int32_t temperature_nRF51_get(void);

int Read32_ASM_SP(void);

