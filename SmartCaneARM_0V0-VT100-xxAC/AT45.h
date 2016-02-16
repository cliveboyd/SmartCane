#include <stdbool.h> 
#include <stdint.h>

#include "global.h"

void nrf_delay_ms(uint32_t volatile number_of_ms);

void AT45_flashwrite(int x,int y);

void AT45_flashread(int x, int y);

void AT45_sramwrite(int x, int y, int z);

void AT45_deselect (void);

void AT45_select (void);

void AT45_busy(void);

int AT45_status(void);

int spi_bcm_write (int x);

void AT45_sendaddr (int x);

int AT45_getpaddr (int x);							// Page   Address

int AT45_getbaddr (int x);							// Buffer Address

int AT45_init(void);

int AT45_memread(int x);

int AT45_write_page(char* data , int y);

int AT45_write_block(char *data[], int y);

int AT45_id(void);

int AT45_WhoAmI(void);								// device id == 0x1f26 ---> AT45DB161E

int AT45_initialize(void);

void AT45DB161E_init(void);

void AT45_Congigure_IO_Pins(void);

int BCM_Out_Low(void);

int BCM_Out_High(void);
