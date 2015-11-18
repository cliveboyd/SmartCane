/***************************/
1. setup_compass() for initializing compass;
2. mpu_set_compass_sample_rate for setting up compass sampling rate;
3. Read compass register by calling mpu_get_compass_reg() at the frequency being set by sampling rate;
4. Compass sampling rate at maximum could be set to gyro sampling rate.
/**************************/

int mpu_set_bypass(unsigned char bypass_on)
{
unsigned char tmp;

if (st.chip_cfg.bypass_mode == bypass_on)
return 0;

if (bypass_on) {
	if (i2c_read(st.hw->addr, st.reg->user_ctrl, 1, &tmp))
		return -1;
	tmp &= ~BIT_AUX_IF_EN;
	if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &tmp))
		return -1;
	delay_ms(3);
	tmp = BIT_BYPASS_EN;
	if (st.chip_cfg.active_low_int)
		tmp |= BIT_ACTL;
	if (st.chip_cfg.latched_int)
		tmp |= BIT_LATCH_EN | BIT_ANY_RD_CLR;
	if (i2c_write(st.hw->addr, st.reg->int_pin_cfg, 1, &tmp))
		return -1;
	} else {
	/* Enable I2C master mode if compass is being used. */
		if (i2c_read(st.hw->addr, st.reg->user_ctrl, 1, &tmp))
			return -1;
		if (st.chip_cfg.sensors & INV_XYZ_COMPASS)
			tmp |= BIT_AUX_IF_EN;
		else
			tmp &= ~BIT_AUX_IF_EN;
		if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &tmp))
			return -1;
		delay_ms(3);
		if (st.chip_cfg.active_low_int)
			tmp = BIT_ACTL;
		else
			tmp = 0;
		if (st.chip_cfg.latched_int)
			tmp |= BIT_LATCH_EN | BIT_ANY_RD_CLR;
		if (i2c_write(st.hw->addr, st.reg->int_pin_cfg, 1, &tmp))
			return -1;
	}
	st.chip_cfg.bypass_mode = bypass_on;
	return 0;
}

/*Initialize compass*/
static int setup_compass(void)
{
	#ifdef AK89xx_SECONDARY
	unsigned char data[4], akm_addr;

	mpu_set_bypass(1);

	/* Find compass. Possible addresses range from 0x0C to 0x0F. */
	for (akm_addr = 0x0C; akm_addr <= 0x0F; akm_addr++) {
		int result;
		result = i2c_read(akm_addr, AKM_REG_WHOAMI, 1, data);
		if (!result && (data[0] == AKM_WHOAMI))
			break;
	}

	if (akm_addr > 0x0F) {
		/* TODO: Handle this case in all compass-related functions. */
		log_e("Compass not found.\n");
		return -1;
	}

	st.chip_cfg.compass_addr = akm_addr;

	data[0] = AKM_POWER_DOWN;
	if (i2c_write(st.chip_cfg.compass_addr, AKM_REG_CNTL, 1, data))
		return -1;
	delay_ms(1);

	data[0] = AKM_FUSE_ROM_ACCESS;
	if (i2c_write(st.chip_cfg.compass_addr, AKM_REG_CNTL, 1, data))
		return -1;
	delay_ms(1);

	/* Get sensitivity adjustment data from fuse ROM. */
	if (i2c_read(st.chip_cfg.compass_addr, AKM_REG_ASAX, 3, data))
		return -1;
	st.chip_cfg.mag_sens_adj[0] = (long)data[0] + 128;
	st.chip_cfg.mag_sens_adj[1] = (long)data[1] + 128;
	st.chip_cfg.mag_sens_adj[2] = (long)data[2] + 128;

	data[0] = AKM_POWER_DOWN;
	if (i2c_write(st.chip_cfg.compass_addr, AKM_REG_CNTL, 1, data))
		return -1;
	delay_ms(1);

	mpu_set_bypass(0);

	/* Set up master mode, master clock, and ES bit. */
	data[0] = 0x40;
	if (i2c_write(st.hw->addr, st.reg->i2c_mst, 1, data))
		return -1;

	/* Slave 0 reads from AKM data registers. */
	data[0] = BIT_I2C_READ | st.chip_cfg.compass_addr;
	if (i2c_write(st.hw->addr, st.reg->s0_addr, 1, data))
		return -1;

	/* Compass reads start at this register. */
	data[0] = AKM_REG_ST1;
	if (i2c_write(st.hw->addr, st.reg->s0_reg, 1, data))
		return -1;

	/* Enable slave 0, 8-byte reads. */
	data[0] = BIT_SLAVE_EN | 8;
	if (i2c_write(st.hw->addr, st.reg->s0_ctrl, 1, data))
		return -1;

	/* Slave 1 changes AKM measurement mode. */
	data[0] = st.chip_cfg.compass_addr;
	if (i2c_write(st.hw->addr, st.reg->s1_addr, 1, data))
		return -1;

	/* AKM measurement mode register. */
	data[0] = AKM_REG_CNTL;
	if (i2c_write(st.hw->addr, st.reg->s1_reg, 1, data))
		return -1;

	/* Enable slave 1, 1-byte writes. */
	data[0] = BIT_SLAVE_EN | 1;
	if (i2c_write(st.hw->addr, st.reg->s1_ctrl, 1, data))
		return -1;

	/* Set slave 1 data. */
	data[0] = AKM_SINGLE_MEASUREMENT;
	if (i2c_write(st.hw->addr, st.reg->s1_do, 1, data))
		return -1;

	/* Trigger slave 0 and slave 1 actions at each sample. */
	data[0] = 0x03;
	if (i2c_write(st.hw->addr, st.reg->i2c_delay_ctrl, 1, data))
		return -1;

#ifdef MPU9150
	/* For the MPU9150, the auxiliary I2C bus needs to be set to VDD. */
	data[0] = BIT_I2C_MST_VDDIO;
	if (i2c_write(st.hw->addr, st.reg->yg_offs_tc, 1, data))
		return -1;
#endif // MPU9150

return 0;
#else //AK89xx_SECONDARY
return -1;
#endif //AK89xx_SECONDARY
}

/**
* @brief Set compass sampling rate.
* The compass on the auxiliary I2C bus is read by the MPU hardware at a
* maximum of 100Hz. The actual rate can be set to a fraction of the gyro
* sampling rate.
*
* \n WARNING: The new rate may be different than what was requested. Call
* mpu_get_compass_sample_rate to check the actual setting.
* @param[in] rate Desired compass sampling rate (Hz).
* @return 0 if successful.
*/
int mpu_set_compass_sample_rate(unsigned short rate)
{
#ifdef AK89xx_SECONDARY
unsigned char div;
if (!rate || rate > st.chip_cfg.sample_rate || rate > MAX_COMPASS_SAMPLE_RATE)
return -1;

div = st.chip_cfg.sample_rate / rate - 1;
if (i2c_write(st.hw->addr, st.reg->s4_ctrl, 1, &div))
return -1;
st.chip_cfg.compass_sample_rate = st.chip_cfg.sample_rate / (div + 1);
return 0;
#else
return -1;
#endif
}

/**
* @brief Read raw compass data.
* @param[out] data Raw data in hardware units.
* @param[out] timestamp Timestamp in milliseconds. Null if not needed.
* @return 0 if successful.
*/
int mpu_get_compass_reg(short *data, unsigned long *timestamp)
{
#ifdef AK89xx_SECONDARY
	unsigned char tmp[9];

	if (!(st.chip_cfg.sensors & INV_XYZ_COMPASS))
	return -1;

	#ifdef AK89xx_BYPASS
	if (i2c_read(st.chip_cfg.compass_addr, AKM_REG_ST1, 8, tmp))
	return -1;
	tmp[8] = AKM_SINGLE_MEASUREMENT;
	if (i2c_write(st.chip_cfg.compass_addr, AKM_REG_CNTL, 1, tmp+8))
	return -1;
	#else
	if (i2c_read(st.hw->addr, st.reg->raw_compass, 8, tmp))
	return -1;
	#endif

	#if defined AK8975_SECONDARY
	/* AK8975 doesn't have the overrun error bit. */
	if (!(tmp[0] & AKM_DATA_READY))
	return -2;
	if ((tmp[7] & AKM_OVERFLOW) || (tmp[7] & AKM_DATA_ERROR))
	return -3;
	#elif defined AK8963_SECONDARY
	/* AK8963 doesn't have the data read error bit. */
	if (!(tmp[0] & AKM_DATA_READY) || (tmp[0] & AKM_DATA_OVERRUN))
	return -2;
	if (tmp[7] & AKM_OVERFLOW)
	return -3;
	#endif
	data[0] = (tmp[2] << 8) | tmp[1];
	data[1] = (tmp[4] << 8) | tmp[3];
	data[2] = (tmp[6] << 8) | tmp[5];

	data[0] = ((long)data[0] * st.chip_cfg.mag_sens_adj[0]) >> 8;
	data[1] = ((long)data[1] * st.chip_cfg.mag_sens_adj[1]) >> 8;
	data[2] = ((long)data[2] * st.chip_cfg.mag_sens_adj[2]) >> 8;

	if (timestamp)
	get_ms(timestamp);
	return 0;
#else
	return -1;
#endif
}

/* 
 * File:   MPU_Test.c
 * Author: Ryan Nazaretian
 *
 * Created on April 16, 2013, 4:47 PM
 */

#include "esos.h"
#include "esos_pic24.h"
#include "MPU.h"

#define MPU_SLAVE_ADDRESS				0xD0  // 0x68 << 1 == 0xD0

#define MPU_REG_INT_PIN_CFG				0x37  // Configures the behavior of the interrupt signals at the INT pins
#define MPU_INT_PIN_CFG_I2C_BYPASS_EN	0x02  // If 1 and I2C_MST_EN = 0, the host application processor will be able to directly access the auxiliary I2C bus


#define MPU_MAG_I2C_ADDR				0x18  // 0x0C << 1 == 0x18
#define MPU_REG_MAG_WIA					0x00  // When read, should report 0x48
#define MPU_REG_MAG_CNTL				0x0A
#define MPU_MAG_SINGLE_MRMNT			0x01  //Single measurement mode

// ST1 Register - Contains Data Ready Bit
#define MPU_REG_MAG_ST1					0x02
#define MPU_MAG_ST1_NORMAL				0x00
#define MPU_MAG_ST1_READY				0x01

// ST2 Register - Contains Data Read Error and Magnetic Sensor Overflow Data
#define MPU_REG_MAG_ST2					0x09
#define MPU_MAG_ST2_DERR				0x04
#define MPU_MAG_ST2_HOFL				0x08

// HXL to HZH - Contains Measurement Data
#define MPU_REG_MAG_HXL					0x03    //Magnetometer X-Axis Low Byte
#define MPU_REG_MAG_HXH					0x04    //Magnetometer X-Axis High Byte
#define MPU_REG_MAG_HYL					0x05    //Magnetometer Y-Axis Low Byte
#define MPU_REG_MAG_HYH					0x06    //Magnetometer Y-Axis High Byte
#define MPU_REG_MAG_HZL					0x07    //Magnetometer Z-Axis Low Byte
#define MPU_REG_MAG_HZH					0x08    //Magnetometer Z-Axis High Byte

ESOS_USER_TASK(ReadMagnetometer)
{
	static UINT16 U16_MagX, U16_MagY, U16_MagZ;
	static uint8 u8_WIA, u8_ST1, u8_ST2;
	ESOS_TASK_BEGIN();

	// Set clock to Phase Locked Loop - Gyroscop X-Axis Reference
	ESOS_TASK_WAIT_ON_WRITE2I2C1(I2C_WADDR(MPU_SLAVE_ADDRESS), MPU_REG_PWR_MGMT_1, MPU_PWR_MGMT_CLKSEL_PLL_X);

	// Enable I2C Bypass to talk to the AK8975 Magnetometer
	ESOS_TASK_WAIT_ON_WRITE2I2C1(I2C_WADDR(MPU_SLAVE_ADDRESS), MPU_REG_INT_PIN_CFG, MPU_INT_PIN_CFG_I2C_BYPASS_EN);

	// Poke the AK8975 to see its Who Am I (WIA), which should return 0x48
	ESOS_TASK_WAIT_ON_WRITE1I2C1(I2C_WADDR(MPU_MAG_I2C_ADDR), MPU_REG_MAG_WIA);
	ESOS_TASK_WAIT_ON_READ1I2C1(I2C_RADDR(MPU_MAG_I2C_ADDR), u8_WIA);

	// Print off WIA Register Reading
	ESOS_TASK_WAIT_ON_AVAILABLE_OUT_COMM();
	ESOS_TASK_WAIT_ON_SEND_STRING("MPU> MAG> WIA Register: Expected 0x48, Read ");
    ESOS_TASK_WAIT_ON_SEND_UINT8_AS_HEX_STRING(u8_WIA);
	ESOS_TASK_WAIT_ON_SEND_UINT8('\n');
    ESOS_TASK_SIGNAL_AVAILABLE_OUT_COMM();

	while(TRUE)
	{
		// Tell AK8975 to take a measurement
		ESOS_TASK_WAIT_ON_WRITE2I2C1(I2C_WADDR(MPU_MAG_I2C_ADDR), MPU_REG_MAG_CNTL, MPU_MAG_SINGLE_MRMNT);

		// Wait for data to return
		do
		{
			ESOS_TASK_WAIT_ON_WRITE1I2C1(I2C_WADDR(MPU_MAG_I2C_ADDR), MPU_REG_MAG_ST1);
			ESOS_TASK_WAIT_ON_READ1I2C1(I2C_RADDR(MPU_MAG_I2C_ADDR), u8_ST1);
			ESOS_TASK_YIELD();
		} while(u8_ST1 != MPU_MAG_ST1_READY);

		// Read ST2 Status
		ESOS_TASK_WAIT_ON_WRITE1I2C1(I2C_WADDR(MPU_MAG_I2C_ADDR), MPU_REG_MAG_ST2);
		ESOS_TASK_WAIT_ON_READ1I2C1(I2C_RADDR(MPU_MAG_I2C_ADDR), u8_ST2);

		// Print ST2 Status
		ESOS_TASK_WAIT_ON_AVAILABLE_OUT_COMM();
		ESOS_TASK_WAIT_ON_SEND_STRING("\nMPU> MAG> Status 2: ");
		ESOS_TASK_WAIT_ON_SEND_UINT8_AS_HEX_STRING(u8_ST2);
		if(u8_ST2 & MPU_MAG_ST2_DERR)
			ESOS_TASK_WAIT_ON_SEND_STRING("\nMPU> MAG> Data Read Error occurred.");
		if(u8_ST2 & MPU_MAG_ST2_HOFL)
			ESOS_TASK_WAIT_ON_SEND_STRING("\nMPU> MAG> Magnetic sensor overflow occurred.");
		ESOS_TASK_WAIT_ON_SEND_UINT8('\n');
		ESOS_TASK_SIGNAL_AVAILABLE_OUT_COMM();

		/*********************************************************************
		 * Page 21/33 of Data Sheet:
		 * Addresses from 02H to 09H and from 10H to 12H are compliant with
		 * automatic increment function of serial interface respectively.
		 *********************************************************************/
		
		// Set Register to HXL (First register for measurement data)
		ESOS_TASK_WAIT_ON_WRITE1I2C1(I2C_WADDR(MPU_MAG_I2C_ADDR), MPU_REG_MAG_HXL);

		// Read Magnetometer X-Axis
		ESOS_TASK_WAIT_ON_READ2I2C1(I2C_RADDR(MPU_MAG_I2C_ADDR), U16_MagX.u8Lsb, U16_MagX.u8Msb);

		// Read Magnetometer Y-Axis
		ESOS_TASK_WAIT_ON_READ2I2C1(I2C_RADDR(MPU_MAG_I2C_ADDR), U16_MagY.u8Lsb, U16_MagY.u8Msb);

		// Read Magnetometer Z-Axis
		ESOS_TASK_WAIT_ON_READ2I2C1(I2C_RADDR(MPU_MAG_I2C_ADDR), U16_MagZ.u8Lsb, U16_MagZ.u8Msb);


		// Print raw measurement data
		ESOS_TASK_WAIT_ON_AVAILABLE_OUT_COMM();
		ESOS_TASK_WAIT_ON_SEND_STRING("MPU> MAG> X-Axis: ");
		ESOS_TASK_WAIT_ON_SEND_UINT16_AS_HEX_STRING(U16_MagX.u16);
		ESOS_TASK_WAIT_ON_SEND_STRING("\nMPU> MAG> Y-Axis: ");
		ESOS_TASK_WAIT_ON_SEND_UINT16_AS_HEX_STRING(U16_MagY.u16);
		ESOS_TASK_WAIT_ON_SEND_STRING("\nMPU> MAG> Z-Axis: ");
		ESOS_TASK_WAIT_ON_SEND_UINT16_AS_HEX_STRING(U16_MagZ.u16);
		ESOS_TASK_WAIT_ON_SEND_UINT8('\n');
		ESOS_TASK_SIGNAL_AVAILABLE_OUT_COMM();

		// Delay 500ms
		ESOS_TASK_WAIT_TICKS(500);
	}

	ESOS_TASK_END();
}

void user_init(void)
{
	__esos_unsafe_PutString(HELLO_MSG);
	esos_pic24_configI2C1(400);
	esos_RegisterTask(ReadMagnetometer);
}
