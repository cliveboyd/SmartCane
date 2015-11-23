/* Copyright (c) <2015> <Shun Bai (wanyancan at gmail)>

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
 
 
#include "ltc2943.h"

#ifndef u8
typedef uint8_t     	u8;
typedef uint16_t    	u16;
typedef uint32_t    	u32;
typedef uint64_t    	u64;
typedef int32_t			s32;
typedef volatile u8     vu8;
typedef volatile u32    vu32;
typedef volatile u64    vu64;
#endif

#include "nrf_delay.h"
#include "Communication.h"


/**
 * struct i2c_msg - an I2C transaction segment beginning with START
 * @addr: Slave address, either seven or ten bits.  When this is a ten
 *      bit address, I2C_M_TEN must be set in @flags and the adapter
 *      must support I2C_FUNC_10BIT_ADDR.
 * @flags: I2C_M_RD is handled by all adapters.  No other flags may be
 *      provided unless the adapter exported the relevant I2C_FUNC_*
 *      flags through i2c_check_functionality().
 * @len: Number of data bytes in @buf being read from or written to the
 *      I2C slave address.  For read transactions where I2C_M_RECV_LEN
 *      is set, the caller guarantees that this buffer can hold up to
 *      32 bytes in addition to the initial length byte sent by the
 *      slave (plus, if used, the SMBus PEC); and this value will be
 *      incremented by the number of block data bytes received.
 * @buf: The buffer into which data is read, or from which it's written.
 *
 * An i2c_msg is the low level representation of one segment of an I2C
 * transaction.  It is visible to drivers in the @i2c_transfer() procedure,
 * to userspace from i2c-dev, and to I2C adapter drivers through the
 * @i2c_adapter.@master_xfer() method.
 *
 * Except when I2C "protocol mangling" is used, all I2C adapters implement
 * the standard rules for I2C transactions.  Each transaction begins with a
 * START.  That is followed by the slave address, and a bit encoding read
 * versus write.  Then follow all the data bytes, possibly including a byte
 * with SMBus PEC.  The transfer terminates with a NAK, or when all those
 * bytes have been transferred and ACKed.  If this is the last message in a
 * group, it is followed by a STOP.  Otherwise it is followed by the next
 * @i2c_msg transaction segment, beginning with a (repeated) START.
 *
 * Alternatively, when the adapter supports I2C_FUNC_PROTOCOL_MANGLING then
 * passing certain @flags may have changed those standard protocol behaviors.
 * Those flags are only for use with broken/nonconforming slaves, and with
 * adapters which are known to support the specific mangling options they
 * need (one or more of IGNORE_NAK, NO_RD_ACK, NOSTART, and REV_DIR_ADDR).
 */
struct i2c_msg {
        u16 addr;     /* slave address  */
        u16 flags;
#define I2C_M_TEN               0x0010  /* this is a ten bit chip address 		*/
#define I2C_M_RD                0x0001  /* read data, from slave to master 		*/
#define I2C_M_STOP              0x8000  /* if I2C_FUNC_PROTOCOL_MANGLING 		*/
#define I2C_M_NOSTART           0x4000  /* if I2C_FUNC_NOSTART 					*/
#define I2C_M_REV_DIR_ADDR      0x2000  /* if I2C_FUNC_PROTOCOL_MANGLING 		*/
#define I2C_M_IGNORE_NAK        0x1000  /* if I2C_FUNC_PROTOCOL_MANGLING 		*/
#define I2C_M_NO_RD_ACK         0x0800  /* if I2C_FUNC_PROTOCOL_MANGLING 		*/
#define I2C_M_RECV_LEN          0x0400  /* length will be first received byte 	*/
        u16 len;              /* msg length                           			*/
        u8 *buf;              /* pointer to msg data                  			*/
};


/**
 * struct i2c_client - represent an I2C slave device
 * @flags: I2C_CLIENT_TEN indicates the device uses a ten bit chip address;
 *      I2C_CLIENT_PEC indicates it uses SMBus Packet Error Checking
 * @addr: Address used on the I2C bus connected to the parent adapter.
 * @name: Indicates the type of the device, usually a chip name that's
 *      generic enough to hide second-sourcing and compatible revisions.
 * @adapter: manages the bus segment hosting this I2C device
 * @dev: Driver model device node for the slave.
 * @irq: indicates the IRQ generated by this device (if any)
 * @detected: member of an i2c_driver.clients list or i2c-core's
 *      userspace_devices list
 * @slave_cb: Callback when I2C slave mode of an adapter is used. The adapter
 *      calls it to pass on slave events to the slave driver.
 *
 * An i2c_client identifies a single device (i.e. chip) connected to an
 * i2c bus. The behaviour exposed to Linux is defined by the driver
 * managing the device.
 */
struct i2c_client {
        unsigned short flags;           /* div., see below              */
        unsigned short addr;            /* chip address - NOTE: 7bit    */
                                        /* addresses are stored in the  */
                                        /* _LOWER_ 7 bits               */
#define I2C_NAME_SIZE   20	
        char name[I2C_NAME_SIZE];
//        struct i2c_adapter *adapter;    /* the adapter we sit on      */
//        struct device dev;              /* the device structure       */
//        int irq;                        /* irq issued by device       */
//        struct list_head detected;
#ifdef  CONFIG_I2C_SLAVE 
        i2c_slave_cb_t slave_cb;        /* callback for slave mode      */
#endif
};

struct i2c_client ltc2943_client;

enum power_supply_property {
        /* Properties of type `int' */
        POWER_SUPPLY_PROP_STATUS = 0,
        POWER_SUPPLY_PROP_CHARGE_TYPE,
        POWER_SUPPLY_PROP_HEALTH,
        POWER_SUPPLY_PROP_PRESENT,
        POWER_SUPPLY_PROP_ONLINE,
        POWER_SUPPLY_PROP_AUTHENTIC,
        POWER_SUPPLY_PROP_TECHNOLOGY,
        POWER_SUPPLY_PROP_CYCLE_COUNT,
        POWER_SUPPLY_PROP_VOLTAGE_MAX,
        POWER_SUPPLY_PROP_VOLTAGE_MIN,
        POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
        POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
        POWER_SUPPLY_PROP_VOLTAGE_NOW,
        POWER_SUPPLY_PROP_VOLTAGE_AVG,
        POWER_SUPPLY_PROP_VOLTAGE_OCV,
        POWER_SUPPLY_PROP_VOLTAGE_BOOT,
        POWER_SUPPLY_PROP_CURRENT_MAX,
        POWER_SUPPLY_PROP_CURRENT_NOW,
        POWER_SUPPLY_PROP_CURRENT_AVG,
        POWER_SUPPLY_PROP_CURRENT_BOOT,
        POWER_SUPPLY_PROP_POWER_NOW,
        POWER_SUPPLY_PROP_POWER_AVG,
        POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
        POWER_SUPPLY_PROP_CHARGE_EMPTY_DESIGN,
        POWER_SUPPLY_PROP_CHARGE_FULL,
        POWER_SUPPLY_PROP_CHARGE_EMPTY,
        POWER_SUPPLY_PROP_CHARGE_NOW,
        POWER_SUPPLY_PROP_CHARGE_AVG,
        POWER_SUPPLY_PROP_CHARGE_COUNTER,
        POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
        POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
        POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
        POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
        POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
        POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
        POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
        POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN,
        POWER_SUPPLY_PROP_ENERGY_EMPTY_DESIGN,
        POWER_SUPPLY_PROP_ENERGY_FULL,
        POWER_SUPPLY_PROP_ENERGY_EMPTY,
        POWER_SUPPLY_PROP_ENERGY_NOW,
        POWER_SUPPLY_PROP_ENERGY_AVG,
        POWER_SUPPLY_PROP_CAPACITY, /* in percents! */
        POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN, /* in percents! */
        POWER_SUPPLY_PROP_CAPACITY_ALERT_MAX, /* in percents! */
        POWER_SUPPLY_PROP_CAPACITY_LEVEL,
        POWER_SUPPLY_PROP_TEMP,
        POWER_SUPPLY_PROP_TEMP_MAX,
        POWER_SUPPLY_PROP_TEMP_MIN,
        POWER_SUPPLY_PROP_TEMP_ALERT_MIN,
        POWER_SUPPLY_PROP_TEMP_ALERT_MAX,
        POWER_SUPPLY_PROP_TEMP_AMBIENT,
        POWER_SUPPLY_PROP_TEMP_AMBIENT_ALERT_MIN,
        POWER_SUPPLY_PROP_TEMP_AMBIENT_ALERT_MAX,
        POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
        POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
        POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
        POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
        POWER_SUPPLY_PROP_TYPE, /* use power_supply.type instead */
        POWER_SUPPLY_PROP_SCOPE,
        POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
        POWER_SUPPLY_PROP_CALIBRATE,
        /* Properties of type `const char *' */
        POWER_SUPPLY_PROP_MODEL_NAME,
        POWER_SUPPLY_PROP_MANUFACTURER,
        POWER_SUPPLY_PROP_SERIAL_NUMBER,
};

enum power_supply_type {
        POWER_SUPPLY_TYPE_UNKNOWN = 0,
        POWER_SUPPLY_TYPE_BATTERY,
        POWER_SUPPLY_TYPE_UPS,
        POWER_SUPPLY_TYPE_MAINS,
        POWER_SUPPLY_TYPE_USB,          /* Standard Downstream Port */
        POWER_SUPPLY_TYPE_USB_DCP,      /* Dedicated Charging Port */
        POWER_SUPPLY_TYPE_USB_CDP,      /* Charging Downstream Port */
        POWER_SUPPLY_TYPE_USB_ACA,      /* Accessory Charger Adapters */
};

enum power_supply_notifier_events {
        PSY_EVENT_PROP_CHANGED,
};

union power_supply_propval {
        int intval;
        const char *strval;
};

//struct device_node;
//struct power_supply;
//struct device {
//	int rev;
//};

///* Run-time specific power supply configuration */
//struct power_supply_config {
//        struct device_node *of_node;
//        /* Driver private data */
//        void *drv_data;

//        char **supplied_to;
//        size_t num_supplicants;
//};

/* Description of power supply */
struct power_supply_desc {
//        const char *name;
        enum power_supply_type type;
        enum power_supply_property *properties;
        size_t num_properties;

//        /*
//         * Functions for drivers implementing power supply class.
//         * These shouldn't be called directly by other drivers for accessing
//         * this power supply. Instead use power_supply_*() functions (for
//         * example power_supply_get_property()).
//         */
//        int (*get_property)(struct power_supply *psy,
//                            enum power_supply_property psp,
//                            union power_supply_propval *val);
//        int (*set_property)(struct power_supply *psy,
//                            enum power_supply_property psp,
//                            const union power_supply_propval *val);
//        /*
//         * property_is_writeable() will be called during registration
//         * of power supply. If this happens during device probe then it must
//         * not access internal data of device (because probe did not end).
//         */
//        int (*property_is_writeable)(struct power_supply *psy,
//                                     enum power_supply_property psp);
//        void (*external_power_changed)(struct power_supply *psy);
//        void (*set_charged)(struct power_supply *psy);

//        /*
//         * Set if thermal zone should not be created for this power supply.
//         * For example for virtual supplies forwarding calls to actual
//         * sensors or other supplies.
//         */
//        bool no_thermal;
//        /* For APM emulation, think legacy userspace. */
//        int use_for_apm;
};

//struct power_supply {
//        const struct power_supply_desc *desc;

//        char **supplied_to;
//        size_t num_supplicants;

//        char **supplied_from;
//        size_t num_supplies;
//        struct device_node *of_node;

//        /* Driver private data */
//        void *drv_data;

//        /* private */
//        struct device dev;
////        struct work_struct changed_work;
////        struct delayed_work deferred_register_work;
////        spinlock_t changed_lock;
//        bool changed;
////        atomic_t use_cnt;
//#ifdef CONFIG_THERMAL
//        struct thermal_zone_device *tzd;
//        struct thermal_cooling_device *tcd;
//#endif

//#ifdef CONFIG_LEDS_TRIGGERS
//        struct led_trigger *charging_full_trig;
//        char *charging_full_trig_name;
//        struct led_trigger *charging_trig;
//        char *charging_trig_name;
//        struct led_trigger *full_trig;
//        char *full_trig_name;
//        struct led_trigger *online_trig;
//        char *online_trig_name;
//        struct led_trigger *charging_blink_full_solid_trig;
//        char *charging_blink_full_solid_trig_name;
//#endif
//};

#define pr_err(format, ...) 		do {} while (0) //fprintf (stderr, format, ## __VA_ARGS__)
#ifdef DEBUG
#define pr_debug(format, ...) 		do {} while (0) //fprintf (stderr, format, ## __VA_ARGS__)
#else
#define pr_debug(format, ...) 		do {} while (0)
#endif
#define dev_err(dev, format, ...) 	do {} while (0) //fprintf (stderr, format, ## __VA_ARGS__)
#define dev_warn(dev, format, ...) 	do {} while (0) //fprintf (stderr, format, ## __VA_ARGS__)
#define dev_dbg(dev, format, ...) 	do {} while (0) //fprintf (stderr, format, ## __VA_ARGS__)
	 
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]) )	
#define I16_MSB(x)                      ((x >> 8) & 0xFF)
#define I16_LSB(x)                      (x & 0xFF)

#define LTC294X_WORK_DELAY              10      /* Update delay in seconds */

#define LTC294X_MAX_VALUE               0xFFFF
#define LTC294X_MID_SUPPLY              0x7FFF

#define LTC2941_MAX_PRESCALER_EXP       7
#define LTC2943_MAX_PRESCALER_EXP       6
#define LTC2943_ADDRESS					0x64

enum ltc294x_reg {
        LTC294X_REG_STATUS              = 0x00,
        LTC294X_REG_CONTROL             = 0x01,
        LTC294X_REG_ACC_CHARGE_MSB      = 0x02,
        LTC294X_REG_ACC_CHARGE_LSB      = 0x03,
        LTC294X_REG_THRESH_HIGH_MSB     = 0x04,
        LTC294X_REG_THRESH_HIGH_LSB     = 0x05,
        LTC294X_REG_THRESH_LOW_MSB      = 0x06,
        LTC294X_REG_THRESH_LOW_LSB      = 0x07,
        LTC294X_REG_VOLTAGE_MSB 		= 0x08,
        LTC294X_REG_VOLTAGE_LSB 		= 0x09,
		LTC294X_REG_VOLTAGE_HIGH_MSB 	= 0x0a,
		LTC294X_REG_VOLTAGE_HIGH_LSB 	= 0x0b,
		LTC294X_REG_VOLTAGE_LOW_MSB 	= 0x0c,
		LTC294X_REG_VOLTAGE_LOW_LSB 	= 0x0d,		
        LTC294X_REG_CURRENT_MSB 		= 0x0e,
        LTC294X_REG_CURRENT_LSB 		= 0x0f,
		LTC294X_REG_CURRENT_HIGH_MSB 	= 0x10,
		LTC294X_REG_CURRENT_HIGH_LSB 	= 0x11,
		LTC294X_REG_CURRENT_LOW_MSB	 	= 0x12,
		LTC294X_REG_CURRENT_LOW_LSB 	= 0x13,			
        LTC294X_REG_TEMPERATURE_MSB     = 0x14,
        LTC294X_REG_TEMPERATURE_LSB     = 0x15,
		LTC294X_REG_TEMPERATURE_HIGH 	= 0x16,
		LTC294X_REG_TEMPERATURE_LOW 	= 0x17
};

#define EINVAL         22      /* Invalid argument */
#define EPERM          23      /* Invalid permission */

#define BIT(nr)                 (1UL << (nr))
#define LTC2943_REG_CONTROL_MODE_MASK 			(BIT(7) | BIT(6))
#define LTC2943_REG_CONTROL_MODE_SCAN 			(BIT(7))
#define LTC2943_REG_CONTROL_MODE_MANUAL 		(BIT(6))
#define LTC2943_REG_CONTROL_MODE_AUTO 			(BIT(7) | BIT(6))
#define LTC294X_REG_CONTROL_PRESCALER_MASK      (BIT(5) | BIT(4) | BIT(3))
#define LTC294X_REG_CONTROL_SHUTDOWN_MASK       (BIT(0))
#define LTC294X_REG_CONTROL_PRESCALER_SET(x) \
        ((x << 3) & LTC294X_REG_CONTROL_PRESCALER_MASK)
#define LTC294X_REG_CONTROL_ALCC_CONFIG_DISABLED        0
#define LTC294X_REG_CONTROL_ALCC_CONFIG_OUTPUT_ALERT        (BIT(2))
#define LTC294X_REG_CONTROL_ALCC_CONFIG_INPUT_RESET_CHARGE  (BIT(1))

#define LTC2941_NUM_REGS        0x08
#define LTC2943_NUM_REGS        0x18

struct ltc294x_info {
        struct i2c_client *client;      /* I2C Client pointer */
//      struct power_supply *supply;    /* Supply pointer */
        struct power_supply_desc supply_desc;   /* Supply description */
//      struct delayed_work work;       /* Work scheduler */
        int num_regs;   /* Number of registers (chip type) */
        int id;         /* Identifier of ltc294x chip */
        int charge;     /* Last charge register content */
        int r_sense;    /* mOhm */
        int Qlsb;       /* nAh */
};

struct ltc294x_info ltc2943_info;

//static DEFINE_IDR(ltc294x_id);
//static DEFINE_MUTEX(ltc294x_lock);

static inline int convert_bin_to_uAh(
        const struct ltc294x_info *info, int Q)
{
        return ((Q * (info->Qlsb / 10))) / 100;
}

static inline int convert_uAh_to_bin(
        const struct ltc294x_info *info, int uAh)
{
        int Q;

        Q = (uAh * 100) / (info->Qlsb/10);
        return (Q < LTC294X_MAX_VALUE) ? Q : LTC294X_MAX_VALUE;
}

static int i2c_transfer(struct i2c_msg* pMsg, int numOfMsg)
{
	bool stopSign = false;
	for(int i=0;i<numOfMsg;i++)
	{
		if(i==numOfMsg-1)
			stopSign = true;
		if(pMsg->flags & I2C_M_RD) {
			if(!I2C_Read(pMsg->addr, pMsg->buf, pMsg->len, stopSign))
				return -1;
		}else {
			if(!I2C_Write(pMsg->addr, pMsg->buf, pMsg->len, stopSign))
				return -1;
		}
		pMsg++;
	}
	return 0;
}
static int i2c_smbus_write_i2c_block_data(struct i2c_client *client, 
		u8 reg_start, int num_regs, const u8 *buf)
{
	if(I2C_Write(client->addr, (unsigned char*)buf, num_regs, true))
		return 0;
	return -1;
}

static int ltc294x_read_regs(struct i2c_client *client,
        enum ltc294x_reg reg, u8 *buf, int num_regs)
{
        int ret;
        struct i2c_msg msgs[2] ;
        u8 reg_start = reg;

        msgs[0].addr    = client->addr;
        msgs[0].len     = 1;
        msgs[0].buf     = &reg_start;

        msgs[1].addr    = client->addr;
        msgs[1].len     = num_regs;
        msgs[1].buf     = buf;
        msgs[1].flags   = I2C_M_RD;

        ret = i2c_transfer(&msgs[0], 2);
        if (ret < 0) {
                dev_err(&client->dev, "ltc2941 read_reg failed!\n");
                return ret;
        }

        dev_dbg(&client->dev, "%s (%#x, %d) -> %#x\n",
                __func__, reg, num_regs, *buf);

        return 0;
}

static int ltc294x_write_regs(struct i2c_client *client,
        enum ltc294x_reg reg, const u8 *buf, int num_regs)
{
	
//        int ret;
//        u8 reg_start = reg;

//        ret = i2c_smbus_write_i2c_block_data(client, reg_start, num_regs, buf);
//        if (ret < 0) {
//                dev_err(&client->dev, "ltc2941 write_reg failed!\n");
//                return ret;
//        }

//        dev_dbg(&client->dev, "%s (%#x, %d) -> %#x\n",
//                __func__, reg, num_regs, *buf);

		int ret;
        struct i2c_msg msgs[1];
		u8 localbuf[20]; // enough for most I2C writing
		localbuf[0] = reg;
		
		memcpy(&localbuf[1],buf,num_regs);
        u8 reg_start = reg;

        msgs[0].addr    = client->addr;
        msgs[0].len     = num_regs+1;
        msgs[0].buf     = localbuf;
		msgs[0].flags	= 0;
        ret = i2c_transfer(&msgs[0], 1);
        if (ret < 0) {
                dev_err(&client->dev, "ltc2941 read_reg failed!\n");
                return ret;
        }

        dev_dbg(&client->dev, "%s (%#x, %d) -> %#x\n",
                __func__, reg, num_regs, *buf);

		
        return 0;
}

int ltc294x_reset( int prescaler_exp)
{
		struct ltc294x_info *info = &ltc2943_info;
        int ret;
        u8 value[3];
        u8 control;

        /* Read status and control registers */
	ret = ltc294x_read_regs(info->client, LTC294X_REG_CONTROL, &value[0], 1);
        if (ret < 0) {
                dev_err(&info->client->dev,
                        "Could not read registers from device\n");
                goto error_exit;
        }

        control = LTC294X_REG_CONTROL_PRESCALER_SET(prescaler_exp) |
                                LTC294X_REG_CONTROL_ALCC_CONFIG_OUTPUT_ALERT;
        /* Set the 2943 mode*/
        if (info->num_regs == LTC2943_NUM_REGS)
                control |= LTC2943_REG_CONTROL_MODE_SCAN;

		do {
			if (value[0] != control) {
					ret = ltc294x_write_regs(info->client,
							LTC294X_REG_CONTROL, &control, 1);
					if (ret < 0) {
							dev_err(&info->client->dev,
									"Could not write register\n");
							goto error_exit;
					}
			}
			/* Read status and control registers again */
			ret = ltc294x_read_regs(info->client, LTC294X_REG_CONTROL, &value[0], 1);
			if (ret < 0) {
					dev_err(&info->client->dev,
							"Could not read registers from device\n");
					goto error_exit;
			}
		}
		while(value[0]!= control);
		

        return 0;

error_exit:
        return ret;
}

int ltc294x_read_charge_register(void)
{
		struct ltc294x_info *info = &ltc2943_info;
        int ret;
        u8 datar[2];

        ret = ltc294x_read_regs(info->client,
                LTC294X_REG_ACC_CHARGE_MSB, &datar[0], 2);
        if (ret < 0)
                return ret;
        return (datar[0] << 8) + datar[1];
}

int ltc294x_get_charge_now(int *val)
{
		struct ltc294x_info *info = &ltc2943_info;
        int value = ltc294x_read_charge_register();

        if (value < 0)
                return value;
        /* When r_sense < 0, this counts up when the battery discharges */
        if (info->Qlsb < 0)
                value -= 0xFFFF;
        *val = convert_bin_to_uAh(info, value);
        return 0;
}

int ltc294x_set_charge_now(int val)
{
		struct ltc294x_info *info = &ltc2943_info;
        int ret;
        u8 dataw[2];
        u8 ctrl_reg;
        s32 value;

        value = convert_uAh_to_bin(info, val);
        /* Direction depends on how sense+/- were connected */
        if (info->Qlsb < 0)
                value += 0xFFFF;
        if ((value < 0) || (value > 0xFFFF)) /* input validation */
                return -EINVAL;

        /* Read control register */
        ret = ltc294x_read_regs(info->client,
                LTC294X_REG_CONTROL, &ctrl_reg, 1);
        if (ret < 0)
                return ret;
        /* Disable analog section */
        ctrl_reg |= LTC294X_REG_CONTROL_SHUTDOWN_MASK;
        ret = ltc294x_write_regs(info->client,
                LTC294X_REG_CONTROL, &ctrl_reg, 1);
        if (ret < 0)
                return ret;
        /* Set new charge value */
        dataw[0] = I16_MSB(value);
        dataw[1] = I16_LSB(value);
        ret = ltc294x_write_regs(info->client,
                LTC294X_REG_ACC_CHARGE_MSB, &dataw[0], 2);
        if (ret < 0)
                goto error_exit;
        /* Enable analog section */
error_exit:
        ctrl_reg &= ~LTC294X_REG_CONTROL_SHUTDOWN_MASK;
        ret = ltc294x_write_regs(info->client,
                LTC294X_REG_CONTROL, &ctrl_reg, 1);

        return ret < 0 ? ret : 0;
}

int ltc294x_get_charge_counter(int *val)
{
		struct ltc294x_info *info = &ltc2943_info;
        int value = ltc294x_read_charge_register();

        if (value < 0)
                return value;
        value -= LTC294X_MID_SUPPLY;
        *val = convert_bin_to_uAh(info, value);
        return 0;
}

int ltc294x_get_voltage(int *val)
{
		struct ltc294x_info *info = &ltc2943_info;
        int ret;
        u8 datar[2];
        u32 value;

        ret = ltc294x_read_regs(info->client,
                LTC294X_REG_VOLTAGE_MSB, &datar[0], 2);
        value = (datar[0] << 8) | datar[1];
        *val = ((value * 23600) / 0xFFFF) * 1000; /* in uV */
        return ret;
}

int ltc294x_get_current( int *val)
{
	struct ltc294x_info *info = &ltc2943_info;
    int ret,i;
    u8 datar[2];
    s32 value;
	
	
	ret = ltc294x_read_regs(info->client,
			LTC294X_REG_CURRENT_MSB, &datar[0], 2);
	value = (datar[0] << 8) | datar[1];
	value -= 0x7FFF;
	/* Value is in range -32k..+32k, r_sense is usually 10..50 mOhm,
	 * the formula below keeps everything in s32 range while preserving
	 * enough digits */
	*val = 1000 * ((60000 * value) / (info->r_sense * 0x7FFF)); /* in uA */
	return ret;
}

int ltc294x_get_temperature(int *val)
{
		struct ltc294x_info *info = &ltc2943_info;
        int ret;
        u8 datar[2];
        u32 value;

        ret = ltc294x_read_regs(info->client,
                LTC294X_REG_TEMPERATURE_MSB, &datar[0], 2);
        value = (datar[0] << 8) | datar[1];
        /* Full-scale is 510 Kelvin, convert to centidegrees  */
        *val = (((51000 * value) / 0xFFFF) - 27215);
        return ret;
}

void ltc294x_update(void)
{
	struct ltc294x_info *info = &ltc2943_info;
	
	int charge = ltc294x_read_charge_register();

	if (charge != info->charge) {
			info->charge = charge;
	}
}


static enum power_supply_property ltc294x_properties[] = {
        POWER_SUPPLY_PROP_CHARGE_COUNTER,
        POWER_SUPPLY_PROP_CHARGE_NOW,
        POWER_SUPPLY_PROP_VOLTAGE_NOW,
        POWER_SUPPLY_PROP_CURRENT_NOW,
        POWER_SUPPLY_PROP_TEMP,
};

int ltc294x_init(void)
{

	struct i2c_client *client = &ltc2943_client;
	
	ltc2943_client.addr = LTC2943_ADDRESS;
	struct ltc294x_info *info = &ltc2943_info;
	
	int ret;

    u32 prescaler_exp;
    s32 r_sense;

    info->num_regs = LTC2943_NUM_REGS;

	/* r_sense can be negative, when sense+ is connected to the battery
	 * instead of the sense-. This results in reversed measurements. */
    info->r_sense = 20; // in mOhm
	
	prescaler_exp = LTC2943_MAX_PRESCALER_EXP;


	if (info->num_regs == LTC2943_NUM_REGS) {
			if (prescaler_exp > LTC2943_MAX_PRESCALER_EXP)
					prescaler_exp = LTC2943_MAX_PRESCALER_EXP;
			info->Qlsb = ((340 * 50000) / r_sense) /
							(4096 / (1 << (2*prescaler_exp)));
	} else {
			if (prescaler_exp > LTC2941_MAX_PRESCALER_EXP)
					prescaler_exp = LTC2941_MAX_PRESCALER_EXP;
			info->Qlsb = ((85 * 50000) / r_sense) /
							(128 / (1 << prescaler_exp));
	}

	info->client = client;

	info->supply_desc.type = POWER_SUPPLY_TYPE_BATTERY;
	info->supply_desc.properties = ltc294x_properties;
	if (info->num_regs >= LTC294X_REG_TEMPERATURE_LSB)
			info->supply_desc.num_properties =
					ARRAY_SIZE(ltc294x_properties);
	else if (info->num_regs >= LTC294X_REG_CURRENT_LSB)
			info->supply_desc.num_properties =
					ARRAY_SIZE(ltc294x_properties) - 1;
	else if (info->num_regs >= LTC294X_REG_VOLTAGE_LSB)
			info->supply_desc.num_properties =
					ARRAY_SIZE(ltc294x_properties) - 2;
	else
			info->supply_desc.num_properties =
					ARRAY_SIZE(ltc294x_properties) - 3;


        ret = ltc294x_reset(prescaler_exp);
        if (ret < 0) {
                dev_err(&client->dev, "Communication with chip failed\n");
                goto fail_comm;
        }


        return 0;

fail_comm:
        return ret;
}

#ifdef CONFIG_PM_SLEEP

static int ltc294x_suspend(struct device *dev)
{
        struct i2c_client *client = to_i2c_client(dev);
        struct ltc294x_info *info = i2c_get_clientdata(client);

        cancel_delayed_work(&info->work);
        return 0;
}

static int ltc294x_resume(struct device *dev)
{
        struct i2c_client *client = to_i2c_client(dev);
        struct ltc294x_info *info = i2c_get_clientdata(client);

        schedule_delayed_work(&info->work, LTC294X_WORK_DELAY * HZ);
        return 0;
}

static SIMPLE_DEV_PM_OPS(ltc294x_pm_ops, ltc294x_suspend, ltc294x_resume);
#define LTC294X_PM_OPS (&ltc294x_pm_ops)

#else
#define LTC294X_PM_OPS NULL
#endif /* CONFIG_PM_SLEEP */

