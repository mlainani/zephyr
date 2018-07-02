/*
 * Copyright (c) 2018 Madani Lainani.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "i2c-priv.h"

#include <board.h>
#include <device.h>
#include <errno.h>
#include <soc.h>

#define I2C_BUS_STATE_UNKNOWN           0x0
#define I2C_BUS_STATE_IDLE              0x1
#define I2C_BUS_STATE_OWNER             0x2
#define I2C_BUS_STATE_BUSY              0x3

#define I2C_MASTER_ACTION_REPEAT_START  0x1
#define I2C_MASTER_ACTION_READ          0x2
#define I2C_MASTER_ACTION_STOP          0x3

#define I2C_MASTER_ACK_ACTION_ACK       0x0
#define I2C_MASTER_ACK_ACTION_NACK      0x1

#define div_ceil(a, b)  (((a) + (b) - 1) / (b))

#if CONFIG_SYS_LOG_I2C_LEVEL == SYS_LOG_LEVEL_DEBUG
static char *bus_state_str[] = {"Unknown", "Idle", "Owner", "Busy"};
#endif

/* Device constant configuration parameters */
struct i2c_sam0_dev_cfg {
	SercomI2cm *regs;
	u32_t pm_apbcmask;
	u16_t gclk_clkctrl_id;
	u32_t bitrate;
	u32_t rise_time_ns;
};

/* Device run time data */
struct i2c_sam0_dev_data {
	struct k_mutex mutex;
};

#define DEV_CFG(dev) \
	((const struct i2c_sam0_dev_cfg *const)(dev)->config->config_info)
#define DEV_DATA(dev) ((struct i2c_sam0_dev_data *const)(dev)->driver_data)

static void wait_synchronization(SercomI2cm *const i2cm)
{
	/* SYNCBUSY is a register */
	while (i2cm->SYNCBUSY.reg & SERCOM_I2CM_SYNCBUSY_MASK) {
	}
}

static int i2c_sam0_set_bitrate(SercomI2cm *const i2cm, u32_t bitrate,
				u32_t rise_time_ns)
{
	i2cm->BAUD.bit.BAUD = CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC /
			      ((2 * bitrate) - 5 - (((CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / 1000000)
						     * rise_time_ns) / (2 * 1000)));
	return 0;
}

static int i2c_sam0_configure(struct device *dev, u32_t dev_config)
{
	const struct i2c_sam0_dev_cfg *cfg = DEV_CFG(dev);
	SercomI2cm *const i2cm = cfg->regs;
	u32_t bitrate;
	int retval;

	/* Set Master mode */
	if (!(dev_config & I2C_MODE_MASTER)) {
		SYS_LOG_ERR("slave mode is not supported");
		return -EINVAL;
	}

	i2cm->CTRLA.reg =
		SERCOM_I2CM_CTRLA_MODE(SERCOM_I2CM_CTRLA_MODE_I2C_MASTER_Val);

	if (dev_config & I2C_ADDR_10_BITS) {
		SYS_LOG_ERR("10-bit addressing mode is not supported");
		return -EINVAL;
	}

	/* Configure clock */
	switch (I2C_SPEED_GET(dev_config)) {
	case I2C_SPEED_STANDARD:
		bitrate = I2C_BITRATE_STANDARD;
		break;
	default:
		SYS_LOG_ERR("unsupported speed value");
		return -EINVAL;
	}

	retval = i2c_sam0_set_bitrate(i2cm, bitrate, cfg->rise_time_ns);

	return 0;
}

static int i2c_sam0_init(struct device *dev)
{
	const struct i2c_sam0_dev_cfg *cfg = DEV_CFG(dev);
	struct i2c_sam0_dev_data *dev_data = DEV_DATA(dev);
	SercomI2cm *const i2cm = cfg->regs;
	u32_t bitrate_cfg;
	int retval;

	k_mutex_init(&dev_data->mutex);

	/* Enable the GCLK */
	GCLK->CLKCTRL.reg = cfg->gclk_clkctrl_id | GCLK_CLKCTRL_GEN_GCLK0 |
			    GCLK_CLKCTRL_CLKEN;

	/* Enable SERCOM clock in PM */
	PM->APBCMASK.reg |= cfg->pm_apbcmask;

	/* Reset all registers to their initial state */
	i2cm->CTRLA.bit.SWRST = 1;
	wait_synchronization(i2cm);

	/* Set to default configuration */
	bitrate_cfg = _i2c_map_dt_bitrate(cfg->bitrate);
	retval = i2c_sam0_configure(dev, I2C_MODE_MASTER | bitrate_cfg);
	if (retval != 0) {
		return retval;
	}

	/* Enable SERCOM peripheral */
	i2cm->CTRLA.bit.ENABLE = 1;
	wait_synchronization(i2cm);

	/* Force bus state into the IDLE state  */
	i2cm->STATUS.bit.BUSSTATE = 1;
	wait_synchronization(i2cm);

	if (i2cm->STATUS.bit.BUSSTATE == I2C_BUS_STATE_IDLE) {
		SYS_LOG_DBG("%s was initialized successfully",
			    dev->config->name);
	}

	return 0;
}

static int i2c_sam0_read(struct device *dev, struct i2c_msg *msg, u16_t addr)
{
	SercomI2cm *const i2cm = DEV_CFG(dev)->regs;

	SYS_LOG_DBG("bus state is %s",
		    bus_state_str[i2cm->STATUS.bit.BUSSTATE]);

	/* Wait for the bus to become idle if we don't own it already */
	while ((i2cm->STATUS.bit.BUSSTATE != I2C_BUS_STATE_OWNER) &&
	       (i2cm->STATUS.bit.BUSSTATE != I2C_BUS_STATE_IDLE)) {
	}

	SYS_LOG_DBG("issuing %s START condition",
		    i2cm->STATUS.bit.BUSSTATE == I2C_BUS_STATE_OWNER ?
		    "repeated" : "");
		
	/* Issue (repeated) START condition followed by address */
	i2cm->ADDR.bit.ADDR = (addr << 1) | I2C_MSG_READ;

	/*
	 * Not being able to make sense of Atmel's description re. the
	 * circumstances under which the Master on Bus flag is set in master
	 * read mode, I must rely on Arduino developper's interpretation.
	 */
	while (!i2cm->INTFLAG.bit.SB) {
		if (i2cm->INTFLAG.bit.MB) { /* Negative ACK from slave */
			SYS_LOG_ERR("NACK rcvd from device @ 0x%x", addr);
			return -EIO;
		}
	}

	SYS_LOG_DBG("arbitration won, owning the bus");

	/*
	 * This test may be useless in the light of previous comment regarding
	 * Master on Bus flag setting in master read mode. It shouldn't hurt
	 * though.
	 */
	if (i2cm->STATUS.bit.RXNACK) { /* Negative ACK from slave */
		SYS_LOG_ERR("NACK rcvd from device @ 0x%x", addr);
		return -EIO;
	}

#if 1
	/*
	 * Accessing DATA.DATA auto-triggers I2C bus operations. The operation
	 * performed depends on the state of CTRLB.ACKACT, CTRLB.SMEN and the type
	 * of access (read/write).
	 */
	for (int offset = 0; offset < msg->len; ++offset) {
		if (offset == msg->len - 1) { /* Reading last byte */
			/*
			 * Set acknowledge action to NACK to stop slave
			 * transmission.
			 */
			i2cm->CTRLB.bit.ACKACT = I2C_MASTER_ACK_ACTION_NACK;

			while (!i2cm->INTFLAG.bit.SB) {
			}

			msg->buf[offset] = i2cm->DATA.bit.DATA;
		}
		else { /* Expecting more bytes */
			i2cm->CTRLB.bit.ACKACT = I2C_MASTER_ACK_ACTION_ACK;

			/* Read received byte */
			while (!i2cm->INTFLAG.bit.SB) {
			}

			msg->buf[offset] = i2cm->DATA.bit.DATA;

			/*
			 * Execute acknowledge action succeeded by a byte read.
			 */
			i2cm->CTRLB.bit.CMD = I2C_MASTER_ACTION_READ;

			wait_synchronization(i2cm);
		}
	}
#else
	/* Read received byte */
	while (!i2cm->INTFLAG.bit.SB) {
	}
	msg->buf[0] = i2cm->DATA.bit.DATA;

	for (int offset = 1; offset < msg->len; ++offset) {
		/* Set acknowledge action */
		if (offset == msg->len - 2) {
			/* Set acknowledge action to NACK to stop slave transmission */
			i2cm->CTRLB.bit.ACKACT = I2C_MASTER_ACK_ACTION_NACK;
		}
		else {
			i2cm->CTRLB.bit.ACKACT = I2C_MASTER_ACK_ACTION_ACK;
		}

		/* Execute acknowledge action succeeded by a byte read */
		i2cm->CTRLB.bit.CMD = I2C_MASTER_ACTION_READ;

		wait_synchronization(i2cm);

		/* Read received byte */
		while (!i2cm->INTFLAG.bit.SB) {
		}
		msg->buf[offset] = i2cm->DATA.bit.DATA;
	}
#endif

	if (msg->flags & I2C_MSG_STOP) { /* Issue STOP condition */
		i2cm->CTRLB.bit.CMD = I2C_MASTER_ACTION_STOP;
		wait_synchronization(i2cm);
		SYS_LOG_DBG("Issue STOP condition");
	}

	return 0;
}

static int i2c_sam0_write(struct device *dev, struct i2c_msg *msg, u16_t addr)
{
	SercomI2cm *const i2cm = DEV_CFG(dev)->regs;

	SYS_LOG_DBG("bus state is %s", bus_state_str[i2cm->STATUS.bit.BUSSTATE]);

	/* Wait for the bus to become idle if we don't own it already */
	while ((i2cm->STATUS.bit.BUSSTATE != I2C_BUS_STATE_OWNER) &&
	       (i2cm->STATUS.bit.BUSSTATE != I2C_BUS_STATE_IDLE)) {
	}

	/*
	 * As of release 1.12.99 I2C API definition, repeated START is relevant
	 * only to read operations.
	 */
	SYS_LOG_DBG("issuing START condition");

	/* Issue START condition followed by address */
	i2cm->ADDR.bit.ADDR = (addr << 1) | I2C_MSG_WRITE;

	while (!i2cm->INTFLAG.bit.MB) {
	}


	if (i2cm->STATUS.bit.ARBLOST) {
		SYS_LOG_ERR("arbitration lost, ending operation");
		return -EIO;
	}

	SYS_LOG_DBG("arbitration won, owning the bus");

	if (i2cm->STATUS.bit.RXNACK) { /* Negative ACK from slave */
		SYS_LOG_ERR("NACK rcvd from device @ 0x%x", addr);
		return -EIO;
	}

	for (int offset = 0; offset < msg->len; ++offset) {
		if (offset == 0) {
			SYS_LOG_DBG("starting transfer of %d bytes", msg->len);
		}

		i2cm->DATA.bit.DATA = msg->buf[offset];

		/*
		 * The Master on Bus flag is set regardless of the occurrence
		 * of a bus error or an arbitration lost condition, hence the
		 * need for the additionnal test.
		 */
		while (!i2cm->INTFLAG.bit.MB) {
			if (i2cm->STATUS.bit.BUSERR) {
				SYS_LOG_ERR("bus error occured");
				return -EIO;
			}
		}

		if (i2cm->STATUS.bit.RXNACK) {
			SYS_LOG_ERR("NACK rcvd from slave @ 0x%x for byte #%d",
				    addr, offset + 1);
			return -EIO;
		}
	}

	if (msg->flags & I2C_MSG_STOP) { /* Issue STOP condition */
		i2cm->CTRLB.bit.CMD = I2C_MASTER_ACTION_STOP;
		wait_synchronization(i2cm);
		SYS_LOG_DBG("issued STOP condition");
	}

	return 0;
}

static int i2c_sam0_transfer(struct device *dev, struct i2c_msg *msgs,
			     u8_t num_msgs, u16_t addr)
{
	struct i2c_sam0_dev_data *dev_data = DEV_DATA(dev);
	int ret = 0;
	SercomI2cm *const i2cm = DEV_CFG(dev)->regs;

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	SYS_LOG_DBG("");
	SYS_LOG_DBG("transaction-start addr=0x%x", addr);

	for (int i = 0; i < num_msgs; ++i) {
		SYS_LOG_DBG("msg len=%d %s%s%s", msgs[i].len,
			    (msgs[i].flags & I2C_MSG_READ) ? "R" : "W",
			    (msgs[i].flags & I2C_MSG_STOP) ? "S" : "-",
			    (msgs[i].flags & I2C_MSG_RESTART) ? "+" : "-");

		if (msgs[i].flags & I2C_MSG_READ) {
			ret = i2c_sam0_read(dev, msgs + i, addr);
		} else {
			ret = i2c_sam0_write(dev, msgs + i, addr);
		}

		if (ret < 0) { /* Issue STOP condition */
			i2cm->CTRLB.bit.CMD = I2C_MASTER_ACTION_STOP;
			wait_synchronization(i2cm);
			SYS_LOG_DBG("issued STOP condition");
		}
	}

	k_mutex_unlock(&dev_data->mutex);

	return ret;
}

static const struct i2c_driver_api i2c_sam0_driver_api = {
	.configure = i2c_sam0_configure,
	.transfer = i2c_sam0_transfer,
};

#define I2C_SAM0_CONFIG_DEFN(n)							\
	static const struct i2c_sam0_dev_cfg i2c_sam0_config_##n = {		\
		.regs = (SercomI2cm *)CONFIG_I2C_SAM0_SERCOM##n##_BASE_ADDRESS,	\
		.bitrate = CONFIG_I2C_SAM0_SERCOM##n##_CLK_FREQ,		\
		.rise_time_ns = CONFIG_I2C_SAM0_SERCOM##n##_RISE_TIME_NS,	\
		.pm_apbcmask = PM_APBCMASK_SERCOM##n,				\
		.gclk_clkctrl_id = GCLK_CLKCTRL_ID_SERCOM##n##_CORE,		\
	}

#define I2C_SAM0_DEVICE_INIT(n)						     \
	static struct i2c_sam0_dev_data i2c_sam0_dev_data_##n;		     \
	I2C_SAM0_CONFIG_DEFN(n);					     \
	DEVICE_AND_API_INIT(i2c_sam0_##n, CONFIG_I2C_SAM0_SERCOM##n##_LABEL, \
			    &i2c_sam0_init, &i2c_sam0_dev_data_##n,	     \
			    &i2c_sam0_config_##n, POST_KERNEL,		     \
			    CONFIG_I2C_INIT_PRIORITY, &i2c_sam0_driver_api); \

#if CONFIG_I2C_SAM0_SERCOM0_BASE_ADDRESS
I2C_SAM0_DEVICE_INIT(0);
#endif

#if CONFIG_I2C_SAM0_SERCOM1_BASE_ADDRESS
I2C_SAM0_DEVICE_INIT(1);
#endif

#if CONFIG_I2C_SAM0_SERCOM2_BASE_ADDRESS
I2C_SAM0_DEVICE_INIT(2);
#endif

#if CONFIG_I2C_SAM0_SERCOM3_BASE_ADDRESS
I2C_SAM0_DEVICE_INIT(3);
#endif

#if CONFIG_I2C_SAM0_SERCOM4_BASE_ADDRESS
I2C_SAM0_DEVICE_INIT(4);
#endif

#if CONFIG_I2C_SAM0_SERCOM5_BASE_ADDRESS
I2C_SAM0_DEVICE_INIT(5);
#endif
