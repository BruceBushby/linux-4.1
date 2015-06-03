/*!
 * @section LICENSE
 * (C) Copyright 2011~2014 Bosch Sensortec GmbH All Rights Reserved
 *
 * This software program is licensed subject to the GNU General
 * Public License (GPL).Version 2,June 1991,
 * available at http://www.fsf.org/copyleft/gpl.html
 *
 * @filename bgsh_i2c.c
 * @date     "Tue Aug 19 16:21:59 2014 +0800"
 * @id       "a9250a8"
 * @version  1.0.0
 *
 * @brief
 * The i2c driver code for Bosch Generic sensor hub
 *
 * @detail
 * This file implements the i2c driver code used by Bosch Generic sensor hub
*/


#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include "bgsh_core.h"

#define BGSH_USE_BASIC_I2C_FUNC

static struct i2c_client *bgsh_i2c_client;

/*! @defgroup bgsh_i2c_src
 *  @brief bgsh i2c driver module
 @{*/
/*! maximum retry times during i2c transfer */
#define BGSH_MAX_RETRY_I2C_XFER 10
/*! wait time after i2c transfer error occurred */
#define BGSH_I2C_WRITE_DELAY_TIME 1

#ifdef BGSH_USE_BASIC_I2C_FUNC
/*!
 * @brief define i2c wirte function
 *
 * @param client the pointer of i2c client
 * @param reg_addr register address
 * @param data the pointer of data buffer
 * @param len block size need to write
 *
 * @return zero success, non-zero failed
 * @retval zero success
 * @retval non-zero failed
*/
static int8_t bgsh_i2c_read(struct i2c_client *client, uint8_t reg_addr,
		uint8_t *data, uint8_t len)
{
	int32_t retry;

	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = 1,
		 .buf = &reg_addr,
		},

		{
		 .addr = client->addr,
		 .flags = I2C_M_RD,
		 .len = len,
		 .buf = data,
		 },
	};

	for (retry = 0; retry < BGSH_MAX_RETRY_I2C_XFER; retry++) {
		if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) > 0)
			break;
		else
			mdelay(BGSH_I2C_WRITE_DELAY_TIME);
	}

	if (BGSH_MAX_RETRY_I2C_XFER <= retry) {
		PERR("I2C xfer error");
		return -EIO;
	}

	return 0;
}

/*!
 * @brief define i2c wirte function
 *
 * @param client the pointer of i2c client
 * @param reg_addr register address
 * @param data the pointer of data buffer
 * @param len block size need to write
 *
 * @return zero success, non-zero failed
 * @retval zero success
 * @retval non-zero failed
*/
static int8_t bgsh_i2c_write(struct i2c_client *client, uint8_t reg_addr,
		uint8_t *data, uint8_t len)
{
	uint8_t buffer[2];
	int32_t retry;
	struct i2c_msg msg = {
		 .addr = client->addr,
		 .flags = 0,
		 .len = 2,
		 .buf = buffer,
	};

	while (0 != len--) {
		msg.buf = buffer;
		msg.buf[0] = reg_addr;
		msg.buf[1] = *data;
		for (retry = 0; retry < BGSH_MAX_RETRY_I2C_XFER; retry++) {
			if (i2c_transfer(client->adapter, &msg, 1) > 0)
				break;
			mdelay(BGSH_I2C_WRITE_DELAY_TIME);
		}
		if (BGSH_MAX_RETRY_I2C_XFER <= retry) {
			PERR("I2C xfer error");
			return -EIO;
		}
		reg_addr++;
		data++;
	}

	return 0;
}
#endif/*BGSH_USE_BASIC_I2C_FUNC*/

/*!
 * @brief define i2c block wirte function
 *
 * @param dev_addr sensor i2c address
 * @param reg_addr register address
 * @param data the pointer of data buffer
 * @param len block size need to write
 *
 * @return zero success, non-zero failed
 * @retval zero success
 * @retval non-zero failed
*/
static int8_t bgsh_i2c_write_block
	(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len)
{
	int8_t err = 0;

	if (NULL == bgsh_i2c_client)
		return -1;

#ifdef BGSH_USE_BASIC_I2C_FUNC
	err = bgsh_i2c_write(bgsh_i2c_client, reg_addr, data, len);
#else
	err = i2c_smbus_write_i2c_block_data(bgsh_i2c_client, \
			reg_addr, len, data);
#endif
	if (err < 0)
		return err;

	return 0;
}

/*!
 * @brief define i2c block read function
 *
 * @param dev_addr sensor i2c address
 * @param reg_addr register address
 * @param data the pointer of data buffer
 * @param len block size need to read
 *
 * @return zero success, non-zero failed
 * @retval zero success
 * @retval non-zero failed
*/
static int8_t bgsh_i2c_read_block
	(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len)
{
	int8_t err = 0;

	if (NULL == bgsh_i2c_client)
		return -1;

#ifdef BGSH_USE_BASIC_I2C_FUNC
	err = bgsh_i2c_read(bgsh_i2c_client, reg_addr, data, len);
#else
	err = i2c_smbus_read_i2c_block_data(bgsh_i2c_client, \
			reg_addr, len, data);
#endif
	if (err < 0)
		return err;

	return 0;
}

#if FW_DOWNLOAD_SUPPORT == 1
/*!
 * @brief define i2c read function, without reg addr
	 for FW download function using
 *
 * @param data the pointer of data buffer
 * @param len block size need to write
 *
 * @return zero success, non-zero failed
 * @retval zero success
 * @retval non-zero failed
*/
static int8_t bgsh_i2c_read_noreg
	(uint8_t dev_addr, uint8_t *data, uint8_t len)

{
	int32_t retry;

	struct i2c_msg msg[] = {
		{
		 .addr = bgsh_i2c_client->addr,
		 .flags = I2C_M_RD,
		 .len = len,
		 .buf = data,
		 },
	};

	for (retry = 0; retry < BGSH_MAX_RETRY_I2C_XFER; retry++) {
		if (i2c_transfer(bgsh_i2c_client->adapter,
				msg, ARRAY_SIZE(msg)) > 0)
			break;
		else
			mdelay(BGSH_I2C_WRITE_DELAY_TIME);
		PDEBUG("I2C read rty %d", retry);
	}

	if (BGSH_MAX_RETRY_I2C_XFER <= retry) {
		PERR("I2C xfer error");
		return -EIO;
	}

	return 0;
}

/*!
 * @brief define i2c write function, without reg addr
 * for FW download function using
 *
 * @param client the pointer of i2c client
 * @param data the pointer of data buffer
 * @param len block size need to write
 *
 * @return zero success, non-zero failed
 * @retval zero success
 * @retval non-zero failed
*/
static int8_t bgsh_i2c_write_noreg
	(uint8_t dev_addr, uint8_t *data, uint8_t len)
{
	int32_t retry;
	struct i2c_msg msg = {
		.addr = bgsh_i2c_client->addr,
		.flags = 0,
		.len = len,
		.buf = NULL,
	};

	msg.buf = kmalloc(len, GFP_KERNEL);
	if (!msg.buf) {
		PERR("Allocate mem failed\n");
		return -ENOMEM;
	}
	memcpy(&msg.buf[0], data, len);

	for (retry = 0; retry < BGSH_MAX_RETRY_I2C_XFER; retry++) {
		if (i2c_transfer(bgsh_i2c_client->adapter, &msg, 1) > 0)
			break;
		else
			mdelay(BGSH_I2C_WRITE_DELAY_TIME);
		PDEBUG("I2C write rty %d", retry);
	}

	kfree(msg.buf);

	if (BGSH_MAX_RETRY_I2C_XFER <= retry) {
		PERR("I2C xfer error");
		return -EIO;
	}

	return 0;
}
#endif


/*!
 * @brief i2c bus operation
*/
static const struct bgsh_bus_ops bgsh_i2c_bus_ops = {
	/**< i2c block write pointer */
	.bus_write  = bgsh_i2c_write_block,
	/**< i2c block read pointer */
	.bus_read   = bgsh_i2c_read_block,
#if FW_DOWNLOAD_SUPPORT == 1
	.bus_read_noreg   = bgsh_i2c_read_noreg,
	.bus_write_noreg   = bgsh_i2c_write_noreg,
#endif
};

/*!
 * @brief BGSH probe function via i2c bus
 *
 * @param client the pointer of i2c client
 * @param id the pointer of i2c device id
 *
 * @return zero success, non-zero failed
 * @retval zero success
 * @retval non-zero failed
*/
static int32_t bgsh_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct bgsh_data_bus data_bus = {
		.bops = &bgsh_i2c_bus_ops,
		.client = client,
		.irq = client->irq,
	};

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		PERR("i2c_check_functionality error!");
		return -EIO;
	}

	if (NULL == bgsh_i2c_client)
		bgsh_i2c_client = client;
	else{
		PERR("This driver does not support multiple clients!");
		return -EINVAL;
	}

	return bgsh_probe(&client->dev, &data_bus);
}

/*!
 * @brief shutdown bgsh device in i2c driver
 *
 * @param client the pointer of i2c client
 *
 * @return no return value
*/
static void bgsh_i2c_shutdown(struct i2c_client *client)
{
#ifdef CONFIG_PM
	bgsh_disable(&client->dev);
#endif
}

/*!
 * @brief remove bgsh i2c client
 *
 * @param client the pointer of i2c client
 *
 * @return zero
 * @retval zero
*/
static int32_t bgsh_i2c_remove(struct i2c_client *client)
{
	return bgsh_remove(&client->dev);
}

#ifdef CONFIG_PM
/*!
 * @brief suspend bgsh device in i2c driver
 *
 * @param dev the pointer of device
 *
 * @return zero
 * @retval zero
*/
static int32_t bgsh_i2c_suspend(struct device *dev)
{
	return bgsh_suspend(dev);
}

/*!
 * @brief resume bgsh device in i2c driver
 *
 * @param dev the pointer of device
 *
 * @return zero
 * @retval zero
*/
static int32_t bgsh_i2c_resume(struct device *dev)
{
	return bgsh_resume(dev);
}

/*!
 * @brief register i2c device power manager hooks
*/
static const struct dev_pm_ops bgsh_i2c_pm_ops = {
	/**< device suspend */
	.suspend = bgsh_i2c_suspend,
	/**< device resume */
	.resume = bgsh_i2c_resume
};
#endif

/*!
 * @brief register i2c device id
*/
static const struct i2c_device_id bgsh_id[] = {
	{ SENSOR_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, bgsh_id);

/*!
 * @brief register i2c driver hooks
*/
static struct i2c_driver bgsh_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = SENSOR_NAME,
#ifdef CONFIG_PM
		.pm    = &bgsh_i2c_pm_ops,
#endif
	},
	.id_table   = bgsh_id,
	.probe      = bgsh_i2c_probe,
	.shutdown   = bgsh_i2c_shutdown,
	.remove     = bgsh_i2c_remove
};

/*!
 * @brief initialize bgsh i2c module
 *
 * @return zero success, non-zero failed
 * @retval zero success
 * @retval non-zero failed
*/
static int32_t __init bgsh_i2c_init(void)
{
	return i2c_add_driver(&bgsh_i2c_driver);
}

/*!
 * @brief remove bgsh i2c module
 *
 * @return no return value
*/
static void __exit bgsh_i2c_exit(void)
{
	i2c_del_driver(&bgsh_i2c_driver);
}


MODULE_AUTHOR("Contact <contact@bosch-sensortec.com>");
MODULE_DESCRIPTION("BGSH I2C DRIVER");
MODULE_LICENSE("GPL v2");

module_init(bgsh_i2c_init);
module_exit(bgsh_i2c_exit);
/*@}*/


