
/*!
 * @section LICENSE
 * (C) Copyright 2011~2014 Bosch Sensortec GmbH All Rights Reserved
 *
 * This software program is licensed subject to the GNU General
 * Public License (GPL).Version 2,June 1991,
 * available at http://www.fsf.org/copyleft/gpl.html
 *
 * @filename bgsh_spi.c
 * @date     "Wed May 7 17:28:29 2014 +0800"
 * @id       "f01b4dc"
 * @version  1.0.0
 *
 * @brief
 * The spi drivercode for Bosch Generic sensor hub
 *
 * @detail
 * This file implements the spi driver code used by Bosch Generic sensor hub
*/

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include "bgsh_core.h"

/*! @defgroup bgsh_spi_src
 *  @brief bgsh spi driver module
 @{*/
/*! the maximum of transfer buffer size */
#define BGSH_MAX_BUFFER_SIZE      32

static struct spi_device *bgsh_spi_client;

/*!
 * @brief define spi wirte function
 *
 * @param dev_addr sensor device address
 * @param reg_addr register address
 * @param data the pointer of data buffer
 * @param len block size need to write
 *
 * @return zero success, non-zero failed
 * @retval zero success
 * @retval non-zero failed
*/
static int8_t bgsh_spi_write_block(uint8_t dev_addr,
	uint8_t reg_addr, uint8_t *data, uint8_t len)
{
	struct spi_device *client = bgsh_spi_client;
	uint8_t buffer[BGSH_MAX_BUFFER_SIZE + 1];
	struct spi_transfer xfer = {
		.tx_buf     = buffer,
		.len        = len + 1,
	};
	struct spi_message msg;

	if (len > BGSH_MAX_BUFFER_SIZE)
		return -EINVAL;

	buffer[0] = reg_addr&0x7F;/* write: MSB = 0 */
	memcpy(&buffer[1], data, len);

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);
	return spi_sync(client, &msg);
}

/*!
 * @brief define spi read function
 *
 * @param dev_addr sensor device address
 * @param reg_addr register address
 * @param data the pointer of data buffer
 * @param len block size need to read
 *
 * @return zero success, non-zero failed
 * @retval zero success
 * @retval non-zero failed
*/
static int8_t bgsh_spi_read_block(uint8_t dev_addr,
	uint8_t reg_addr, uint8_t *data, uint8_t len)
{
	struct spi_device *client = bgsh_spi_client;
	uint8_t reg = reg_addr | 0x80;/* read: MSB = 1 */
	struct spi_transfer xfer[2] = {
		[0] = {
			.tx_buf = &reg,
			.len = 1,
		},
		[1] = {
			.rx_buf = data,
			.len = len,
		}
	};
	struct spi_message msg;

	spi_message_init(&msg);
	spi_message_add_tail(&xfer[0], &msg);
	spi_message_add_tail(&xfer[1], &msg);
	return spi_sync(client, &msg);
}

/*!
 * @brief spi bus operation
*/
static const struct bgsh_bus_ops bgsh_spi_bus_ops = {
	/**< spi block write pointer */
	.bus_write  = bgsh_spi_write_block,
	/**< spi block read pointer */
	.bus_read   = bgsh_spi_read_block
};

/*!
 * @brief BGSH probe function via spi bus
 *
 * @param client the pointer of spi client
 *
 * @return zero success, non-zero failed
 * @retval zero success
 * @retval non-zero failed
*/
static int32_t bgsh_spi_probe(struct spi_device *client)
{
	int32_t status;
	struct bgsh_data_bus data_bus = {
		.bops = &bgsh_spi_bus_ops,
		.client = client,
		.irq = client->irq,
	};

	if (NULL == bgsh_spi_client)
		bgsh_spi_client = client;
	else{
		PERR("This driver does not support multiple clients!");
		return -EINVAL;
	}

	client->bits_per_word = 8;
	status = spi_setup(client);
	if (status < 0) {
		PERR("spi_setup failed!");
		return status;
	}

	return bgsh_probe(&client->dev, &data_bus);
}

/*!
 * @brief shutdown bgsh device in spi driver
 *
 * @param client the pointer of spi client
 *
 * @return no return value
*/
static void bgsh_spi_shutdown(struct spi_device *client)
{
#ifdef CONFIG_PM
	bgsh_disable(&client->dev);
#endif
}

/*!
 * @brief remove bgsh spi client
 *
 * @param client the pointer of spi client
 *
 * @return zero
 * @retval zero
*/
static int32_t bgsh_spi_remove(struct spi_device *client)
{
	return bgsh_remove(&client->dev);
}

#ifdef CONFIG_PM
/*!
 * @brief suspend bgsh device in spi driver
 *
 * @param dev the pointer of device
 *
 * @return zero
 * @retval zero
*/
static int32_t bgsh_spi_suspend(struct device *dev)
{
	return bgsh_suspend(dev);
}

/*!
 * @brief resume bgsh device in spi driver
 *
 * @param dev the pointer of device
 *
 * @return zero
 * @retval zero
*/
static int32_t bgsh_spi_resume(struct device *dev)
{
	return bgsh_resume(dev);
}

/*!
 * @brief register spi device power manager hooks
*/
static const struct dev_pm_ops bgsh_spi_pm_ops = {
	/**< device suspend */
	.suspend = bgsh_spi_suspend,
	/**< device resume */
	.resume  = bgsh_spi_resume
};
#endif

/*!
 * @brief register spi device id
*/
static const struct spi_device_id bgsh_id[] = {
	{ SENSOR_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, bgsh_id);

/*!
 * @brief register spi driver hooks
*/
static struct spi_driver bgsh_spi_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = SENSOR_NAME,
#ifdef CONFIG_PM
		.pm = &bgsh_spi_pm_ops,
#endif
	},
	.id_table = bgsh_id,
	.probe    = bgsh_spi_probe,
	.shutdown = bgsh_spi_shutdown,
	.remove   = bgsh_spi_remove
};

/*!
 * @brief initialize bgsh spi module
 *
 * @return zero success, non-zero failed
 * @retval zero success
 * @retval non-zero failed
*/
static int32_t __init bgsh_spi_init(void)
{
	return spi_register_driver(&bgsh_spi_driver);
}

/*!
 * @brief remove bgsh spi module
 *
 * @return no return value
*/
static void __exit bgsh_spi_exit(void)
{
	spi_unregister_driver(&bgsh_spi_driver);
}


MODULE_AUTHOR("Contact <contact@bosch-sensortec.com>");
MODULE_DESCRIPTION("BGSH SPI DRIVER");
MODULE_LICENSE("GPL v2");

module_init(bgsh_spi_init);
module_exit(bgsh_spi_exit);
/*@}*/

