
/*!
 * @section LICENSE
 * (C) Copyright 2011~2014 Bosch Sensortec GmbH All Rights Reserved
 *
 * This software program is licensed subject to the GNU General
 * Public License (GPL).Version 2,June 1991,
 * available at http://www.fsf.org/copyleft/gpl.html
 *
 * @filename bgsh_core.h
 * @date     "Sat Oct 11 16:12:16 2014 +0800"
 * @id       "762cc9e"
 * @version  1.0.0
 *
 * @brief
 * The core code for Bosch Generic sensor hub
 *
 * @detail
 * This file implements the core code of driver used by Bosch Generic sensor hub
*/

#ifndef _BGSH_CORE_H
#define _BGSH_CORE_H

#ifndef MODULE_TAG
#define MODULE_TAG "BGSH"
#endif

/*! define BGSH device name */
#define SENSOR_NAME "bgsh"

/* BGSH need power on even when system goes to suspend */
#undef CONFIG_HAS_EARLYSUSPEND
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#define LOG_LEVEL LOG_LEVEL_I
#include "bs_log.h"

/***** sensor debug funcion *****/
#define BGSH_DEBUG
#define BGSH_API_CHECK
#define BGSH_VALUE_CHECK

/****** sensor funcion ******/
#define BGSH_SIC_MATRIX_WRITE_IN_ONE_COMMAND 0
#define QUAT_SENSOR_REVERTION
#define FW_DOWNLOAD_SUPPORT 1

#if FW_DOWNLOAD_SUPPORT == 1
#define BGSH_FW_FILE_NAME "/data/misc/sensor/bgsh_fw.hex"
#define BGSH_BL_FILE_NAME "/data/misc/sensor/bgsh_bl.hex"
#define BGSH_FW_SAVED_NAME "/data/misc/sensor/bgsh_bin_saved.bin"
/* Hex file max size: 0.9M(to bin, 256KB). normally should under 500k */
#define BGSH_FW_FILE_SIZE (900*1024)
#define BGSH_FW_BIN_SIZE (256*1024)
#define BGSH_FWDL_RETRY_TIMES (3)
#define BGSH_FWDL_SECTOR_RESERVED 0x100
#define BGSH_FWDL_PACKAGE_LENGTH 64
#define BGSH_FWDL_BUFFER_LENGTH (BGSH_FWDL_PACKAGE_LENGTH+4)

/* for CRC calculation */
#define POLY        0x1021
#define REVPOLY        0x0811

#define BOOTLOADER_ADDRESS_SHIFT 0x4000
#define MAGIC_WORD_ADDRESS (0x3fffc)
#define ACK_PKG_LENGTH 2

enum {
	FILE_TYPE_ALL = 0,
	FILE_TYPE_BOOTLOADER,
	FILE_TYPE_APPCODE,
	FILE_TYPE_END
};

#define COMMAND_START_BYTE 0xaa
#define COMMAND_END_BYTE 0x0a
enum {
	RESET_BNO = 0x01,
	SWITCH_APPLICATION_MODE = 0x02,
	SWITCH_BOOTLOADER_MODE = 0x03,
	UPGRADE_BOOTLOADER = 0x04,
};

#define FW_DOWNLOAD_BIN_SAVE 0
#endif

/* include the register map */
#include "bgsh_reg.h"

/*! define BGSH register name according to API */
#define BGSH_REG_NAME(name) BGSH_##name##__REG
/*! define BGSH value name according to API */
#define BGSH_VAL_NAME(name) BGSH_##name

#define BGSH_BUS_WRITE_DELAY_TIME 5

/* sensor specific */
#define SENSOR_CHIP_ID_BGSH (0xA0)

#define BGSH_DEFAULT_ODR FASTEST_MODE_1
#define BGSH_DEFAULT_AXIS 0

#define BGSH_MAX_RETRY_WAKEUP (5)

#define BGSH_DELAY_MIN (1)
#define BGSH_DELAY_DEFAULT (200)

#define MAG_VALUE_MAX (32767)
#define MAG_VALUE_MIN (-32768)

#define BYTES_PER_LINE (16)

/* only 8 supported now, but there should be x/y swap to z. todo later */
#define BGSH_REMAPPING_MAX 8
struct axis_remap {
	uint8_t remap_value;
	uint8_t sx;
	uint8_t sy;
	uint8_t sz;
};

/*!
 * @brief bus communication operation
*/
struct bgsh_bus_ops {
	/*!write pointer */
	BGSH_WR_FUNC_PTR;
	/*!read pointer */
	BGSH_RD_FUNC_PTR;
#if FW_DOWNLOAD_SUPPORT == 1
	BGSH_WR_NOREG_FUNC_PTR;
	BGSH_RD_NOREG_FUNC_PTR;
#endif
};

/*!
 * @brief bus data client
*/
struct bgsh_data_bus {
	/*!bus communication operation */
	const struct bgsh_bus_ops *bops;
	/*!bgsh client */
	void *client;
	int irq;
	int place;
};

struct bosch_sensor_specific {
	char *name;
	/* 0 to 7 */
	uint32_t place;
	int32_t irq;
	int32_t (*irq_gpio_cfg)(void);
};

struct bgsh_client_data {
	struct bgsh_data_bus data_bus;
	struct bgsh_t device;
	struct device *dev;
	struct input_dev *input;
	struct delayed_work work;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend_handler;
#endif

	atomic_t delay;
	/* whether the system in suspend state */
	atomic_t in_suspend;

	uint8_t enable;
	uint8_t op_mode;
	uint8_t pwr_mode;
	uint8_t odr;
	uint8_t place;

	uint8_t acc_pw_mode;
	uint8_t gyro_pw_mode;
	uint8_t mag_pw_mode;
	atomic_t channels;

	struct mutex mutex_bus_op;
	/* controls not only reg, but also workqueue */
	struct mutex mutex_enable;
	struct bosch_sensor_specific *bst_pd;
	int irq;
};

struct reg_node {
	uint8_t pageid;
	uint8_t addr;
	uint8_t pos;
	uint8_t msk;
	uint8_t len;
};

#define MAKE_REG_NODE(node, item) \
{\
	node.pageid = TO_PAGEID(item);\
	node.addr = TO_REG(item);\
	node.pos = TO_POS(item);\
	node.msk = TO_MSK(item);\
	node.len = TO_LEN(item);\
}

int32_t bgsh_probe(struct device *dev, struct bgsh_data_bus *data_bus);
int32_t bgsh_remove(struct device *dev);
int32_t bgsh_suspend(struct device *dev);
int32_t bgsh_resume(struct device *dev);

#ifdef CONFIG_PM
int32_t bgsh_disable(struct device *dev);
#endif

#endif/*_BGSH_CORE_H*/
/*@}*/

