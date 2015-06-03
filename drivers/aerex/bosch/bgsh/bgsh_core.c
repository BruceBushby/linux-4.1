
/*!
 * @section LICENSE
 * (C) Copyright 2011~2014 Bosch Sensortec GmbH All Rights Reserved
 *
 * This software program is licensed subject to the GNU General
 * Public License (GPL).Version 2,June 1991,
 * available at http://www.fsf.org/copyleft/gpl.html
 *
 * @filename bgsh_core.c
 * @date     "Sat Oct 11 16:12:16 2014 +0800"
 * @id       "762cc9e"
 * @version  1.0.1
 *
 * @brief
 * The core code of BGSH device driver
 *
 * @detail
 * This file implements the core code of BGSH device driver,
 * which includes hardware related functions, input device register,
 * device attribute files, etc.
*/

#ifdef __KERNEL__
#include <linux/kernel.h>
#include <linux/unistd.h>
#include <linux/types.h>
#include <linux/string.h>
#else
#include <unistd.h>
#include <sys/types.h>
#include <string.h>
#endif

#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include "bgsh_core.h"

#if FW_DOWNLOAD_SUPPORT == 1
#include <linux/fs.h>
#include <linux/uaccess.h>
#endif

/* only 8 supported now, but there should be x/y swap to z. todo later */
struct axis_remap axis_remap_tab[BGSH_REMAPPING_MAX] = { /* BGSH */
	/* value sx sy sz */
	{REMAP_X_Y,    1, 0, 0}, /* P0 */
	{DEFAULT_AXIS, 0, 0, 0}, /* P1 */
	{DEFAULT_AXIS, 1, 1, 0}, /* P2 */
	{REMAP_X_Y,    0, 1, 0}, /* P3 */
	{DEFAULT_AXIS, 1, 0, 1}, /* P4 */
	{REMAP_X_Y,    0, 0, 1}, /* P5 */
	{REMAP_X_Y,    1, 1, 1}, /* P6 */
	{DEFAULT_AXIS, 0, 1, 1}, /* P7 */
};

BGSH_RETURN_FUNCTION_TYPE bgsh_write_page_id(
	struct bgsh_client_data *client, uint8_t page_id)
{
	BGSH_RETURN_FUNCTION_TYPE comres = SUCCESS;
	if ((&client->device) == BGSH_Zero_U8X) {
		return E_NULL_PTR;
	} else {
		comres = client->device.BGSH_BUS_WRITE_FUNC
			(client->device.dev_addr,
			TO_REG(BGSH_PAGE_ID), &page_id, 1);
		if (comres == SUCCESS)
			client->device.page_id = page_id;
	}
	return comres;
}

BGSH_RETURN_FUNCTION_TYPE bgsh_write_register(
	struct bgsh_client_data *client, uint8_t pageid,
	uint8_t addr, uint8_t *data, uint8_t len)
{
	BGSH_RETURN_FUNCTION_TYPE comres = SUCCESS;
	if (&(client->device) == BGSH_Zero_U8X) {
		return E_NULL_PTR;
	} else {
		comres = bgsh_write_page_id(client, pageid);
		if (comres == SUCCESS)
			comres = client->device.BGSH_BUS_WRITE_FUNC
				(client->device.dev_addr, addr, data, len);
	}
	return comres;
}

BGSH_RETURN_FUNCTION_TYPE bgsh_read_register(
	struct bgsh_client_data *client, uint8_t pageid,
	uint8_t addr, uint8_t *data, uint8_t len)
{
	BGSH_RETURN_FUNCTION_TYPE comres = SUCCESS;
	if (&(client->device) == BGSH_Zero_U8X) {
		return E_NULL_PTR;
	} else {
		comres = bgsh_write_page_id(client, pageid);
		if (comres == SUCCESS)
			comres = client->device.BGSH_BUS_READ_FUNC
				(client->device.dev_addr, addr, data, len);
	}
	return comres;
}

static void bgsh_delay(uint32_t msec)
{
	mdelay(msec);
}

BGSH_RETURN_FUNCTION_TYPE bgsh_init(struct bgsh_t *bgsh)
{
	BGSH_RETURN_FUNCTION_TYPE comres = SUCCESS;
	uint8_t a_data_u8r = BGSH_Zero_U8X;
	uint8_t a_swid_u8r[2] = {0, 0};

	bgsh->dev_addr = BGSH_BUS_ADDR;

	a_data_u8r = PAGE_ZERO;
	comres += bgsh->BGSH_BUS_WRITE_FUNC(bgsh->dev_addr,
		TO_REG(BGSH_PAGE_ID), &a_data_u8r, 1);
	if (comres != SUCCESS)
		return comres;
	bgsh->page_id = a_data_u8r;

	comres += bgsh->BGSH_BUS_READ_FUNC
		(bgsh->dev_addr,
		TO_REG(BGSH_CHIP_ID), &a_data_u8r, 1);
	bgsh->chip_id = a_data_u8r;

	comres += bgsh->BGSH_BUS_READ_FUNC
		(bgsh->dev_addr,
		TO_REG(BGSH_ACC_REV_ID), &a_data_u8r, 1);
	bgsh->accel_revision_id = a_data_u8r;

	comres += bgsh->BGSH_BUS_READ_FUNC
		(bgsh->dev_addr,
		TO_REG(BGSH_MAG_REV_ID), &a_data_u8r, 1);
	bgsh->mag_revision_id = a_data_u8r;

	comres += bgsh->BGSH_BUS_READ_FUNC
		(bgsh->dev_addr,
		TO_REG(BGSH_GYR_REV_ID), &a_data_u8r, 1);
	bgsh->gyro_revision_id = a_data_u8r;

	comres += bgsh->BGSH_BUS_READ_FUNC
		(bgsh->dev_addr,
		TO_REG(BGSH_BL_REV_ID), &a_data_u8r, 1);
	bgsh->bootloder_revision_id = a_data_u8r;

	comres += bgsh->BGSH_BUS_READ_FUNC(bgsh->dev_addr,
		TO_REG(BGSH_SW_REV_ID_LSB), a_swid_u8r, 2);
		a_swid_u8r[0] = BGSH_GET_BITSLICE(a_swid_u8r[0],
		BGSH_SW_REV_ID_LSB);
	bgsh->sw_revision_id = (BGSH_U16)
		((((BGSH_U16)((int8_t)a_swid_u8r[1])) <<
		BGSH_SFT_8_POS) | (a_swid_u8r[0]));

	return comres;
}

#ifdef BGSH_API_CHECK
void bgsh_check_api(struct reg_node node)
{
	uint8_t len = node.len;
	uint8_t msk = 0;

	if (len == 0)
		PERR("API check, len error, at 0x%x 0x%x 0x%x", node.pageid,
			node.addr, node.pos);

	while (len)
		msk |= 1<<(--len);
	if ((msk << node.pos) != node.msk)
		PERR("API check error, at 0x%x 0x%x 0x%x", node.pageid,
			node.addr, node.pos);
}
#else
#define bgsh_check_api(node)
#endif

#ifdef BGSH_VALUE_CHECK
void bgsh_check_value(struct reg_node node, uint8_t value)
{
	if ((0xFF & ~(node.msk)) & ((value)<<node.pos))
		PERR("API value error, 0x%x 0x%x 0x%x 0x%x", node.pageid,
			node.addr, node.pos, value);
}
#else
#define bgsh_check_value(node, value)
#endif

int32_t bgsh_read_node(struct bgsh_client_data *client,
	struct reg_node node, uint8_t *data_ptr, uint32_t length,
	int32_t *result_ptr)
{
	uint8_t v_data_u8r = BGSH_Zero_U8X;
	int32_t v_status = SUCCESS;

	bgsh_check_api(node);

	if (&(client->device) == BGSH_Zero_U8X)
		v_status = E_NULL_PTR;
	else {
		if (client->device.page_id != node.pageid)
			v_status = bgsh_write_page_id
				(client, node.pageid);
		if (v_status == SUCCESS) {
			if ((length) == 1) {
				v_status =
					client->device.BGSH_BUS_READ_FUNC
					(client->device.dev_addr,
					node.addr, &v_data_u8r, 1);
				*data_ptr = (v_data_u8r & node.msk) >> node.pos;
			} else
				v_status = client->device.BGSH_BUS_READ_FUNC
					(client->device.dev_addr,
					node.addr, data_ptr, length);
		}
	}

	if (v_status != SUCCESS)
		PERR("API read error, %d %d %d", node.pageid,
			node.addr, node.pos);
	*result_ptr += v_status;

	return v_status;
}

int32_t bgsh_write_node(struct bgsh_client_data *client,
	struct reg_node node, uint8_t data, int32_t *result_ptr)
{
	uint8_t v_data_u8w = BGSH_Zero_U8X;
	int32_t v_status = SUCCESS;

	bgsh_check_api(node);
	bgsh_check_value(node, data);

	if (&(client->device) == BGSH_Zero_U8X)
		v_status = E_NULL_PTR;
	else {
		if (client->device.page_id != node.pageid)
			v_status = bgsh_write_page_id
				(client, node.pageid);
		if (v_status == SUCCESS) {
			if (node.msk != 0xff) {
				v_status =
					client->device.BGSH_BUS_READ_FUNC
					(client->device.dev_addr,
					node.addr, &v_data_u8w, 1);
				v_data_u8w = (v_data_u8w & ~node.msk)
					| ((data<<node.pos) & node.msk);
			} else {
				v_data_u8w = data;
			}
			if (v_status == SUCCESS)
				v_status = client->device.BGSH_BUS_WRITE_FUNC
					(client->device.dev_addr,
					node.addr, &v_data_u8w, 1);
		}
	}

	if (v_status != SUCCESS)
		PERR("API write error, %d %d %d %d", node.pageid,
			node.addr, node.pos, data);
	*result_ptr += v_status;

	return v_status;
}

/* this function must be called after device initial
	if before inital, use the data_bus, as in function bgsh_wakeup() */
static int32_t bgsh_reset(struct bgsh_client_data *client_data,
	uint8_t mode)
{
	int32_t ret = SUCCESS;
	struct reg_node node;

	PDEBUG("bgsh reset to %d", mode);
	if (mode == RESET_TO_BOOTLOADER_MODE) {
		MAKE_REG_NODE(node, BGSH_SENOSR_BL_MODE);
		bgsh_write_node(client_data, node, ENABLE, &ret);
	} else if (mode == RESET_TO_NORMAL_MODE) {
		MAKE_REG_NODE(node, BGSH_RST_SYS);
		bgsh_write_node(client_data, node, ENABLE, &ret);
	}

	if (ret != SUCCESS)
		PERR("reset to mode %d fail, ret=%d", mode, ret);

	return ret;
}

static int32_t bgsh_get_op_mode
	(struct bgsh_client_data *client_data, uint8_t *op_mode)
{
	int32_t ret = SUCCESS;
	struct reg_node node;

	if (client_data == NULL)
		return E_NULL_PTR;

	MAKE_REG_NODE(node, BGSH_OPERATION_MODE);
	bgsh_read_node(client_data, node, op_mode, 1, &ret);
	return ret;
}

static int32_t bgsh_set_op_mode(
	struct bgsh_client_data *client_data, uint8_t op_mode)
{
	int32_t ret = SUCCESS;
	struct reg_node node;
	uint8_t op_mode_readback = OPERATION_MODE_CONFIG;
	uint8_t i = 0;

	if (client_data == NULL)
		return E_NULL_PTR;
	PDEBUG("%d", op_mode);

	MAKE_REG_NODE(node, BGSH_OPERATION_MODE);
	i = 0;
	while (1) {
		ret = SUCCESS;
		bgsh_write_node(client_data, node, op_mode, &ret);
		bgsh_read_node(client_data, node, &op_mode_readback, 1, &ret);

		if ((ret == SUCCESS) && (op_mode == op_mode_readback))
			break;
		if (++i > 3) {
			ret = -EIO;
			break;
		}
		PINFO("set op mode %d, rty %d times, ret: %d, readback: %d",
			op_mode, i, ret, op_mode_readback);
	}

	if ((ret == SUCCESS) && (op_mode == op_mode_readback))
		client_data->op_mode = op_mode;
	else {
		client_data->op_mode = OPERATION_MODE_CONFIG;
		PERR("set op mode %d fail, ret: %d, readback: %d",
			op_mode, ret, op_mode_readback);
	}

	return ret;
}

/* op_mode must be CONFIG, before you call it */
static int32_t bgsh_set_pwr_mode(
	struct bgsh_client_data *client_data, uint8_t pwr_mode)
{
	int32_t ret = SUCCESS;
	struct reg_node node;

	if (client_data == NULL)
		return E_NULL_PTR;
	PDEBUG("%d", pwr_mode);

	MAKE_REG_NODE(node, BGSH_POWER_MODE);
	bgsh_write_node(client_data, node,
		pwr_mode, &ret);

	if (ret == SUCCESS)
		client_data->pwr_mode = pwr_mode;
	else
		client_data->pwr_mode = SENSOR_POWER_MODE_LOWERPOWER;

	return ret;
}

static int32_t bgsh_set_sensor_remap(
	struct bgsh_client_data *client_data, int32_t place)
{
	int32_t ret = SUCCESS;
	uint8_t op_mode = OPERATION_MODE_CONFIG;
	struct reg_node node;

	if (client_data != NULL) {
		if ((place < 0) || (place >= BGSH_REMAPPING_MAX))
			return -1;

		mutex_lock(&client_data->mutex_bus_op);
		op_mode = client_data->op_mode;
		if (op_mode != OPERATION_MODE_CONFIG)
			ret = bgsh_set_op_mode(client_data,
				OPERATION_MODE_CONFIG);
		if (ret == SUCCESS) {
			MAKE_REG_NODE(node, BGSH_REMAP_AXIS_VALUE);
			bgsh_write_node(client_data, node,
				axis_remap_tab[place].remap_value, &ret);
			MAKE_REG_NODE(node, BGSH_REMAP_X_SIGN);
			bgsh_write_node(client_data, node,
				axis_remap_tab[place].sx, &ret);
			MAKE_REG_NODE(node, BGSH_REMAP_Y_SIGN);
			bgsh_write_node(client_data, node,
				axis_remap_tab[place].sy, &ret);
			MAKE_REG_NODE(node, BGSH_REMAP_Z_SIGN);
			bgsh_write_node(client_data, node,
				axis_remap_tab[place].sz, &ret);
		}
		if (op_mode != OPERATION_MODE_CONFIG)
			ret += bgsh_set_op_mode(client_data, op_mode);
		mutex_unlock(&client_data->mutex_bus_op);
	}

	return ret;
}

static void bgsh_dump_reg(struct bgsh_client_data *client_data)
{
#ifdef BGSH_DEBUG
	int32_t i;
	uint8_t dbg_buf[BGSH_REGISTER_MAP_END];
	uint8_t dbg_buf_str[BGSH_REGISTER_MAP_END * 3 + 1 + 4*8] = "";
	uint8_t pageid = 0;
	int32_t ret = SUCCESS;
	struct bgsh_data_bus *data_bus = &client_data->data_bus;

	for (i = 0; i < BYTES_PER_LINE; i++) {
		dbg_buf[i] = i;
		if (i % BYTES_PER_LINE == 0)
			sprintf(dbg_buf_str + i * 3 + (i/16) * 4,
				"%c%c%c%c",
				'I', 'd', 'x', ' ');
		sprintf(dbg_buf_str + i * 3 + (i/16 + 1) * 4,
				"%02x%c", dbg_buf[i],
				(((i + 1) % BYTES_PER_LINE == 0) ? '\n' : ' '));
	}
	printk(KERN_DEBUG "\nRegister Dump:\n");
	printk(KERN_DEBUG "%s", dbg_buf_str);

	pageid = 0;
	while (1) {
		ret = data_bus->bops->bus_write(BGSH_BUS_ADDR,
			TO_REG(BGSH_PAGE_ID), &pageid, 1);
		if (ret != SUCCESS) {
			printk(KERN_DEBUG "Set page id to %d error!\n", pageid);
			return;
		}
		client_data->device.page_id = pageid;

		printk(KERN_DEBUG "Registers in page %d:\n", pageid);
		ret = data_bus->bops->bus_read(BGSH_BUS_ADDR, 0,
			dbg_buf, BGSH_REGISTER_MAP_END);

		for (i = 0; i < BGSH_REGISTER_MAP_END; i++) {
			if (i % BYTES_PER_LINE == 0)
				sprintf(dbg_buf_str + i * 3 + (i/16) * 4,
					"%c%c%c%c",
					'0' + i/16, 'x', ':', ' ');

			sprintf(dbg_buf_str + i * 3 + (i/16 + 1) * 4,
					"%02x%c", dbg_buf[i],
					(((i + 1) % BYTES_PER_LINE == 0)
						? '\n' : ' ')
					);
		}
		if (ret != SUCCESS)
			printk(KERN_DEBUG "read reg error\n");
		else
			printk(KERN_DEBUG "%s", dbg_buf_str);

		pageid++;
		if (pageid > BGSH_REGISTER_PAGEID_MAX)
			break;
	}
#endif
}

static void bgsh_work_func(struct work_struct *work)
{
	struct bgsh_client_data *client_data =
		container_of((struct delayed_work *)work,
			struct bgsh_client_data, work);
	unsigned long delay =
		msecs_to_jiffies(atomic_read(&client_data->delay));

	int32_t ret = SUCCESS;
	uint8_t op_mode = 0;
	uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	struct reg_node node;

	/* TODO */
	mutex_lock(&client_data->mutex_bus_op);
	ret = bgsh_get_op_mode(client_data, &op_mode);

	switch (op_mode) {
	case OPERATION_MODE_CONFIG:
		break;

	case OPERATION_MODE_ACCONLY:
		MAKE_REG_NODE(node, BGSH_ACC_DATA_X_LSB_VALUEX);
		bgsh_read_node(client_data, node,
			data, 6, &ret);
		break;

	case OPERATION_MODE_MAGONLY:
		MAKE_REG_NODE(node, BGSH_MAG_DATA_X_LSB_VALUEX);
		bgsh_read_node(client_data, node,
			data, 6, &ret);
		break;

	case OPERATION_MODE_GYRONLY:
		MAKE_REG_NODE(node, BGSH_GYR_DATA_X_LSB_VALUEX);
		bgsh_read_node(client_data, node,
			data, 6, &ret);
		break;

	/*TODO*/
	default:
		break;
	}
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret == SUCCESS) {
		/*
		input_sync(client_data->input);
		*/
	}

	schedule_delayed_work(&client_data->work, delay);
}

static ssize_t bgsh_show_op_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int32_t ret = SUCCESS;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	uint8_t op_mode = 0xff;

	mutex_lock(&client_data->mutex_bus_op);
	ret = bgsh_get_op_mode(client_data, &op_mode);
	mutex_unlock(&client_data->mutex_bus_op);

	PDEBUG("%d", op_mode);
	if (ret == SUCCESS)
		ret = sprintf(buf, "%d\n", op_mode);

	return ret;
}

static ssize_t bgsh_store_op_mode(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int32_t ret = SUCCESS;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	unsigned long op_mode;

	ret = kstrtoul(buf, 10, &op_mode);
	if (ret)
		return ret;

	PDEBUG("cur op_mode=%d, set to %ld", client_data->op_mode, op_mode);

	if (op_mode > OPERATION_MODE_NDOF)
		return -EINVAL;

	if (op_mode != client_data->op_mode) {
		mutex_lock(&client_data->mutex_bus_op);
		if (client_data->op_mode != OPERATION_MODE_CONFIG)
			ret += bgsh_set_op_mode(client_data,
				OPERATION_MODE_CONFIG);
		if ((ret == SUCCESS) && (op_mode != OPERATION_MODE_CONFIG))
			ret += bgsh_set_op_mode(client_data,
				(uint8_t)op_mode);
		mutex_unlock(&client_data->mutex_bus_op);
	}

	if (ret)
		return ret;
	else
		return count;
}

static ssize_t bgsh_show_selftest(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	uint8_t selftest = 0;
	uint8_t acc_status = 0;
	uint8_t gyro_status = 0;
	uint8_t mag_status = 0;
	uint8_t mcu_status = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	int32_t ret = SUCCESS;
	struct reg_node node;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_SELF_TEST);
	bgsh_read_node(client_data, node, &selftest, 1, &ret);
	MAKE_REG_NODE(node, BGSH_ST_ACC);
	bgsh_read_node(client_data, node, &acc_status, 1, &ret);
	MAKE_REG_NODE(node, BGSH_ST_GYR);
	bgsh_read_node(client_data, node, &gyro_status, 1, &ret);
	MAKE_REG_NODE(node, BGSH_ST_MAG);
	bgsh_read_node(client_data, node, &mag_status, 1, &ret);
	MAKE_REG_NODE(node, BGSH_ST_MCU);
	bgsh_read_node(client_data, node, &mcu_status, 1, &ret);
	mutex_unlock(&client_data->mutex_bus_op);
	PDEBUG("%x %x %x %x %x", selftest, acc_status, gyro_status,
			mag_status, mcu_status);

	if (ret == SUCCESS)
		ret = sprintf(buf, "%d %d %d %d %d\n",
			selftest, acc_status, gyro_status,
			mag_status, mcu_status);
	else
		ret = -EIO;

	return ret;
}

static ssize_t bgsh_store_selftest(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long tmp;
	uint8_t selftest;
	uint8_t op_mode = OPERATION_MODE_CONFIG;
	int32_t ret = SUCCESS;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	struct reg_node node;

	ret = kstrtoul(buf, 10, &tmp);
	if (ret)
		return ret;
	PDEBUG("%ld", tmp);

	if (tmp > 1)
		return -EINVAL;

	selftest = (uint8_t)tmp;

	mutex_lock(&client_data->mutex_bus_op);
	op_mode = client_data->op_mode;
	if (op_mode != OPERATION_MODE_CONFIG)
		ret = bgsh_set_op_mode(client_data, OPERATION_MODE_CONFIG);
	if (ret == SUCCESS) {
		MAKE_REG_NODE(node, BGSH_SELF_TEST);
		bgsh_write_node(client_data, node, selftest, &ret);
	}
	if (op_mode != OPERATION_MODE_CONFIG)
		ret += bgsh_set_op_mode(client_data, op_mode);
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret)
		return ret;

	return count;
}

static ssize_t bgsh_show_pwr_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int32_t ret = SUCCESS;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	uint8_t mode = 0xff;
	struct reg_node node;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_POWER_MODE);
	bgsh_read_node(client_data, node, &mode, 1, &ret);
	mutex_unlock(&client_data->mutex_bus_op);
	PDEBUG("%d", mode);

	if (ret == SUCCESS)
		ret = sprintf(buf, "%d\n", mode);

	return ret;
}

static ssize_t bgsh_store_pwr_mode(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int32_t ret = SUCCESS;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	unsigned long mode;
	uint8_t op_mode = OPERATION_MODE_CONFIG;

	ret = kstrtoul(buf, 10, &mode);
	if (ret)
		return ret;

	PDEBUG("pwr mode=%ld", mode);

	mutex_lock(&client_data->mutex_bus_op);
	op_mode = client_data->op_mode;
	if (op_mode != OPERATION_MODE_CONFIG)
		ret = bgsh_set_op_mode(client_data, OPERATION_MODE_CONFIG);
	ret += bgsh_set_pwr_mode(client_data, (uint8_t)mode);
	if (op_mode != OPERATION_MODE_CONFIG)
		ret += bgsh_set_op_mode(client_data, op_mode);
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret)
		return ret;
	else
		return count;
}

static ssize_t bgsh_show_chip_id(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	int32_t ret = SUCCESS;
	uint8_t chip_id = 0;
	struct reg_node node;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_CHIP_ID);
	bgsh_read_node(client_data, node, &chip_id, 1, &ret);
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret == SUCCESS)
		ret = sprintf(buf, "%d\n", chip_id);

	return ret;
}

static ssize_t bgsh_show_chip_ver(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	int32_t ret = SUCCESS;
	uint8_t acc_id = 0, gyro_id = 0, mag_id = 0, bl_ver = 0;
	uint8_t sw_ver[2] = {0, 0};
	struct reg_node node;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_ACC_REV_ID);
	bgsh_read_node(client_data, node, &acc_id, 1, &ret);
	MAKE_REG_NODE(node, BGSH_GYR_REV_ID);
	bgsh_read_node(client_data, node, &gyro_id, 1, &ret);
	MAKE_REG_NODE(node, BGSH_MAG_REV_ID);
	bgsh_read_node(client_data, node, &mag_id, 1, &ret);
	MAKE_REG_NODE(node, BGSH_BL_REV_ID);
	bgsh_read_node(client_data, node, &bl_ver, 1, &ret);
	MAKE_REG_NODE(node, BGSH_SW_REV_ID_LSB);
	bgsh_read_node(client_data, node, sw_ver, 2, &ret);
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret == SUCCESS)
		ret = sprintf(buf, "%d %d %d %d %d\n",
			acc_id, gyro_id, mag_id, bl_ver,
			(BGSH_S16)((sw_ver[1]<<BGSH_SFT_8_POS) | sw_ver[0])
			);

	PDEBUG("acc:%x, gyro:%x, mag:%x, bl:%x, sw:%x",
			acc_id, gyro_id, mag_id, bl_ver,
			(BGSH_S16)((sw_ver[1]<<BGSH_SFT_8_POS) | sw_ver[0])
			);

	return ret;
}

static ssize_t bgsh_show_odr(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	uint8_t data_out_rate = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	int32_t ret = SUCCESS;
	struct reg_node node;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_OUTPUT_DATA_RATE);
	bgsh_read_node(client_data, node,
		&data_out_rate, 1, &ret);
	mutex_unlock(&client_data->mutex_bus_op);
	PDEBUG("%d", data_out_rate);

	if (ret == SUCCESS) {
		if (data_out_rate <= NORMAL_MODE)
			ret = sprintf(buf, "%d\n", data_out_rate);
		else
			ret = -EINVAL;
	}

	return ret;
}

static ssize_t bgsh_store_odr(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long tmp;
	uint8_t data_out_rate;
	int32_t ret = SUCCESS;
	uint8_t op_mode = OPERATION_MODE_CONFIG;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	struct reg_node node;

	ret = kstrtoul(buf, 10, &tmp);
	if (ret)
		return ret;
	PDEBUG("%ld", tmp);

	if (tmp > 255)
		return -EINVAL;

	data_out_rate = (uint8_t)tmp;

	mutex_lock(&client_data->mutex_bus_op);
	if (data_out_rate <= NORMAL_MODE) {
		op_mode = client_data->op_mode;
		if (op_mode != OPERATION_MODE_CONFIG)
			ret = bgsh_set_op_mode(client_data,
				OPERATION_MODE_CONFIG);
		if (ret == SUCCESS) {
			MAKE_REG_NODE(node, BGSH_OUTPUT_DATA_RATE);
			bgsh_write_node(client_data, node,
				data_out_rate, &ret);
		}
		if (ret == SUCCESS)
			client_data->odr = data_out_rate;
		if (op_mode != OPERATION_MODE_CONFIG)
			ret += bgsh_set_op_mode(client_data, op_mode);
	} else
		ret = -EINVAL;
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret)
		return ret;

	return count;
}

static ssize_t bgsh_show_channels(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);

	PDEBUG("%d", atomic_read(&client_data->channels));

	return sprintf(buf, "%d\n", atomic_read(&client_data->channels));
}

static ssize_t bgsh_store_channels(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	unsigned long channels;
	int32_t ret = SUCCESS;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	struct reg_node node;

	ret = kstrtoul(buf, 10, &data);
	if (ret)
		return ret;

	channels = atomic_read(&client_data->channels);
	if (channels == data)
		return count;
	PDEBUG("cur channels: %lx, set to %lx", channels, data);

	mutex_lock(&client_data->mutex_bus_op);
	if ((channels & (1<<SENSOR_HANDLE_ACCELERATION))
		&& !(data & (1<<SENSOR_HANDLE_ACCELERATION))) {
			MAKE_REG_NODE(node, BGSH_DATA_SEL_ACC);
			bgsh_write_node(client_data, node,
				DISABLE, &ret);
	} else if (!(channels & (1<<SENSOR_HANDLE_ACCELERATION))
		&& (data & (1<<SENSOR_HANDLE_ACCELERATION))) {
			MAKE_REG_NODE(node, BGSH_DATA_SEL_ACC);
			bgsh_write_node(client_data, node,
				ENABLE, &ret);
	}

	if ((channels & (1<<SENSOR_HANDLE_GYROSCOPE))
		&& !(data & (1<<SENSOR_HANDLE_GYROSCOPE))) {
			MAKE_REG_NODE(node, BGSH_DATA_SEL_GYR);
			bgsh_write_node(client_data, node,
				DISABLE, &ret);
	} else if (!(channels & (1<<SENSOR_HANDLE_GYROSCOPE))
		&& (data & (1<<SENSOR_HANDLE_GYROSCOPE))) {
			MAKE_REG_NODE(node, BGSH_DATA_SEL_GYR);
			bgsh_write_node(client_data, node,
				ENABLE, &ret);
	}

	if ((channels & (1<<SENSOR_HANDLE_MAGNETIC_FIELD))
		&& !(data & (1<<SENSOR_HANDLE_MAGNETIC_FIELD))) {
			MAKE_REG_NODE(node, BGSH_DATA_SEL_MAG);
			bgsh_write_node(client_data, node,
				DISABLE, &ret);
	} else if (!(channels & (1<<SENSOR_HANDLE_MAGNETIC_FIELD))
		&& (data & (1<<SENSOR_HANDLE_MAGNETIC_FIELD))) {
			MAKE_REG_NODE(node, BGSH_DATA_SEL_MAG);
			bgsh_write_node(client_data, node,
				ENABLE, &ret);
	}

	if ((channels & (1<<SENSOR_HANDLE_ORIENTATION))
		&& !(data & (1<<SENSOR_HANDLE_ORIENTATION))) {
			MAKE_REG_NODE(node, BGSH_DATA_SEL_EUL);
			bgsh_write_node(client_data, node,
				DISABLE, &ret);
	} else if (!(channels & (1<<SENSOR_HANDLE_ORIENTATION))
		&& (data & (1<<SENSOR_HANDLE_ORIENTATION))) {
			MAKE_REG_NODE(node, BGSH_DATA_SEL_EUL);
			bgsh_write_node(client_data, node,
				ENABLE, &ret);
	}

	if ((channels & (1<<SENSOR_HANDLE_GRAVITY))
		&& !(data & (1<<SENSOR_HANDLE_GRAVITY))) {
			MAKE_REG_NODE(node, BGSH_DATA_SEL_GRV);
			bgsh_write_node(client_data, node,
				DISABLE, &ret);
	} else if (!(channels & (1<<SENSOR_HANDLE_GRAVITY))
		&& (data & (1<<SENSOR_HANDLE_GRAVITY))) {
			MAKE_REG_NODE(node, BGSH_DATA_SEL_GRV);
			bgsh_write_node(client_data, node,
				ENABLE, &ret);
	}

	if ((channels & (1<<SENSOR_HANDLE_LINEAR_ACCELERATION))
		&& !(data & (1<<SENSOR_HANDLE_LINEAR_ACCELERATION))) {
			MAKE_REG_NODE(node, BGSH_DATA_SEL_LINEAR_ACC);
			bgsh_write_node(client_data, node,
				DISABLE, &ret);
	} else if (!(channels & (1<<SENSOR_HANDLE_LINEAR_ACCELERATION))
		&& (data & (1<<SENSOR_HANDLE_LINEAR_ACCELERATION))) {
			MAKE_REG_NODE(node, BGSH_DATA_SEL_LINEAR_ACC);
			bgsh_write_node(client_data, node,
				ENABLE, &ret);
	}

	if ((channels & (1<<SENSOR_HANDLE_ROTATION_VECTOR))
		&& !(data & (1<<SENSOR_HANDLE_ROTATION_VECTOR))) {
			MAKE_REG_NODE(node, BGSH_DATA_SEL_QUA);
			bgsh_write_node(client_data, node,
				DISABLE, &ret);
	} else if (!(channels & (1<<SENSOR_HANDLE_ROTATION_VECTOR))
		&& (data & (1<<SENSOR_HANDLE_ROTATION_VECTOR))) {
			MAKE_REG_NODE(node, BGSH_DATA_SEL_QUA);
			bgsh_write_node(client_data, node,
				ENABLE, &ret);
	}

	if (ret) {
		PERR("Set channels error");
		mutex_unlock(&client_data->mutex_bus_op);
		return -EIO;
	}

	atomic_set(&client_data->channels, data);
	mutex_unlock(&client_data->mutex_bus_op);

	return count;
}

static ssize_t bgsh_show_acc_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	uint8_t acc_calib = 0;
	uint8_t data[6] = {0, 0, 0, 0, 0, 0};
	int32_t ret = SUCCESS;
	struct reg_node node;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_ACC_DATA_X_LSB_VALUEX);
	bgsh_read_node(client_data, node, data, 6, &ret);
	MAKE_REG_NODE(node, BGSH_ACC_CALIB_STAT);
	bgsh_read_node(client_data, node, &acc_calib, 1, &ret);
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret != SUCCESS) {
		PERR("read sensor data fail, ret=%d", ret);
		return -EIO;
	}

	ret = sprintf(buf, "%d %d %d %d\n",
			(BGSH_S16)((data[1]<<BGSH_SFT_8_POS) | data[0]),
			(BGSH_S16)((data[3]<<BGSH_SFT_8_POS) | data[2]),
			(BGSH_S16)((data[5]<<BGSH_SFT_8_POS) | data[4]),
			acc_calib);
	PDEBUG_DLOG("%d %d %d %d",
			(BGSH_S16)((data[1]<<BGSH_SFT_8_POS) | data[0]),
			(BGSH_S16)((data[3]<<BGSH_SFT_8_POS) | data[2]),
			(BGSH_S16)((data[5]<<BGSH_SFT_8_POS) | data[4]),
			acc_calib);

	return ret;
}

static ssize_t bgsh_show_mag_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	uint8_t mag_calib = 0;
	uint8_t data[6] = {0, 0, 0, 0, 0, 0};
	int32_t ret = SUCCESS;
	struct reg_node node;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_MAG_DATA_X_LSB_VALUEX);
	bgsh_read_node(client_data, node, data, 6, &ret);
	MAKE_REG_NODE(node, BGSH_MAG_CALIB_STAT);
	bgsh_read_node(client_data, node, &mag_calib, 1, &ret);
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret != 0) {
		PERR("read sensor data fail, ret=%d", ret);
		return -EIO;
	}

	ret = sprintf(buf, "%d %d %d %d\n",
			(BGSH_S16)((data[1]<<BGSH_SFT_8_POS) | data[0]),
			(BGSH_S16)((data[3]<<BGSH_SFT_8_POS) | data[2]),
			(BGSH_S16)((data[5]<<BGSH_SFT_8_POS) | data[4]),
			mag_calib);
	PDEBUG_DLOG("%d %d %d %d",
			(BGSH_S16)((data[1]<<BGSH_SFT_8_POS) | data[0]),
			(BGSH_S16)((data[3]<<BGSH_SFT_8_POS) | data[2]),
			(BGSH_S16)((data[5]<<BGSH_SFT_8_POS) | data[4]),
			mag_calib);

	return ret;
}

static ssize_t bgsh_show_gyro_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	uint8_t gyro_calib = 0;
	uint8_t data[6] = {0, 0, 0, 0, 0, 0};
	int32_t ret = SUCCESS;
	struct reg_node node;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_GYR_DATA_X_LSB_VALUEX);
	bgsh_read_node(client_data, node, data, 6, &ret);
	MAKE_REG_NODE(node, BGSH_GYR_CALIB_STAT);
	bgsh_read_node(client_data, node, &gyro_calib, 1, &ret);
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret != 0) {
		PERR("read sensor data fail, ret=%d", ret);
		return -EIO;
	}

	ret = sprintf(buf, "%d %d %d %d\n",
			(BGSH_S16)((data[1]<<BGSH_SFT_8_POS) | data[0]),
			(BGSH_S16)((data[3]<<BGSH_SFT_8_POS) | data[2]),
			(BGSH_S16)((data[5]<<BGSH_SFT_8_POS) | data[4]),
			gyro_calib);
	PDEBUG_DLOG("%d %d %d %d",
			(BGSH_S16)((data[1]<<BGSH_SFT_8_POS) | data[0]),
			(BGSH_S16)((data[3]<<BGSH_SFT_8_POS) | data[2]),
			(BGSH_S16)((data[5]<<BGSH_SFT_8_POS) | data[4]),
			gyro_calib);

	return ret;
}

static ssize_t bgsh_show_euler_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	uint8_t data[6] = {0, 0, 0, 0, 0, 0};
	uint8_t status = 0;
	int32_t ret = SUCCESS;
	struct reg_node node;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_EUL_HEADING_LSB_VALUEH);
	bgsh_read_node(client_data, node, data, 6, &ret);
	MAKE_REG_NODE(node, BGSH_SYS_CALIB_STAT);
	bgsh_read_node(client_data, node, &status, 1, &ret);
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret != 0) {
		PERR("read sensor data fail, ret=%d", ret);
		return -EIO;
	}

	/* report in h, p, r, status */
	ret = sprintf(buf, "%d %d %d %d\n",
			(BGSH_S16)((data[1]<<BGSH_SFT_8_POS) | data[0]),
			(BGSH_S16)((data[5]<<BGSH_SFT_8_POS) | data[4]),
			(BGSH_S16)((data[3]<<BGSH_SFT_8_POS) | data[2]),
			status
			);
	PDEBUG_DLOG("%d %d %d %d",
			(BGSH_S16)((data[1]<<BGSH_SFT_8_POS) | data[0]),
			(BGSH_S16)((data[5]<<BGSH_SFT_8_POS) | data[4]),
			(BGSH_S16)((data[3]<<BGSH_SFT_8_POS) | data[2]),
			status
			);

	return ret;
}

static ssize_t bgsh_show_gravity_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	uint8_t data[6] = {0, 0, 0, 0, 0, 0};
	int32_t ret = SUCCESS;
	struct reg_node node;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_GRV_DATA_X_LSB_VALUEX);
	bgsh_read_node(client_data, node, data, 6, &ret);
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret != 0) {
		PERR("read sensor data fail, ret=%d", ret);
		return -EIO;
	}

	ret = sprintf(buf, "%d %d %d\n",
			(BGSH_S16)((data[1]<<BGSH_SFT_8_POS) | data[0]),
			(BGSH_S16)((data[3]<<BGSH_SFT_8_POS) | data[2]),
			(BGSH_S16)((data[5]<<BGSH_SFT_8_POS) | data[4])
			);
	PDEBUG_DLOG("%d %d %d",
			(BGSH_S16)((data[1]<<BGSH_SFT_8_POS) | data[0]),
			(BGSH_S16)((data[3]<<BGSH_SFT_8_POS) | data[2]),
			(BGSH_S16)((data[5]<<BGSH_SFT_8_POS) | data[4])
			);

	return ret;
}

static ssize_t bgsh_show_linear_accel_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	uint8_t data[6] = {0, 0, 0, 0, 0, 0};
	int32_t ret = SUCCESS;
	struct reg_node node;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_LIA_DATA_X_LSB_VALUEX);
	bgsh_read_node(client_data, node, data, 6, &ret);
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret != 0) {
		PERR("read sensor data fail, ret=%d", ret);
		return -EIO;
	}

	ret = sprintf(buf, "%d %d %d\n",
			(BGSH_S16)((data[1]<<BGSH_SFT_8_POS) | data[0]),
			(BGSH_S16)((data[3]<<BGSH_SFT_8_POS) | data[2]),
			(BGSH_S16)((data[5]<<BGSH_SFT_8_POS) | data[4])
			);
	PDEBUG_DLOG("%d %d %d",
			(BGSH_S16)((data[1]<<BGSH_SFT_8_POS) | data[0]),
			(BGSH_S16)((data[3]<<BGSH_SFT_8_POS) | data[2]),
			(BGSH_S16)((data[5]<<BGSH_SFT_8_POS) | data[4])
			);

	return ret;
}

static ssize_t bgsh_show_quat_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	int32_t ret = SUCCESS;
	struct reg_node node;
	struct bgsh_quaternion quat_data;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_QUA_DATA_W_LSB_VALUEW);
	bgsh_read_node(client_data, node,
		data, 8, &ret);
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret != 0) {
		PERR("read sensor data fail, ret=%d", ret);
		return -EIO;
	}

	quat_data.w = (BGSH_S16)((data[1]<<BGSH_SFT_8_POS) | data[0]);
	quat_data.x = (BGSH_S16)((data[3]<<BGSH_SFT_8_POS) | data[2]);
	quat_data.y = (BGSH_S16)((data[5]<<BGSH_SFT_8_POS) | data[4]);
	quat_data.z = (BGSH_S16)((data[7]<<BGSH_SFT_8_POS) | data[6]);
#ifdef QUAT_SENSOR_REVERTION
	if (quat_data.w < 0) {
		quat_data.w = 0 - quat_data.w;
		quat_data.x = 0 - quat_data.x;
		quat_data.y = 0 - quat_data.y;
		quat_data.z = 0 - quat_data.z;
	}
#endif

	/* report in x, y, z, w */
	ret = sprintf(buf, "%d %d %d %d\n",
			quat_data.x, quat_data.y,
			quat_data.z, quat_data.w
			);
	PDEBUG_DLOG("%d %d %d %d",
			quat_data.x, quat_data.y,
			quat_data.z, quat_data.w
			);
	return ret;
}

static ssize_t bgsh_show_temperature(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	int32_t ret = SUCCESS;
	uint8_t data[2] = {0, 0};
	struct reg_node node;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_TEMP);
	bgsh_read_node(client_data, node, data, 2, &ret);
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret == SUCCESS)
		ret = sprintf(buf, "%d\n",
			(BGSH_S16)((data[1]<<BGSH_SFT_8_POS) | data[0]));
	PDEBUG_DLOG("%d", (BGSH_S16)((data[1]<<BGSH_SFT_8_POS) | data[0]));

	return ret;
}

static ssize_t bgsh_show_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	int32_t ret = SUCCESS;

	mutex_lock(&client_data->mutex_enable);
	ret = sprintf(buf, "%d\n", client_data->enable);
	mutex_unlock(&client_data->mutex_enable);
	return ret;
}

static ssize_t bgsh_store_enable(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int32_t ret = SUCCESS;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);

	ret = kstrtoul(buf, 10, &data);
	if (ret)
		return ret;
	PDEBUG("%ld", data);

	data = data ? 1 : 0;
	mutex_lock(&client_data->mutex_enable);
	if (data != client_data->enable) {
		if (data) {
			schedule_delayed_work(
					&client_data->work,
					msecs_to_jiffies(atomic_read(
							&client_data->delay)));
		} else
			cancel_delayed_work_sync(&client_data->work);

		client_data->enable = data;
	}
	mutex_unlock(&client_data->mutex_enable);

	return count;
}

static ssize_t bgsh_show_delay(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);

	return sprintf(buf, "%d\n", atomic_read(&client_data->delay));
}

static ssize_t bgsh_store_delay(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int32_t ret = SUCCESS;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);

	ret = kstrtoul(buf, 10, &data);
	if (ret)
		return ret;

	PDEBUG("%ld", data);

	if (data < BGSH_DELAY_MIN)
		data = BGSH_DELAY_MIN;

	atomic_set(&client_data->delay, data);

	return count;
}


static ssize_t bgsh_show_place(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	int32_t ret = SUCCESS;

	PDEBUG("%d", client_data->place);
	ret = sprintf(buf, "%d\n", client_data->place);

	return ret;
}

static ssize_t bgsh_store_place(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	long data;
	int32_t ret = SUCCESS;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);

	ret = kstrtol(buf, 10, &data);
	if (ret)
		return ret;
	PDEBUG("%ld", data);

	if ((data < -1) || (data >= BGSH_REMAPPING_MAX)) {
		ret = -EINVAL;
		return ret;
	}

	/* hal will not set default place, just use driver setting */
	if (data != -1)
		client_data->place = data;

	ret = bgsh_set_sensor_remap(client_data, client_data->place);
	if (ret)
		return -EIO;

	return count;
}

static ssize_t bgsh_show_data_format_acc(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	uint8_t unit = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	int32_t ret = SUCCESS;
	struct reg_node node;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_ACC_UNIT);
	bgsh_read_node(client_data, node, &unit, 1, &ret);
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret == SUCCESS)
		ret = sprintf(buf, "%d\n", unit);
	else
		ret = -EIO;

	return ret;
}

static ssize_t bgsh_store_data_format_acc(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long tmp;
	uint8_t unit;
	uint8_t op_mode = OPERATION_MODE_CONFIG;
	int32_t ret = SUCCESS;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	struct reg_node node;

	ret = kstrtoul(buf, 10, &tmp);
	if (ret)
		return ret;
	PDEBUG("%ld", tmp);

	if (tmp > 1)
		return -EINVAL;

	unit = (uint8_t)tmp;

	mutex_lock(&client_data->mutex_bus_op);
	op_mode = client_data->op_mode;
	if (op_mode != OPERATION_MODE_CONFIG)
		ret = bgsh_set_op_mode(client_data, OPERATION_MODE_CONFIG);
	if (ret == SUCCESS) {
		MAKE_REG_NODE(node, BGSH_ACC_UNIT);
		bgsh_write_node(client_data, node, unit, &ret);
	}
	if (op_mode != OPERATION_MODE_CONFIG)
		ret += bgsh_set_op_mode(client_data, op_mode);
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret)
		return ret;

	return count;
}

static ssize_t bgsh_show_data_format_gyro(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	uint8_t unit = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	int32_t ret = SUCCESS;
	struct reg_node node;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_GYR_UNIT);
	bgsh_read_node(client_data, node, &unit, 1, &ret);
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret == SUCCESS)
		ret = sprintf(buf, "%d\n", unit);
	else
		ret = -EIO;

	return ret;
}

static ssize_t bgsh_store_data_format_gyro(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long tmp;
	uint8_t unit;
	int32_t ret = SUCCESS;
	uint8_t op_mode = OPERATION_MODE_CONFIG;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	struct reg_node node;

	ret = kstrtoul(buf, 10, &tmp);
	if (ret)
		return ret;
	PDEBUG("%ld", tmp);

	if (tmp > 1)
		return -EINVAL;

	unit = (uint8_t)tmp;

	mutex_lock(&client_data->mutex_bus_op);
	op_mode = client_data->op_mode;
	if (op_mode != OPERATION_MODE_CONFIG)
		ret = bgsh_set_op_mode(client_data, OPERATION_MODE_CONFIG);
	if (ret == SUCCESS) {
		MAKE_REG_NODE(node, BGSH_GYR_UNIT);
		bgsh_write_node(client_data, node, unit, &ret);
	}
	if (op_mode != OPERATION_MODE_CONFIG)
		ret += bgsh_set_op_mode(client_data, op_mode);
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret)
		return ret;

	return count;
}

static ssize_t bgsh_show_data_format_euler(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	uint8_t unit = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	int32_t ret = SUCCESS;
	struct reg_node node;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_EUL_UNIT);
	bgsh_read_node(client_data, node, &unit, 1, &ret);
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret == SUCCESS)
		ret = sprintf(buf, "%d\n", unit);
	else
		ret = -EIO;

	return ret;
}

static ssize_t bgsh_store_data_format_euler(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long tmp;
	uint8_t unit;
	int32_t ret = SUCCESS;
	uint8_t op_mode = OPERATION_MODE_CONFIG;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	struct reg_node node;

	ret = kstrtoul(buf, 10, &tmp);
	if (ret)
		return ret;
	PDEBUG("%ld", tmp);

	if (tmp > 1)
		return -EINVAL;

	unit = (uint8_t)tmp;

	mutex_lock(&client_data->mutex_bus_op);
	op_mode = client_data->op_mode;
	if (op_mode != OPERATION_MODE_CONFIG)
		ret = bgsh_set_op_mode(client_data, OPERATION_MODE_CONFIG);
	if (ret == SUCCESS) {
		MAKE_REG_NODE(node, BGSH_EUL_UNIT);
		bgsh_write_node(client_data, node, unit, &ret);
	}
	if (op_mode != OPERATION_MODE_CONFIG)
		ret += bgsh_set_op_mode(client_data, op_mode);
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret)
		return ret;

	return count;
}

static ssize_t bgsh_show_data_format_temperature(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	uint8_t unit = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	int32_t ret = SUCCESS;
	struct reg_node node;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_TEMP_UNIT);
	bgsh_read_node(client_data, node, &unit, 1, &ret);
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret == SUCCESS)
		ret = sprintf(buf, "%d\n", unit);
	else
		ret = -EIO;

	return ret;
}

static ssize_t bgsh_store_data_format_temperature(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long tmp;
	uint8_t unit;
	int32_t ret = SUCCESS;
	uint8_t op_mode = OPERATION_MODE_CONFIG;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	struct reg_node node;

	ret = kstrtoul(buf, 10, &tmp);
	if (ret)
		return ret;
	PDEBUG("%ld", tmp);

	if (tmp > 1)
		return -EINVAL;

	unit = (uint8_t)tmp;

	mutex_lock(&client_data->mutex_bus_op);
	op_mode = client_data->op_mode;
	if (op_mode != OPERATION_MODE_CONFIG)
		ret = bgsh_set_op_mode(client_data, OPERATION_MODE_CONFIG);
	if (ret == SUCCESS) {
		MAKE_REG_NODE(node, BGSH_TEMP_UNIT);
		bgsh_write_node(client_data, node, unit, &ret);
	}
	if (op_mode != OPERATION_MODE_CONFIG)
		ret += bgsh_set_op_mode(client_data, op_mode);
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret)
		return ret;

	return count;
}

static ssize_t bgsh_show_acc_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	uint8_t pw_mode = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	int32_t ret = SUCCESS;
	struct reg_node node;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_ACC_PWR_MODE);
	bgsh_read_node(client_data, node, &pw_mode, 1, &ret);
	mutex_unlock(&client_data->mutex_bus_op);
	PDEBUG("%d", pw_mode);

	if (ret == SUCCESS)
		ret = sprintf(buf, "%d\n", pw_mode);
	else
		ret = -EIO;

	return ret;
}

static ssize_t bgsh_store_acc_mode(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long tmp;
	uint8_t pw_mode;
	int32_t ret = SUCCESS;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	uint8_t op_mode = OPERATION_MODE_CONFIG;
	struct reg_node node;

	ret = kstrtoul(buf, 10, &tmp);
	if (ret)
		return ret;
	PDEBUG("%ld", tmp);

	pw_mode = (uint8_t)tmp;
	if (pw_mode >= BGSH_Six_U8X)
		return -EINVAL;

	if (client_data->acc_pw_mode != pw_mode) {
		mutex_lock(&client_data->mutex_bus_op);
		op_mode = client_data->op_mode;
		if (op_mode != OPERATION_MODE_CONFIG)
			ret = bgsh_set_op_mode(client_data,
				OPERATION_MODE_CONFIG);
		if (ret == SUCCESS) {
			MAKE_REG_NODE(node, BGSH_ACC_PWR_MODE);
			bgsh_write_node(client_data, node,
				pw_mode, &ret);
		}
		if (ret == SUCCESS)
			client_data->acc_pw_mode = pw_mode;
		if (op_mode != OPERATION_MODE_CONFIG)
			ret += bgsh_set_op_mode(client_data, op_mode);
		mutex_unlock(&client_data->mutex_bus_op);

		if (ret)
			return ret;
	}

	return count;
}

static ssize_t bgsh_show_acc_grange(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	uint8_t range = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	int32_t ret = SUCCESS;
	struct reg_node node;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_ACC_RANGE);
	bgsh_read_node(client_data, node, &range, 1, &ret);
	mutex_unlock(&client_data->mutex_bus_op);
	PDEBUG("%d", range);

	if (ret == SUCCESS)
		ret = sprintf(buf, "%d\n", range);
	else
		ret = -EIO;

	return ret;
}

static ssize_t bgsh_store_acc_grange(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long tmp;
	uint8_t range;
	int32_t ret = SUCCESS;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	uint8_t op_mode = OPERATION_MODE_CONFIG;
	struct reg_node node;

	ret = kstrtoul(buf, 10, &tmp);
	if (ret)
		return ret;
	PDEBUG("%ld", tmp);

	range = (uint8_t)tmp;
	if (range >= BGSH_Five_U8X)
		return -EINVAL;

	mutex_lock(&client_data->mutex_bus_op);
	op_mode = client_data->op_mode;
	if (op_mode != OPERATION_MODE_CONFIG)
		ret = bgsh_set_op_mode(client_data, OPERATION_MODE_CONFIG);
	if (ret == SUCCESS) {
		MAKE_REG_NODE(node, BGSH_ACC_RANGE);
		bgsh_write_node(client_data, node, range, &ret);
	}
	if (op_mode != OPERATION_MODE_CONFIG)
		ret += bgsh_set_op_mode(client_data, op_mode);
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret)
		return ret;

	return count;
}

static ssize_t bgsh_show_acc_bandwidth(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	uint8_t bandwidth = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	int32_t ret = SUCCESS;
	struct reg_node node;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_ACC_BW);
	bgsh_read_node(client_data, node, &bandwidth, 1, &ret);
	mutex_unlock(&client_data->mutex_bus_op);
	PDEBUG("%d", bandwidth);

	if (ret == SUCCESS)
		ret = sprintf(buf, "%d\n", bandwidth);
	else
		ret = -EIO;

	return ret;
}

static ssize_t bgsh_store_acc_bandwidth(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long tmp;
	uint8_t bandwidth;
	int32_t ret = SUCCESS;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	uint8_t op_mode = OPERATION_MODE_CONFIG;
	struct reg_node node;

	ret = kstrtoul(buf, 10, &tmp);
	if (ret)
		return ret;
	PDEBUG("%ld", tmp);

	bandwidth = (uint8_t)tmp;
	if (bandwidth >= BGSH_Eight_U8X)
		return -EINVAL;

	mutex_lock(&client_data->mutex_bus_op);
	op_mode = client_data->op_mode;
	if (op_mode != OPERATION_MODE_CONFIG)
		ret = bgsh_set_op_mode(client_data, OPERATION_MODE_CONFIG);
	if (ret == SUCCESS) {
		MAKE_REG_NODE(node, BGSH_ACC_BW);
		bgsh_write_node(client_data, node, bandwidth, &ret);
	}
	if (op_mode != OPERATION_MODE_CONFIG)
		ret += bgsh_set_op_mode(client_data, op_mode);
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret)
		return ret;

	return count;
}

static ssize_t bgsh_show_gyro_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	uint8_t pw_mode = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	int32_t ret = SUCCESS;
	struct reg_node node;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_GYR_POWER_MODE);
	bgsh_read_node(client_data, node,
		&pw_mode, 1, &ret);
	mutex_unlock(&client_data->mutex_bus_op);
	PDEBUG("%d", pw_mode);

	if (ret == SUCCESS)
		ret = sprintf(buf, "%d\n", pw_mode);
	else
		ret = -EIO;

	return ret;
}

static ssize_t bgsh_store_gyro_mode(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long tmp;
	uint8_t pw_mode;
	int32_t ret = SUCCESS;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	uint8_t op_mode = OPERATION_MODE_CONFIG;
	struct reg_node node;

	ret = kstrtoul(buf, 10, &tmp);
	if (ret)
		return ret;
	PDEBUG("%ld", tmp);

	pw_mode = (uint8_t)tmp;
	if (pw_mode >= BGSH_Five_U8X)
		return -EINVAL;

	if (client_data->gyro_pw_mode != pw_mode) {
		mutex_lock(&client_data->mutex_bus_op);
		op_mode = client_data->op_mode;
		if (op_mode != OPERATION_MODE_CONFIG)
			ret = bgsh_set_op_mode(client_data,
				OPERATION_MODE_CONFIG);
		if (ret == SUCCESS) {
			MAKE_REG_NODE(node, BGSH_GYR_POWER_MODE);
			bgsh_write_node(client_data,
				node, pw_mode, &ret);
		}
		if (ret == SUCCESS)
			client_data->gyro_pw_mode = pw_mode;
		if (op_mode != OPERATION_MODE_CONFIG)
			ret += bgsh_set_op_mode(client_data, op_mode);
		mutex_unlock(&client_data->mutex_bus_op);

		if (ret)
			return ret;
	}

	return count;
}

static ssize_t bgsh_show_gyro_range(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	uint8_t range = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	int32_t ret = SUCCESS;
	struct reg_node node;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_GYR_RANGE);
	bgsh_read_node(client_data, node, &range, 1, &ret);
	mutex_unlock(&client_data->mutex_bus_op);
	PDEBUG("%d", range);

	if (ret == SUCCESS)
		ret = sprintf(buf, "%d\n", range);
	else
		ret = -EIO;

	return ret;
}

static ssize_t bgsh_store_gyro_range(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long tmp;
	uint8_t range;
	int32_t ret = SUCCESS;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	uint8_t op_mode = OPERATION_MODE_CONFIG;
	struct reg_node node;

	ret = kstrtoul(buf, 10, &tmp);
	if (ret)
		return ret;
	PDEBUG("%ld", tmp);

	range = (uint8_t)tmp;
	if (range >= BGSH_Five_U8X)
		return -EINVAL;

	mutex_lock(&client_data->mutex_bus_op);
	op_mode = client_data->op_mode;
	if (op_mode != OPERATION_MODE_CONFIG)
		ret = bgsh_set_op_mode(client_data, OPERATION_MODE_CONFIG);
	if (ret == SUCCESS) {
		MAKE_REG_NODE(node, BGSH_GYR_RANGE);
		bgsh_write_node(client_data, node, range, &ret);
	}
	if (op_mode != OPERATION_MODE_CONFIG)
		ret += bgsh_set_op_mode(client_data, op_mode);
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret)
		return ret;

	return count;
}

static ssize_t bgsh_show_gyro_bandwidth(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	uint8_t bandwidth = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	int32_t ret = SUCCESS;
	struct reg_node node;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_GYR_BANDWIDTH);
	bgsh_read_node(client_data, node,
		&bandwidth, 1, &ret);
	mutex_unlock(&client_data->mutex_bus_op);
	PDEBUG("%d", bandwidth);

	if (ret == SUCCESS)
		ret = sprintf(buf, "%d\n", bandwidth);
	else
		ret = -EIO;

	return ret;
}

static ssize_t bgsh_store_gyro_bandwidth(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long tmp;
	uint8_t bandwidth;
	int32_t ret = SUCCESS;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	uint8_t op_mode = OPERATION_MODE_CONFIG;
	struct reg_node node;

	ret = kstrtoul(buf, 10, &tmp);
	if (ret)
		return ret;
	PDEBUG("%ld", tmp);

	bandwidth = (uint8_t)tmp;
	if (bandwidth >= BGSH_Eight_U8X)
		return -EINVAL;

	mutex_lock(&client_data->mutex_bus_op);
	op_mode = client_data->op_mode;
	if (op_mode != OPERATION_MODE_CONFIG)
		ret = bgsh_set_op_mode(client_data,
			OPERATION_MODE_CONFIG);
	if (ret == SUCCESS) {
		MAKE_REG_NODE(node, BGSH_GYR_BANDWIDTH);
		bgsh_write_node(client_data, node,
			bandwidth, &ret);
	}
	if (op_mode != OPERATION_MODE_CONFIG)
		ret += bgsh_set_op_mode(client_data, op_mode);
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret)
		return ret;

	return count;
}

static ssize_t bgsh_show_mag_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	uint8_t pw_mode = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	int32_t ret = SUCCESS;
	struct reg_node node;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_MAG_POWER_MODE);
	bgsh_read_node(client_data, node,
		&pw_mode, 1, &ret);
	mutex_unlock(&client_data->mutex_bus_op);
	PDEBUG("%d", pw_mode);

	if (ret == SUCCESS)
		ret = sprintf(buf, "%d\n", pw_mode);
	else
		ret = -EIO;

	return ret;
}

static ssize_t bgsh_store_mag_mode(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long tmp;
	uint8_t pw_mode;
	int32_t ret = SUCCESS;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	uint8_t op_mode = OPERATION_MODE_CONFIG;
	struct reg_node node;

	ret = kstrtoul(buf, 10, &tmp);
	if (ret)
		return ret;
	PDEBUG("%ld", tmp);

	pw_mode = (uint8_t)tmp;
	if (pw_mode >= BGSH_Four_U8X)
		return -EINVAL;

	if (client_data->mag_pw_mode != pw_mode) {
		mutex_lock(&client_data->mutex_bus_op);
		op_mode = client_data->op_mode;
		if (op_mode != OPERATION_MODE_CONFIG)
			ret = bgsh_set_op_mode(client_data,
				OPERATION_MODE_CONFIG);
		if (ret == SUCCESS) {
			MAKE_REG_NODE(node, BGSH_MAG_POWER_MODE);
			bgsh_write_node(client_data, node,
				pw_mode, &ret);
		}
		if (ret == SUCCESS)
			client_data->mag_pw_mode = pw_mode;
		if (op_mode != OPERATION_MODE_CONFIG)
			ret += bgsh_set_op_mode(client_data, op_mode);
		mutex_unlock(&client_data->mutex_bus_op);

		if (ret)
			return ret;
	}

	return count;
}

static ssize_t bgsh_show_mag_dr(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	uint8_t datarate = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	int32_t ret = SUCCESS;
	struct reg_node node;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_MAG_DATA_OUTRATE);
	bgsh_read_node(client_data, node, &datarate, 1, &ret);
	mutex_unlock(&client_data->mutex_bus_op);
	PDEBUG("%d", datarate);

	if (ret == SUCCESS)
		ret = sprintf(buf, "%d\n", datarate);
	else
		ret = -EIO;

	return ret;
}

static ssize_t bgsh_store_mag_dr(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long tmp;
	uint8_t datarate;
	int32_t ret = SUCCESS;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	uint8_t op_mode = OPERATION_MODE_CONFIG;
	struct reg_node node;

	ret = kstrtoul(buf, 10, &tmp);
	if (ret)
		return ret;
	PDEBUG("%ld", tmp);

	datarate = (uint8_t)tmp;
	if (datarate >= BGSH_Five_U8X)
		return -EINVAL;

	mutex_lock(&client_data->mutex_bus_op);
	op_mode = client_data->op_mode;
	if (op_mode != OPERATION_MODE_CONFIG)
		ret = bgsh_set_op_mode(client_data,
			OPERATION_MODE_CONFIG);
	if (ret == SUCCESS) {
		MAKE_REG_NODE(node, BGSH_MAG_DATA_OUTRATE);
		bgsh_write_node(client_data, node,
			datarate, &ret);
	}
	if (op_mode != OPERATION_MODE_CONFIG)
		ret += bgsh_set_op_mode(client_data, op_mode);
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret)
		return ret;

	return count;
}

static ssize_t bgsh_show_acc_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	uint8_t tmp = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	int32_t ret = SUCCESS;
	struct reg_node node;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_DATA_SEL_ACC);
	bgsh_read_node(client_data, node, &tmp, 1, &ret);
	mutex_unlock(&client_data->mutex_bus_op);
	PDEBUG("%d", tmp);

	if (ret == SUCCESS)
		ret = sprintf(buf, "%d\n", tmp);
	else
		ret = -EIO;

	return ret;
}

static ssize_t bgsh_store_acc_enable(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long tmp;
	uint8_t sel;
	int32_t ret = SUCCESS;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	struct reg_node node;

	ret = kstrtoul(buf, 10, &tmp);
	if (ret)
		return ret;
	PDEBUG("%ld", tmp);

	sel = (uint8_t)tmp;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_DATA_SEL_ACC);
	bgsh_write_node(client_data, node, sel, &ret);
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret)
		return ret;

	return count;
}

static ssize_t bgsh_show_gyro_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	uint8_t tmp = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	int32_t ret = SUCCESS;
	struct reg_node node;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_DATA_SEL_GYR);
	bgsh_read_node(client_data, node, &tmp, 1, &ret);
	mutex_unlock(&client_data->mutex_bus_op);
	PDEBUG("%d", tmp);

	if (ret == SUCCESS)
		ret = sprintf(buf, "%d\n", tmp);
	else
		ret = -EIO;

	return ret;
}

static ssize_t bgsh_store_gyro_enable(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long tmp;
	uint8_t sel;
	int32_t ret = SUCCESS;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	struct reg_node node;

	ret = kstrtoul(buf, 10, &tmp);
	if (ret)
		return ret;
	PDEBUG("%ld", tmp);

	sel = (uint8_t)tmp;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_DATA_SEL_GYR);
	bgsh_write_node(client_data, node, sel, &ret);
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret)
		return ret;

	return count;
}

static ssize_t bgsh_show_mag_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	uint8_t tmp = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	int32_t ret = SUCCESS;
	struct reg_node node;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_DATA_SEL_MAG);
	bgsh_read_node(client_data, node, &tmp, 1, &ret);
	mutex_unlock(&client_data->mutex_bus_op);
	PDEBUG("%d", tmp);

	if (ret == SUCCESS)
		ret = sprintf(buf, "%d\n", tmp);
	else
		ret = -EIO;

	return ret;
}

static ssize_t bgsh_store_mag_enable(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long tmp;
	uint8_t sel;
	int32_t ret = SUCCESS;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	struct reg_node node;

	ret = kstrtoul(buf, 10, &tmp);
	if (ret)
		return ret;
	PDEBUG("%ld", tmp);

	sel = (uint8_t)tmp;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_DATA_SEL_MAG);
	bgsh_write_node(client_data, node, sel, &ret);
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret)
		return ret;

	return count;
}

static ssize_t bgsh_show_euler_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	uint8_t tmp = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	int32_t ret = SUCCESS;
	struct reg_node node;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_DATA_SEL_EUL);
	bgsh_read_node(client_data, node, &tmp, 1, &ret);
	mutex_unlock(&client_data->mutex_bus_op);
	PDEBUG("%d", tmp);

	if (ret == SUCCESS)
		ret = sprintf(buf, "%d\n", tmp);
	else
		ret = -EIO;

	return ret;
}

static ssize_t bgsh_store_euler_enable(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long tmp;
	uint8_t sel;
	int32_t ret = SUCCESS;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	struct reg_node node;

	ret = kstrtoul(buf, 10, &tmp);
	if (ret)
		return ret;
	PDEBUG("%ld", tmp);

	sel = (uint8_t)tmp;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_DATA_SEL_EUL);
	bgsh_write_node(client_data, node, sel, &ret);
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret)
		return ret;

	return count;
}

static ssize_t bgsh_show_quaternion_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	uint8_t tmp = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	int32_t ret = SUCCESS;
	struct reg_node node;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_DATA_SEL_QUA);
	bgsh_read_node(client_data, node, &tmp, 1, &ret);
	mutex_unlock(&client_data->mutex_bus_op);
	PDEBUG("%d", tmp);

	if (ret == SUCCESS)
		ret = sprintf(buf, "%d\n", tmp);
	else
		ret = -EIO;

	return ret;
}

static ssize_t bgsh_store_quaternion_enable(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long tmp;
	uint8_t sel;
	int32_t ret = SUCCESS;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	struct reg_node node;

	ret = kstrtoul(buf, 10, &tmp);
	if (ret)
		return ret;
	PDEBUG("%ld", tmp);

	sel = (uint8_t)tmp;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_DATA_SEL_QUA);
	bgsh_write_node(client_data, node, sel, &ret);
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret)
		return ret;

	return count;
}

static ssize_t bgsh_show_linear_accel_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	uint8_t tmp = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	int32_t ret = SUCCESS;
	struct reg_node node;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_DATA_SEL_LINEAR_ACC);
	bgsh_read_node(client_data, node, &tmp, 1, &ret);
	mutex_unlock(&client_data->mutex_bus_op);
	PDEBUG("%d", tmp);

	if (ret == SUCCESS)
		ret = sprintf(buf, "%d\n", tmp);
	else
		ret = -EIO;

	return ret;
}

static ssize_t bgsh_store_linear_accel_enable(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long tmp;
	uint8_t sel;
	int32_t ret = SUCCESS;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	struct reg_node node;

	ret = kstrtoul(buf, 10, &tmp);
	if (ret)
		return ret;
	PDEBUG("%ld", tmp);

	sel = (uint8_t)tmp;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_DATA_SEL_LINEAR_ACC);
	bgsh_write_node(client_data, node, sel, &ret);
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret)
		return ret;

	return count;
}

static ssize_t bgsh_show_gravity_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	uint8_t tmp = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	int32_t ret = SUCCESS;
	struct reg_node node;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_DATA_SEL_GRV);
	bgsh_read_node(client_data, node, &tmp, 1, &ret);
	mutex_unlock(&client_data->mutex_bus_op);
	PDEBUG("%d", tmp);

	if (ret == SUCCESS)
		ret = sprintf(buf, "%d\n", tmp);
	else
		ret = -EIO;

	return ret;
}

static ssize_t bgsh_store_gravity_enable(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long tmp;
	uint8_t sel;
	int32_t ret = SUCCESS;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	struct reg_node node;

	ret = kstrtoul(buf, 10, &tmp);
	if (ret)
		return ret;
	PDEBUG("%ld", tmp);

	sel = (uint8_t)tmp;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_DATA_SEL_GRV);
	bgsh_write_node(client_data, node, sel, &ret);
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret)
		return ret;

	return count;
}

static ssize_t bgsh_show_temperature_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	uint8_t tmp = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	int32_t ret = SUCCESS;
	struct reg_node node;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_DATA_SEL_TEMP);
	bgsh_read_node(client_data, node, &tmp, 1, &ret);
	mutex_unlock(&client_data->mutex_bus_op);
	PDEBUG("%d", tmp);

	if (ret == SUCCESS)
		ret = sprintf(buf, "%d\n", tmp);
	else
		ret = -EIO;

	return ret;
}

static ssize_t bgsh_store_temperature_enable(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long tmp;
	uint8_t sel;
	int32_t ret = SUCCESS;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	struct reg_node node;

	ret = kstrtoul(buf, 10, &tmp);
	if (ret)
		return ret;
	PDEBUG("%ld", tmp);

	sel = (uint8_t)tmp;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_DATA_SEL_TEMP);
	bgsh_write_node(client_data, node, sel, &ret);
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret)
		return ret;

	return count;
}

static ssize_t bgsh_show_acc_offset_x(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	int32_t ret = SUCCESS;
	uint8_t data[2] = {0, 0};
	struct reg_node node;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_ACC_OFFSET_X_LSB);
	bgsh_read_node(client_data, node, data, 2, &ret);
	mutex_unlock(&client_data->mutex_bus_op);
	PDEBUG("%x %x", data[0], data[1]);

	if (ret == SUCCESS)
		ret = sprintf(buf, "%d\n",
			(BGSH_S16)((data[1]<<BGSH_SFT_8_POS) | data[0])
			);
	else
		ret = -EIO;

	return ret;
}

static ssize_t bgsh_store_acc_offset_x(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	signed long tmp;
	int32_t ret = SUCCESS;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	uint8_t op_mode = OPERATION_MODE_CONFIG;
	struct reg_node node;

	ret = kstrtol(buf, 10, &tmp);
	if (ret)
		return ret;
	PDEBUG("%ld", tmp);

	mutex_lock(&client_data->mutex_bus_op);
	op_mode = client_data->op_mode;
	if (op_mode != OPERATION_MODE_CONFIG)
		ret = bgsh_set_op_mode(client_data, OPERATION_MODE_CONFIG);
	if (ret == SUCCESS) {
		MAKE_REG_NODE(node, BGSH_ACC_OFFSET_X_LSB);
		bgsh_write_node(client_data, node,
			(uint8_t)(tmp & 0x00ff), &ret);
		MAKE_REG_NODE(node, BGSH_ACC_OFFSET_X_MSB);
		bgsh_write_node(client_data, node,
			(uint8_t)((tmp & 0xff00)>>BGSH_SFT_8_POS), &ret);

	}
	if (op_mode != OPERATION_MODE_CONFIG)
		ret += bgsh_set_op_mode(client_data, op_mode);
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret)
		return ret;

	return count;
}

static ssize_t bgsh_show_acc_offset_y(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	int32_t ret = SUCCESS;
	uint8_t data[2] = {0, 0};
	struct reg_node node;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_ACC_OFFSET_Y_LSB);
	bgsh_read_node(client_data, node, data, 2, &ret);
	mutex_unlock(&client_data->mutex_bus_op);
	PDEBUG("%x %x", data[0], data[1]);

	if (ret == SUCCESS)
		ret = sprintf(buf, "%d\n",
			(BGSH_S16)((data[1]<<BGSH_SFT_8_POS) | data[0])
			);
	else
		ret = -EIO;

	return ret;
}

static ssize_t bgsh_store_acc_offset_y(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	signed long tmp;
	int32_t ret = SUCCESS;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	uint8_t op_mode = OPERATION_MODE_CONFIG;
	struct reg_node node;

	ret = kstrtol(buf, 10, &tmp);
	if (ret)
		return ret;
	PDEBUG("%ld", tmp);

	mutex_lock(&client_data->mutex_bus_op);
	op_mode = client_data->op_mode;
	if (op_mode != OPERATION_MODE_CONFIG)
		ret = bgsh_set_op_mode(client_data, OPERATION_MODE_CONFIG);
	if (ret == SUCCESS) {
		MAKE_REG_NODE(node, BGSH_ACC_OFFSET_Y_LSB);
		bgsh_write_node(client_data, node,
			(uint8_t)(tmp & 0x00ff), &ret);
		MAKE_REG_NODE(node, BGSH_ACC_OFFSET_Y_MSB);
		bgsh_write_node(client_data, node,
			(uint8_t)((tmp & 0xff00)>>BGSH_SFT_8_POS), &ret);

	}
	if (op_mode != OPERATION_MODE_CONFIG)
		ret += bgsh_set_op_mode(client_data, op_mode);
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret)
		return ret;

	return count;
}

static ssize_t bgsh_show_acc_offset_z(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	int32_t ret = SUCCESS;
	uint8_t data[2] = {0, 0};
	struct reg_node node;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_ACC_OFFSET_Z_LSB);
	bgsh_read_node(client_data, node, data, 2, &ret);
	mutex_unlock(&client_data->mutex_bus_op);
	PDEBUG("%x %x", data[0], data[1]);

	if (ret == SUCCESS)
		ret = sprintf(buf, "%d\n",
			(BGSH_S16)((data[1]<<BGSH_SFT_8_POS) | data[0])
			);
	else
		ret = -EIO;

	return ret;
}

static ssize_t bgsh_store_acc_offset_z(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	signed long tmp;
	int32_t ret = SUCCESS;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	uint8_t op_mode = OPERATION_MODE_CONFIG;
	struct reg_node node;

	ret = kstrtol(buf, 10, &tmp);
	if (ret)
		return ret;
	PDEBUG("%ld", tmp);

	mutex_lock(&client_data->mutex_bus_op);
	op_mode = client_data->op_mode;
	if (op_mode != OPERATION_MODE_CONFIG)
		ret = bgsh_set_op_mode(client_data, OPERATION_MODE_CONFIG);
	if (ret == SUCCESS) {
		MAKE_REG_NODE(node, BGSH_ACC_OFFSET_Z_LSB);
		bgsh_write_node(client_data, node,
			(uint8_t)(tmp & 0x00ff), &ret);
		MAKE_REG_NODE(node, BGSH_ACC_OFFSET_Z_MSB);
		bgsh_write_node(client_data, node,
			(uint8_t)((tmp & 0xff00)>>BGSH_SFT_8_POS), &ret);

	}
	if (op_mode != OPERATION_MODE_CONFIG)
		ret += bgsh_set_op_mode(client_data, op_mode);
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret)
		return ret;

	return count;
}

static ssize_t bgsh_show_gyro_offset_x(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	int32_t ret = SUCCESS;
	uint8_t data[2] = {0, 0};
	struct reg_node node;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_GYR_OFFSET_X_LSB);
	bgsh_read_node(client_data, node, data, 2, &ret);
	mutex_unlock(&client_data->mutex_bus_op);
	PDEBUG("%x %x", data[0], data[1]);

	if (ret == SUCCESS)
		ret = sprintf(buf, "%d\n",
			(BGSH_S16)((data[1]<<BGSH_SFT_8_POS) | data[0])
			);
	else
		ret = -EIO;

	return ret;
}

static ssize_t bgsh_store_gyro_offset_x(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	signed long tmp;
	int32_t ret = SUCCESS;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	uint8_t op_mode = OPERATION_MODE_CONFIG;
	struct reg_node node;

	ret = kstrtol(buf, 10, &tmp);
	if (ret)
		return ret;
	PDEBUG("%ld", tmp);

	mutex_lock(&client_data->mutex_bus_op);
	op_mode = client_data->op_mode;
	if (op_mode != OPERATION_MODE_CONFIG)
		ret = bgsh_set_op_mode(client_data, OPERATION_MODE_CONFIG);
	if (ret == SUCCESS) {
		MAKE_REG_NODE(node, BGSH_GYR_OFFSET_X_LSB);
		bgsh_write_node(client_data, node,
			(uint8_t)(tmp & 0x00ff), &ret);
		MAKE_REG_NODE(node, BGSH_GYR_OFFSET_X_MSB);
		bgsh_write_node(client_data, node,
			(uint8_t)((tmp & 0xff00)>>BGSH_SFT_8_POS), &ret);

	}
	if (op_mode != OPERATION_MODE_CONFIG)
		ret += bgsh_set_op_mode(client_data, op_mode);
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret)
		return ret;

	return count;
}

static ssize_t bgsh_show_gyro_offset_y(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	int32_t ret = SUCCESS;
	uint8_t data[2] = {0, 0};
	struct reg_node node;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_GYR_OFFSET_Y_LSB);
	bgsh_read_node(client_data, node, data, 2, &ret);
	mutex_unlock(&client_data->mutex_bus_op);
	PDEBUG("%x %x", data[0], data[1]);

	if (ret == SUCCESS)
		ret = sprintf(buf, "%d\n",
			(BGSH_S16)((data[1]<<BGSH_SFT_8_POS) | data[0])
			);
	else
		ret = -EIO;

	return ret;
}


static ssize_t bgsh_store_gyro_offset_y(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	signed long tmp;
	int32_t ret = SUCCESS;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	uint8_t op_mode = OPERATION_MODE_CONFIG;
	struct reg_node node;

	ret = kstrtol(buf, 10, &tmp);
	if (ret)
		return ret;
	PDEBUG("%ld", tmp);

	mutex_lock(&client_data->mutex_bus_op);
	op_mode = client_data->op_mode;
	if (op_mode != OPERATION_MODE_CONFIG)
		ret = bgsh_set_op_mode(client_data, OPERATION_MODE_CONFIG);
	if (ret == SUCCESS) {
		MAKE_REG_NODE(node, BGSH_GYR_OFFSET_Y_LSB);
		bgsh_write_node(client_data, node,
			(uint8_t)(tmp & 0x00ff), &ret);
		MAKE_REG_NODE(node, BGSH_GYR_OFFSET_Y_MSB);
		bgsh_write_node(client_data, node,
			(uint8_t)((tmp & 0xff00)>>BGSH_SFT_8_POS), &ret);

	}
	if (op_mode != OPERATION_MODE_CONFIG)
		ret += bgsh_set_op_mode(client_data, op_mode);
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret)
		return ret;

	return count;
}

static ssize_t bgsh_show_gyro_offset_z(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	int32_t ret = SUCCESS;
	uint8_t data[2] = {0, 0};
	struct reg_node node;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_GYR_OFFSET_Z_LSB);
	bgsh_read_node(client_data, node, data, 2, &ret);
	mutex_unlock(&client_data->mutex_bus_op);
	PDEBUG("%x %x", data[0], data[1]);

	if (ret == SUCCESS)
		ret = sprintf(buf, "%d\n",
			(BGSH_S16)((data[1]<<BGSH_SFT_8_POS) | data[0])
			);
	else
		ret = -EIO;

	return ret;
}

static ssize_t bgsh_store_gyro_offset_z(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	signed long tmp;
	int32_t ret = SUCCESS;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	uint8_t op_mode = OPERATION_MODE_CONFIG;
	struct reg_node node;

	ret = kstrtol(buf, 10, &tmp);
	if (ret)
		return ret;
	PDEBUG("%ld", tmp);

	mutex_lock(&client_data->mutex_bus_op);
	op_mode = client_data->op_mode;
	if (op_mode != OPERATION_MODE_CONFIG)
		ret = bgsh_set_op_mode(client_data, OPERATION_MODE_CONFIG);
	if (ret == SUCCESS) {
		MAKE_REG_NODE(node, BGSH_GYR_OFFSET_Z_LSB);
		bgsh_write_node(client_data, node,
			(uint8_t)(tmp & 0x00ff), &ret);
		MAKE_REG_NODE(node, BGSH_GYR_OFFSET_Z_MSB);
		bgsh_write_node(client_data, node,
			(uint8_t)((tmp & 0xff00)>>BGSH_SFT_8_POS), &ret);

	}
	if (op_mode != OPERATION_MODE_CONFIG)
		ret += bgsh_set_op_mode(client_data, op_mode);
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret)
		return ret;

	return count;
}

static ssize_t bgsh_show_mag_offset_x(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	int32_t ret = SUCCESS;
	uint8_t data[2] = {0, 0};
	struct reg_node node;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_MAG_OFFSET_X_LSB);
	bgsh_read_node(client_data, node, data, 2, &ret);
	mutex_unlock(&client_data->mutex_bus_op);
	PDEBUG("%x %x", data[0], data[1]);

	if (ret == SUCCESS)
		ret = sprintf(buf, "%d\n",
			(BGSH_S16)((data[1]<<BGSH_SFT_8_POS) | data[0])
			);
	else
		ret = -EIO;

	return ret;
}

static ssize_t bgsh_store_mag_offset_x(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	signed long tmp;
	int32_t ret = SUCCESS;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	uint8_t op_mode = OPERATION_MODE_CONFIG;
	struct reg_node node;

	ret = kstrtol(buf, 10, &tmp);
	if (ret)
		return ret;
	PDEBUG("%ld", tmp);

	mutex_lock(&client_data->mutex_bus_op);
	op_mode = client_data->op_mode;
	if (op_mode != OPERATION_MODE_CONFIG)
		ret = bgsh_set_op_mode(client_data, OPERATION_MODE_CONFIG);
	if (ret == SUCCESS) {
		MAKE_REG_NODE(node, BGSH_MAG_OFFSET_X_LSB);
		bgsh_write_node(client_data, node,
			(uint8_t)(tmp & 0x00ff), &ret);
		MAKE_REG_NODE(node, BGSH_MAG_OFFSET_X_MSB);
		bgsh_write_node(client_data, node,
			(uint8_t)((tmp & 0xff00)>>BGSH_SFT_8_POS), &ret);

	}
	if (op_mode != OPERATION_MODE_CONFIG)
		ret += bgsh_set_op_mode(client_data, op_mode);
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret)
		return ret;

	return count;
}

static ssize_t bgsh_show_mag_offset_y(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	int32_t ret = SUCCESS;
	uint8_t data[2] = {0, 0};
	struct reg_node node;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_MAG_OFFSET_Y_LSB);
	bgsh_read_node(client_data, node, data, 2, &ret);
	mutex_unlock(&client_data->mutex_bus_op);
	PDEBUG("%x %x", data[0], data[1]);

	if (ret == SUCCESS)
		ret = sprintf(buf, "%d\n",
			(BGSH_S16)((data[1]<<BGSH_SFT_8_POS) | data[0])
			);
	else
		ret = -EIO;

	return ret;
}

static ssize_t bgsh_store_mag_offset_y(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	signed long tmp;
	int32_t ret = SUCCESS;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	uint8_t op_mode = OPERATION_MODE_CONFIG;
	struct reg_node node;

	ret = kstrtol(buf, 10, &tmp);
	if (ret)
		return ret;
	PDEBUG("%ld", tmp);

	mutex_lock(&client_data->mutex_bus_op);
	op_mode = client_data->op_mode;
	if (op_mode != OPERATION_MODE_CONFIG)
		ret = bgsh_set_op_mode(client_data, OPERATION_MODE_CONFIG);
	if (ret == SUCCESS) {
		MAKE_REG_NODE(node, BGSH_MAG_OFFSET_Y_LSB);
		bgsh_write_node(client_data, node,
			(uint8_t)(tmp & 0x00ff), &ret);
		MAKE_REG_NODE(node, BGSH_MAG_OFFSET_Y_MSB);
		bgsh_write_node(client_data, node,
			(uint8_t)((tmp & 0xff00)>>BGSH_SFT_8_POS), &ret);

	}
	if (op_mode != OPERATION_MODE_CONFIG)
		ret += bgsh_set_op_mode(client_data, op_mode);
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret)
		return ret;

	return count;
}

static ssize_t bgsh_show_mag_offset_z(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	int32_t ret = SUCCESS;
	uint8_t data[2] = {0, 0};
	struct reg_node node;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_MAG_OFFSET_Z_LSB);
	bgsh_read_node(client_data, node, data, 2, &ret);
	mutex_unlock(&client_data->mutex_bus_op);
	PDEBUG("%x %x", data[0], data[1]);

	if (ret == SUCCESS)
		ret = sprintf(buf, "%d\n",
			(BGSH_S16)((data[1]<<BGSH_SFT_8_POS) | data[0])
			);
	else
		ret = -EIO;

	return ret;
}

static ssize_t bgsh_store_mag_offset_z(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	signed long tmp;
	int32_t ret = SUCCESS;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	uint8_t op_mode = OPERATION_MODE_CONFIG;
	struct reg_node node;

	ret = kstrtol(buf, 10, &tmp);
	if (ret)
		return ret;
	PDEBUG("%ld", tmp);

	mutex_lock(&client_data->mutex_bus_op);
	op_mode = client_data->op_mode;
	if (op_mode != OPERATION_MODE_CONFIG)
		ret = bgsh_set_op_mode(client_data, OPERATION_MODE_CONFIG);
	if (ret == SUCCESS) {
		MAKE_REG_NODE(node, BGSH_MAG_OFFSET_Z_LSB);
		bgsh_write_node(client_data, node,
			(uint8_t)(tmp & 0x00ff), &ret);
		MAKE_REG_NODE(node, BGSH_MAG_OFFSET_Z_MSB);
		bgsh_write_node(client_data, node,
			(uint8_t)((tmp & 0xff00)>>BGSH_SFT_8_POS), &ret);
	}
	if (op_mode != OPERATION_MODE_CONFIG)
		ret += bgsh_set_op_mode(client_data, op_mode);
	mutex_unlock(&client_data->mutex_bus_op);

	if (ret)
		return ret;

	return count;
}

static ssize_t bgsh_show_sic_matrix(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	uint8_t data[18];
	int32_t ret = SUCCESS;
	struct reg_node node;

	mutex_lock(&client_data->mutex_bus_op);
	MAKE_REG_NODE(node, BGSH_SIC_MATRIX_0_LSB);
	bgsh_read_node(client_data, node, data, 18, &ret);
	if (ret)
		ret = -EIO;
	mutex_unlock(&client_data->mutex_bus_op);

	if (!ret)
		ret = sprintf(buf, "%d %d %d %d %d %d %d %d %d\n",
			(BGSH_S16)((data[1]<<BGSH_SFT_8_POS) | data[0]),
			(BGSH_S16)((data[3]<<BGSH_SFT_8_POS) | data[2]),
			(BGSH_S16)((data[5]<<BGSH_SFT_8_POS) | data[4]),
			(BGSH_S16)((data[7]<<BGSH_SFT_8_POS) | data[6]),
			(BGSH_S16)((data[9]<<BGSH_SFT_8_POS) | data[8]),
			(BGSH_S16)((data[11]<<BGSH_SFT_8_POS) | data[10]),
			(BGSH_S16)((data[13]<<BGSH_SFT_8_POS) | data[12]),
			(BGSH_S16)((data[15]<<BGSH_SFT_8_POS) | data[14]),
			(BGSH_S16)((data[17]<<BGSH_SFT_8_POS) | data[16])
			);

	return ret;
}

#if BGSH_SIC_MATRIX_WRITE_IN_ONE_COMMAND == 1
static ssize_t bgsh_store_sic_matrix(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	unsigned int data[9];
	int32_t ret = SUCCESS;
	int32_t i;
	uint8_t op_mode = OPERATION_MODE_CONFIG;
	struct reg_node node;

	ret = sscanf(buf,
		"%11x %11x %11x %11x %11x %11x %11x %11x %11x",
		&data[0], &data[1], &data[2],
		&data[3], &data[4], &data[5],
		&data[6], &data[7], &data[8]
		);

	PINFO("num %d,SIC:%x %x %x %x %x %x %x %x %x",
		ret,
		data[0], data[1], data[2],
		data[3], data[4], data[5],
		data[6], data[7], data[8]
		);
	if (ret < 9)
		return ret;
	i = 0;
	while (i < 9) {
		if ((data[i] & 0xffff0000) != 0)
			return -EINVAL;
		i++;
	}

	mutex_lock(&client_data->mutex_bus_op);

	op_mode = client_data->op_mode;
	if (op_mode != OPERATION_MODE_CONFIG)
		ret = bgsh_set_op_mode(client_data, OPERATION_MODE_CONFIG);
	if (ret == SUCCESS) {
		MAKE_REG_NODE(node, BGSH_SIC_MATRIX_0_LSB);
		bgsh_write_node(client_data, node,
			(uint8_t)(data[0] & 0x00ff), &ret);
		MAKE_REG_NODE(node, BGSH_SIC_MATRIX_0_MSB);
		bgsh_write_node(client_data, node,
			(uint8_t)((data[0] & 0xff00)>>BGSH_SFT_8_POS),
			&ret);
		MAKE_REG_NODE(node, BGSH_SIC_MATRIX_1_LSB);
		bgsh_write_node(client_data, node,
			(uint8_t)(data[1] & 0x00ff), &ret);
		MAKE_REG_NODE(node, BGSH_SIC_MATRIX_1_MSB);
		bgsh_write_node(client_data, node,
			(uint8_t)((data[1] & 0xff00)>>BGSH_SFT_8_POS),
			&ret);
		MAKE_REG_NODE(node, BGSH_SIC_MATRIX_2_LSB);
		bgsh_write_node(client_data, node,
			(uint8_t)(data[2] & 0x00ff), &ret);
		MAKE_REG_NODE(node, BGSH_SIC_MATRIX_2_MSB);
		bgsh_write_node(client_data, node,
			(uint8_t)((data[2] & 0xff00)>>BGSH_SFT_8_POS),
			&ret);
		MAKE_REG_NODE(node, BGSH_SIC_MATRIX_3_LSB);
		bgsh_write_node(client_data, node,
			(uint8_t)(data[3] & 0x00ff), &ret);
		MAKE_REG_NODE(node, BGSH_SIC_MATRIX_3_MSB);
		bgsh_write_node(client_data, node,
			(uint8_t)((data[3] & 0xff00)>>BGSH_SFT_8_POS),
			&ret);
		MAKE_REG_NODE(node, BGSH_SIC_MATRIX_4_LSB);
		bgsh_write_node(client_data, node,
			(uint8_t)(data[4] & 0x00ff), &ret);
		MAKE_REG_NODE(node, BGSH_SIC_MATRIX_4_MSB);
		bgsh_write_node(client_data, node,
			(uint8_t)((data[4] & 0xff00)>>BGSH_SFT_8_POS),
			&ret);
		MAKE_REG_NODE(node, BGSH_SIC_MATRIX_5_LSB);
		bgsh_write_node(client_data, node,
			(uint8_t)(data[5] & 0x00ff), &ret);
		MAKE_REG_NODE(node, BGSH_SIC_MATRIX_5_MSB);
		bgsh_write_node(client_data, node,
			(uint8_t)((data[5] & 0xff00)>>BGSH_SFT_8_POS),
			&ret);
		MAKE_REG_NODE(node, BGSH_SIC_MATRIX_6_LSB);
		bgsh_write_node(client_data, node,
			(uint8_t)(data[6] & 0x00ff), &ret);
		MAKE_REG_NODE(node, BGSH_SIC_MATRIX_6_MSB);
		bgsh_write_node(client_data, node,
			(uint8_t)((data[6] & 0xff00)>>BGSH_SFT_8_POS),
			&ret);
		MAKE_REG_NODE(node, BGSH_SIC_MATRIX_7_LSB);
		bgsh_write_node(client_data, node,
			(uint8_t)(data[7] & 0x00ff), &ret);
		MAKE_REG_NODE(node, BGSH_SIC_MATRIX_7_MSB);
		bgsh_write_node(client_data, node,
			(uint8_t)((data[7] & 0xff00)>>BGSH_SFT_8_POS),
			&ret);
		MAKE_REG_NODE(node, BGSH_SIC_MATRIX_8_LSB);
		bgsh_write_node(client_data, node,
			(uint8_t)(data[8] & 0x00ff), &ret);
		MAKE_REG_NODE(node, BGSH_SIC_MATRIX_8_MSB);
		bgsh_write_node(client_data, node,
			(uint8_t)((data[8] & 0xff00)>>BGSH_SFT_8_POS),
			&ret);
	}
	if (op_mode != OPERATION_MODE_CONFIG)
		ret += bgsh_set_op_mode(client_data, op_mode);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret)
		return ret;

	return count;
}
#endif

static ssize_t bgsh_store_sic_matrix_zero(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	unsigned long data;
	int32_t ret = SUCCESS;
	uint8_t op_mode = OPERATION_MODE_CONFIG;
	struct reg_node node;

	ret = kstrtoul(buf, 10, &data);
	if (ret)
		return ret;
	if ((data & 0xffff0000) != 0)
		return -EINVAL;

	mutex_lock(&client_data->mutex_bus_op);
	op_mode = client_data->op_mode;
	if (op_mode != OPERATION_MODE_CONFIG)
		ret = bgsh_set_op_mode(client_data, OPERATION_MODE_CONFIG);
	if (ret == SUCCESS) {
		MAKE_REG_NODE(node, BGSH_SIC_MATRIX_0_LSB);
		bgsh_write_node(client_data, node,
			(uint8_t)(data & 0x00ff), &ret);
		MAKE_REG_NODE(node, BGSH_SIC_MATRIX_0_MSB);
		bgsh_write_node(client_data, node,
			(uint8_t)((data & 0xff00)>>BGSH_SFT_8_POS), &ret);
	}
	if (op_mode != OPERATION_MODE_CONFIG)
		ret += bgsh_set_op_mode(client_data, op_mode);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret)
		return ret;

	return count;
}

static ssize_t bgsh_store_sic_matrix_one(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	unsigned long data;
	int32_t ret = SUCCESS;
	uint8_t op_mode = OPERATION_MODE_CONFIG;
	struct reg_node node;

	ret = kstrtoul(buf, 10, &data);
	if (ret)
		return ret;
	if ((data & 0xffff0000) != 0)
		return -EINVAL;

	mutex_lock(&client_data->mutex_bus_op);
	op_mode = client_data->op_mode;
	if (op_mode != OPERATION_MODE_CONFIG)
		ret = bgsh_set_op_mode(client_data, OPERATION_MODE_CONFIG);
	if (ret == SUCCESS) {
		MAKE_REG_NODE(node, BGSH_SIC_MATRIX_1_LSB);
		bgsh_write_node(client_data, node,
			(uint8_t)(data & 0x00ff), &ret);
		MAKE_REG_NODE(node, BGSH_SIC_MATRIX_1_MSB);
		bgsh_write_node(client_data, node,
			(uint8_t)((data & 0xff00)>>BGSH_SFT_8_POS), &ret);
	}
	if (op_mode != OPERATION_MODE_CONFIG)
		ret += bgsh_set_op_mode(client_data, op_mode);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret)
		return ret;

	return count;
}

static ssize_t bgsh_store_sic_matrix_two(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	unsigned long data;
	int32_t ret = SUCCESS;
	uint8_t op_mode = OPERATION_MODE_CONFIG;
	struct reg_node node;

	ret = kstrtoul(buf, 10, &data);
	if (ret)
		return ret;
	if ((data & 0xffff0000) != 0)
		return -EINVAL;

	mutex_lock(&client_data->mutex_bus_op);
	op_mode = client_data->op_mode;
	if (op_mode != OPERATION_MODE_CONFIG)
		ret = bgsh_set_op_mode(client_data, OPERATION_MODE_CONFIG);
	if (ret == SUCCESS) {
		MAKE_REG_NODE(node, BGSH_SIC_MATRIX_2_LSB);
		bgsh_write_node(client_data, node,
			(uint8_t)(data & 0x00ff), &ret);
		MAKE_REG_NODE(node, BGSH_SIC_MATRIX_2_MSB);
		bgsh_write_node(client_data, node,
			(uint8_t)((data & 0xff00)>>BGSH_SFT_8_POS), &ret);
	}
	if (op_mode != OPERATION_MODE_CONFIG)
		ret += bgsh_set_op_mode(client_data, op_mode);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret)
		return ret;

	return count;
}

static ssize_t bgsh_store_sic_matrix_three(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	unsigned long data;
	int32_t ret = SUCCESS;
	uint8_t op_mode = OPERATION_MODE_CONFIG;
	struct reg_node node;

	ret = kstrtoul(buf, 10, &data);
	if (ret)
		return ret;
	if ((data & 0xffff0000) != 0)
		return -EINVAL;

	mutex_lock(&client_data->mutex_bus_op);
	op_mode = client_data->op_mode;
	if (op_mode != OPERATION_MODE_CONFIG)
		ret = bgsh_set_op_mode(client_data, OPERATION_MODE_CONFIG);
	if (ret == SUCCESS) {
		MAKE_REG_NODE(node, BGSH_SIC_MATRIX_3_LSB);
		bgsh_write_node(client_data, node,
			(uint8_t)(data & 0x00ff), &ret);
		MAKE_REG_NODE(node, BGSH_SIC_MATRIX_3_MSB);
		bgsh_write_node(client_data, node,
			(uint8_t)((data & 0xff00)>>BGSH_SFT_8_POS), &ret);
	}
	if (op_mode != OPERATION_MODE_CONFIG)
		ret += bgsh_set_op_mode(client_data, op_mode);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret)
		return ret;

	return count;
}


static ssize_t bgsh_store_sic_matrix_four(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	unsigned long data;
	int32_t ret = SUCCESS;
	uint8_t op_mode = OPERATION_MODE_CONFIG;
	struct reg_node node;

	ret = kstrtoul(buf, 10, &data);
	if (ret)
		return ret;
	if ((data & 0xffff0000) != 0)
		return -EINVAL;

	mutex_lock(&client_data->mutex_bus_op);
	op_mode = client_data->op_mode;
	if (op_mode != OPERATION_MODE_CONFIG)
		ret = bgsh_set_op_mode(client_data, OPERATION_MODE_CONFIG);
	if (ret == SUCCESS) {
		MAKE_REG_NODE(node, BGSH_SIC_MATRIX_4_LSB);
		bgsh_write_node(client_data, node,
			(uint8_t)(data & 0x00ff), &ret);
		MAKE_REG_NODE(node, BGSH_SIC_MATRIX_4_MSB);
		bgsh_write_node(client_data, node,
			(uint8_t)((data & 0xff00)>>BGSH_SFT_8_POS), &ret);
	}
	if (op_mode != OPERATION_MODE_CONFIG)
		ret += bgsh_set_op_mode(client_data, op_mode);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret)
		return ret;

	return count;
}

static ssize_t bgsh_store_sic_matrix_five(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	unsigned long data;
	int32_t ret = SUCCESS;
	uint8_t op_mode = OPERATION_MODE_CONFIG;
	struct reg_node node;

	ret = kstrtoul(buf, 10, &data);
	if (ret)
		return ret;
	if ((data & 0xffff0000) != 0)
		return -EINVAL;

	mutex_lock(&client_data->mutex_bus_op);
	op_mode = client_data->op_mode;
	if (op_mode != OPERATION_MODE_CONFIG)
		ret = bgsh_set_op_mode(client_data, OPERATION_MODE_CONFIG);
	if (ret == SUCCESS) {
		MAKE_REG_NODE(node, BGSH_SIC_MATRIX_5_LSB);
		bgsh_write_node(client_data, node,
			(uint8_t)(data & 0x00ff), &ret);
		MAKE_REG_NODE(node, BGSH_SIC_MATRIX_5_MSB);
		bgsh_write_node(client_data, node,
			(uint8_t)((data & 0xff00)>>BGSH_SFT_8_POS), &ret);
	}
	if (op_mode != OPERATION_MODE_CONFIG)
		ret += bgsh_set_op_mode(client_data, op_mode);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret)
		return ret;

	return count;
}

static ssize_t bgsh_store_sic_matrix_six(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	unsigned long data;
	int32_t ret = SUCCESS;
	uint8_t op_mode = OPERATION_MODE_CONFIG;
	struct reg_node node;

	ret = kstrtoul(buf, 10, &data);
	if (ret)
		return ret;
	if ((data & 0xffff0000) != 0)
		return -EINVAL;

	mutex_lock(&client_data->mutex_bus_op);
	op_mode = client_data->op_mode;
	if (op_mode != OPERATION_MODE_CONFIG)
		ret = bgsh_set_op_mode(client_data, OPERATION_MODE_CONFIG);
	if (ret == SUCCESS) {
		MAKE_REG_NODE(node, BGSH_SIC_MATRIX_6_LSB);
		bgsh_write_node(client_data, node,
			(uint8_t)(data & 0x00ff), &ret);
		MAKE_REG_NODE(node, BGSH_SIC_MATRIX_6_MSB);
		bgsh_write_node(client_data, node,
			(uint8_t)((data & 0xff00)>>BGSH_SFT_8_POS), &ret);
	}
	if (op_mode != OPERATION_MODE_CONFIG)
		ret += bgsh_set_op_mode(client_data, op_mode);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret)
		return ret;

	return count;
}

static ssize_t bgsh_store_sic_matrix_seven(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	unsigned long data;
	int32_t ret = SUCCESS;
	uint8_t op_mode = OPERATION_MODE_CONFIG;
	struct reg_node node;

	ret = kstrtoul(buf, 10, &data);
	if (ret)
		return ret;
	if ((data & 0xffff0000) != 0)
		return -EINVAL;

	mutex_lock(&client_data->mutex_bus_op);
	op_mode = client_data->op_mode;
	if (op_mode != OPERATION_MODE_CONFIG)
		ret = bgsh_set_op_mode(client_data, OPERATION_MODE_CONFIG);
	if (ret == SUCCESS) {
		MAKE_REG_NODE(node, BGSH_SIC_MATRIX_7_LSB);
		bgsh_write_node(client_data, node,
			(uint8_t)(data & 0x00ff), &ret);
		MAKE_REG_NODE(node, BGSH_SIC_MATRIX_7_MSB);
		bgsh_write_node(client_data, node,
			(uint8_t)((data & 0xff00)>>BGSH_SFT_8_POS), &ret);
	}
	if (op_mode != OPERATION_MODE_CONFIG)
		ret += bgsh_set_op_mode(client_data, op_mode);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret)
		return ret;

	return count;
}

static ssize_t bgsh_store_sic_matrix_eight(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	unsigned long data;
	int32_t ret = SUCCESS;
	uint8_t op_mode = OPERATION_MODE_CONFIG;
	struct reg_node node;

	ret = kstrtoul(buf, 10, &data);
	if (ret)
		return ret;
	if ((data & 0xffff0000) != 0)
		return -EINVAL;

	mutex_lock(&client_data->mutex_bus_op);
	op_mode = client_data->op_mode;
	if (op_mode != OPERATION_MODE_CONFIG)
		ret = bgsh_set_op_mode(client_data, OPERATION_MODE_CONFIG);
	if (ret == SUCCESS) {
		MAKE_REG_NODE(node, BGSH_SIC_MATRIX_8_LSB);
		bgsh_write_node(client_data, node,
			(uint8_t)(data & 0x00ff), &ret);
		MAKE_REG_NODE(node, BGSH_SIC_MATRIX_8_MSB);
		bgsh_write_node(client_data, node,
			(uint8_t)((data & 0xff00)>>BGSH_SFT_8_POS), &ret);
	}
	if (op_mode != OPERATION_MODE_CONFIG)
		ret += bgsh_set_op_mode(client_data, op_mode);
	mutex_unlock(&client_data->mutex_bus_op);
	if (ret)
		return ret;

	return count;
}

#ifdef BGSH_DEBUG
uint8_t reg_op_pageid;
uint8_t reg_op_addr;

static ssize_t bgsh_show_register(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct bgsh_client_data *client_data =
		dev_get_drvdata(dev);

	int32_t ret = SUCCESS;

	mutex_lock(&client_data->mutex_bus_op);
	bgsh_dump_reg(client_data);
	mutex_unlock(&client_data->mutex_bus_op);

	ret = sprintf(buf, "OK\n");

	return ret;
}

static ssize_t bgsh_store_register(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int32_t ret = SUCCESS;
	unsigned int reg_pageid = 0;
	unsigned int reg_addr = 0;
	unsigned int data;
	uint8_t val;
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);

	ret = sscanf(buf, "%11x %11x %11x",
		&reg_pageid, &reg_addr, &data);
	PINFO("%d. reg,id:%x addr:%x data:%x",
		ret, reg_pageid, reg_addr, data);
	if (ret < 3)
		return ret;

	if (data > 0xff) {
		PERR("value error");
		return -EINVAL;
	}

	val = (uint8_t)data;
	reg_op_pageid = (uint8_t)reg_pageid;
	reg_op_addr = (uint8_t)reg_addr;
	if (reg_op_pageid > BGSH_REGISTER_PAGEID_MAX) {
		PERR("fail, pageid must be <= BGSH_REGISTER_PAGEID_MAX");
		reg_op_pageid = 0;
		return -EINVAL;
	}
	if (reg_op_addr >= BGSH_REGISTER_MAP_END) {
		PERR("addr set error");
		reg_op_addr = 0;
		return -EINVAL;
	}

	mutex_lock(&client_data->mutex_bus_op);
	ret = bgsh_write_register(client_data,
		reg_op_pageid, reg_op_addr, &val, 1);
	mutex_unlock(&client_data->mutex_bus_op);

	if (!ret) {
		PINFO("write reg 0x%2x at page %d, value= 0x%2x",
			reg_op_addr, reg_op_pageid, val);
	} else {
		PERR("write reg fail");
	}

	return count;
}

static ssize_t bgsh_show_reg_op_pageid(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int32_t ret = SUCCESS;

	ret = sprintf(buf, "reg_op, pageid: %x, addr=0x%2x\n",
		reg_op_pageid, reg_op_addr);
	PDEBUG("reg_op, pageid: %x, addr=0x%2x",
		reg_op_pageid, reg_op_addr);

	return ret;
}

static ssize_t bgsh_store_reg_op_pageid(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int32_t ret = SUCCESS;
	unsigned long pageid;
	ret = kstrtoul(buf, 16, &pageid);
	if (ret) {
		PERR("try read reg, xfer str fail");
		return ret;
	}

	reg_op_pageid = (uint8_t)pageid;

	if (reg_op_pageid > BGSH_REGISTER_PAGEID_MAX) {
		PERR("fail, pageid must be <= BGSH_REGISTER_PAGEID_MAX");
		reg_op_pageid = 0;
		return -EINVAL;
	}

	PINFO("reg_op, pageid:%x, addr=0x%2x, "
		"please read/write reg_op_value node",
		reg_op_pageid, reg_op_addr);

	return count;
}

static ssize_t bgsh_show_reg_op_addr(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int32_t ret = SUCCESS;

	ret = sprintf(buf, "reg_op, pageid: %x, addr=0x%2x\n",
		reg_op_pageid, reg_op_addr);
	PDEBUG("reg_op, pageid: %x, addr=0x%2x",
		reg_op_pageid, reg_op_addr);

	return ret;
}

static ssize_t bgsh_store_reg_op_addr(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int32_t ret = SUCCESS;
	unsigned long addr;
	ret = kstrtoul(buf, 16, &addr);
	if (ret) {
		PERR("try read reg, xfer str fail");
		return ret;
	}
	if (addr >= BGSH_REGISTER_MAP_END) {
		PERR("addr set error");
		return -EINVAL;
	}

	reg_op_addr = (uint8_t)addr;

	PINFO("reg_op, pageid: %x, addr=%x, "
		"please read/write reg_op_value node",
		reg_op_pageid, reg_op_addr);

	return count;
}


static ssize_t bgsh_show_reg_op_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	int32_t ret = SUCCESS;
	uint8_t data;

	mutex_lock(&client_data->mutex_bus_op);
	ret = bgsh_read_register(client_data,
		reg_op_pageid, reg_op_addr, &data, 1);
	mutex_unlock(&client_data->mutex_bus_op);

	if (!ret) {
		ret = sprintf(buf, "read reg 0x%2x at page %d,"
			"value= 0x%2x\n", reg_op_addr,
			reg_op_pageid, data);
		PDEBUG("read reg 0x%2x at page %d, value= 0x%2x",
			reg_op_addr, reg_op_pageid, data);
	} else {
		ret = sprintf(buf, "read reg fail\n");
		PERR("read reg fail");
	}

	return ret;
}

static ssize_t bgsh_store_reg_op_value(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	int32_t ret = SUCCESS;
	unsigned long data;
	uint8_t val;

	ret = kstrtoul(buf, 16, &data);
	if (ret) {
		PERR("try read reg, xfer str fail");
		return ret;
	}
	if (data > 0xff) {
		PERR("value error");
		return -EINVAL;
	}

	val = (uint8_t)data;
	mutex_lock(&client_data->mutex_bus_op);
	ret = bgsh_write_register(client_data,
		reg_op_pageid, reg_op_addr, &val, 1);
	mutex_unlock(&client_data->mutex_bus_op);

	PINFO("write reg 0x%2x at page %d, value= 0x%2x",
		reg_op_addr, reg_op_pageid, val);
	if (ret)
		PERR("write reg fail");

	return count;
}
#endif

#if FW_DOWNLOAD_SUPPORT == 1
static int32_t fwdl_checkcrc_8bits(char *buffer, uint8_t length)
{
	uint8_t i;
	uint16_t crc = 0;

	for (i = 0; i < length - 1; i++)
		crc += buffer[i];

	if (((crc + buffer[i]) & 0x00ff) == 0)
		return 0;
	else {
		PERR("crc fail: %x %x", crc, buffer[i]);
		PDEBUG_FWDL("len %d, buffer: %x %x %x %x %x %x %x %x %x %x "
			"%x %x %x %x %x %x %x %x %x %x %x %x",
			length, *buffer, *(buffer+1), *(buffer+2), *(buffer+3),
			*(buffer+4), *(buffer+5), *(buffer+6), *(buffer+7),
			*(buffer+8), *(buffer+9), *(buffer+10), *(buffer+11),
			*(buffer+12), *(buffer+13), *(buffer+14), *(buffer+15),
			*(buffer+16), *(buffer+17), *(buffer+18), *(buffer+19),
			*(buffer+20), *(buffer+21));
		return -1;
	}
}

static int32_t fwdl_load_char(char buffer, uint8_t *num, uint32_t i)
{
	if ((buffer >= '0') && (buffer <= '9'))
		*num = buffer - '0';
	else if ((buffer >= 'a') && (buffer <= 'f'))
		*num = buffer - 'a' + 10;
	else if ((buffer >= 'A') && (buffer <= 'F'))
		*num = buffer - 'A' + 10;
	else {
		PERR("file format fail: @%d, char: %d, %c",
			i, buffer, buffer);
		return -EINVAL;
	}

	return SUCCESS;
}

/* to check the file is valid or not */
static int32_t fwdl_checkfile(char *filename, char *buffer,
	int32_t *binsize, uint8_t mode)
{
	struct file *fd;
	mm_segment_t old_fs;
	loff_t pos = 0;

	int32_t filesize;
	char *filebuffer = NULL;
	char buf_line[22];
	uint8_t len_line, j;
	int32_t ret;
	uint32_t i, address, page_address;
	uint8_t num_h, num_l;

	PINFO("Check file: %s", filename);
	filebuffer = kmalloc(BGSH_FW_FILE_SIZE * sizeof(char), GFP_KERNEL);
	if (filebuffer == NULL) {
		PERR("malloc file mem fail");
		return -EIO;
	}

	fd = filp_open(filename, O_RDONLY, 0);
	if (IS_ERR(fd)) {
		PERR("open file fail");
		kfree(filebuffer);
		return -EIO;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	filesize = vfs_read(fd, filebuffer, BGSH_FW_FILE_SIZE, &pos);
	set_fs(old_fs);
	filp_close(fd, NULL);
	if (filesize <= 0) {
		PERR("read file fail");
		kfree(filebuffer);
		return -EIO;
	}

	PDEBUG_FWDL("FW file: "
		"%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x "
		"%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x "
		"%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x",
		*filebuffer, *(filebuffer+1), *(filebuffer+2),
		*(filebuffer+3), *(filebuffer+4), *(filebuffer+5),
		*(filebuffer+6), *(filebuffer+7), *(filebuffer+8),
		*(filebuffer+9), *(filebuffer+10), *(filebuffer+11),
		*(filebuffer+12), *(filebuffer+13), *(filebuffer+14),
		*(filebuffer+15), *(filebuffer+16), *(filebuffer+17),
		*(filebuffer+18), *(filebuffer+19), *(filebuffer+20),
		*(filebuffer+21), *(filebuffer+22), *(filebuffer+23),
		*(filebuffer+24), *(filebuffer+25), *(filebuffer+26),
		*(filebuffer+27), *(filebuffer+28), *(filebuffer+29),
		*(filebuffer+30), *(filebuffer+31), *(filebuffer+32),
		*(filebuffer+33), *(filebuffer+34), *(filebuffer+35),
		*(filebuffer+36), *(filebuffer+37), *(filebuffer+38),
		*(filebuffer+39), *(filebuffer+40), *(filebuffer+41),
		*(filebuffer+42), *(filebuffer+43), *(filebuffer+44),
		*(filebuffer+45), *(filebuffer+46), *(filebuffer+47)
		);
	PDEBUG_FWDL("FW file in char: "
		"%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c"
		"%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c"
		"%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c",
		*filebuffer, *(filebuffer+1), *(filebuffer+2),
		*(filebuffer+3), *(filebuffer+4), *(filebuffer+5),
		*(filebuffer+6), *(filebuffer+7), *(filebuffer+8),
		*(filebuffer+9), *(filebuffer+10), *(filebuffer+11),
		*(filebuffer+12), *(filebuffer+13), *(filebuffer+14),
		*(filebuffer+15), *(filebuffer+16), *(filebuffer+17),
		*(filebuffer+18), *(filebuffer+19), *(filebuffer+20),
		*(filebuffer+21), *(filebuffer+22), *(filebuffer+23),
		*(filebuffer+24), *(filebuffer+25), *(filebuffer+26),
		*(filebuffer+27), *(filebuffer+28), *(filebuffer+29),
		*(filebuffer+30), *(filebuffer+31), *(filebuffer+32),
		*(filebuffer+33), *(filebuffer+34), *(filebuffer+35),
		*(filebuffer+36), *(filebuffer+37), *(filebuffer+38),
		*(filebuffer+39), *(filebuffer+40), *(filebuffer+41),
		*(filebuffer+42), *(filebuffer+43), *(filebuffer+44),
		*(filebuffer+45), *(filebuffer+46), *(filebuffer+47)
		);

	/* analyze the file buffer and fill real data to buffer */
	i = 0;
	page_address = 0x000000;
	memset(buffer, 0xff, BGSH_FW_BIN_SIZE);
	while (1) {
		/* start symble*/
		if (*(filebuffer+i) != ':') {
			PERR("file format fail: @%d, header=%d",
				i, *(filebuffer+i));
			kfree(filebuffer);
			return -EINVAL;
		}

		/* length per record */
		ret = fwdl_load_char(*(filebuffer+i+1), &num_h, i+1);
		if (ret != SUCCESS) {
			kfree(filebuffer);
			return -EINVAL;
		}
		ret = fwdl_load_char(*(filebuffer+i+2), &num_l, i+2);
		if (ret != SUCCESS) {
			kfree(filebuffer);
			return -EINVAL;
		}
		len_line = num_h * 16 + num_l;
		if (len_line > 0x10) {
			PERR("file format fail: @%d, length=%d", i, len_line);
			kfree(filebuffer);
			return -EINVAL;
		}

		/* all data in record and check crc */
		buf_line[0] = len_line;
		for (j = 1; j < len_line + 5; j++) {
			ret = fwdl_load_char(*(filebuffer+i+1+2*j),
				&num_h, i+1+2*j);
			if (ret != SUCCESS) {
				kfree(filebuffer);
				return -EINVAL;
			}
			ret = fwdl_load_char(*(filebuffer+i+2+2*j),
				&num_l, i+2+2*j);
			if (ret != SUCCESS) {
				kfree(filebuffer);
				return -EINVAL;
			}
			buf_line[j] = num_h * 16 + num_l;
		}
		ret = fwdl_checkcrc_8bits(buf_line, len_line + 5);
		if (ret != 0) {
			PERR("file format fail: crc @ 0x%x", i);
			kfree(filebuffer);
			return -EINVAL;
		}

		/* analysis and fill */
		if (buf_line[3] == 0) { /* data frame */
			address = page_address + buf_line[1] * 0x100
				+ buf_line[2];
			if ((mode == FILE_TYPE_APPCODE)
				&& (address < BOOTLOADER_ADDRESS_SHIFT)) {
				PERR("file format fail, mode= %d, @ 0x%x",
					mode, i);
				kfree(filebuffer);
				return -EINVAL;
			} else if ((mode == FILE_TYPE_BOOTLOADER)
				&& (address >= BOOTLOADER_ADDRESS_SHIFT)) {
				PERR("file format fail, mode= %d, @ 0x%x",
					mode, i);
				kfree(filebuffer);
				return -EINVAL;
			}
			memcpy(buffer + address, buf_line + 4, len_line);
			*binsize = address + len_line;
		} else if (buf_line[3] == 1) { /* file end flag */
			break;
		} else if (buf_line[3] == 2) {
			address = buf_line[4] * 0x100 + buf_line[5];
			if (address == 0x1000)
				page_address = 0x010000;
			else if (address == 0x2000)
				page_address = 0x020000;
			else if (address == 0x3000)
				page_address = 0x030000;
			else {
				PERR("file format fail: idx 2 @%d, data=0x%x",
					i, address);
				kfree(filebuffer);
				return -EINVAL;
			}
		} else if (buf_line[3] == 3) {
			/* do nothing to skip it,
				exist in file for d20 device */
			PDEBUG("file format warning: @%d, "
				"data %x %x %x %x %x %x %x",
				i, buf_line[0], buf_line[1], buf_line[2],
				buf_line[3], buf_line[4], buf_line[5],
				buf_line[6]);
		} else if (buf_line[3] == 4) {
			address = buf_line[4] * 0x100 + buf_line[5];
			if (address == 0x40)
				page_address = 0x000000;
			else if (address == 0x41)
				page_address = 0x010000;
			else if (address == 0x42)
				page_address = 0x020000;
			else if (address == 0x43)
				page_address = 0x030000;
			else {
				PERR("file format fail: idx 4 @%d, data=0x%x",
					i, address);
				kfree(filebuffer);
				return -EINVAL;
			}
		} else if (buf_line[3] == 5) {
			/* do nothing to skip it,
				exist in file for g53 device */
			PDEBUG("file format warning: @%d "
				"data %x %x %x %x %x %x %x",
				i, buf_line[0], buf_line[1], buf_line[2],
				buf_line[3], buf_line[4], buf_line[5],
				buf_line[6]);
		} else {
			PERR("file format fail: @%d "
				"data %x %x %x %x %x",
				i, buf_line[0], buf_line[1], buf_line[2],
				buf_line[3], buf_line[4]);
			kfree(filebuffer);
			return -EINVAL;
		}

		/* Record format: */
		/* : ll aaaa tt dd..dd cc xx(0/1/2bits) */
	#if 1
		i += (len_line*2 + 12);
		if ((*(filebuffer+i) == '\r') || (*(filebuffer+i) == '\n'))
			i++;
		if ((*(filebuffer+i) == '\r') || (*(filebuffer+i) == '\n'))
			i++;
	#else
		i += (len_line*2 + 12);
		if (*(filebuffer+i) == ':')
			continue;
		else {
			i++;
			if (*(filebuffer+i) == ':')
				continue;
			else {
				kfree(filebuffer);
				return -EINVAL;
			}
		}
	#endif
	}

/* no need to handle magic code due to bootloade will handle it */
#if 0
	/* magic code*/
	if (mode == FILE_TYPE_BOOTLOADER) {
		*(buffer + MAGIC_WORD_ADDRESS) = 0x55;
		*(buffer + MAGIC_WORD_ADDRESS + 1) = 0xaa;
		*(buffer + MAGIC_WORD_ADDRESS + 2) = 0x55;
		*(buffer + MAGIC_WORD_ADDRESS + 3) = 0xaa;
	} else {
		*binsize -= 0x100;
		*(buffer + MAGIC_WORD_ADDRESS) = 0xaa;
		*(buffer + MAGIC_WORD_ADDRESS + 1) = 0x55;
		*(buffer + MAGIC_WORD_ADDRESS + 2) = 0xff;
		*(buffer + MAGIC_WORD_ADDRESS + 3) = 0x55;
	}
#endif
#if 0
	/* leave the last sector unwrite and let bootloader handle it */
	if (mode != FILE_TYPE_BOOTLOADER)
		*binsize -= BGSH_FWDL_SECTOR_RESERVED;
#endif

	PINFO("HEX file OK, binsize: 0x%x", *binsize);
	PDEBUG_FWDL("FW bin file: %x %x %x %x %x %x %x %x %x %x %x %x "
		"%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x "
		"%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x",
		*buffer, *(buffer+1), *(buffer+2), *(buffer+3),
		*(buffer+4), *(buffer+5), *(buffer+6), *(buffer+7),
		*(buffer+8), *(buffer+9), *(buffer+10), *(buffer+11),
		*(buffer+12), *(buffer+13), *(buffer+14), *(buffer+15),
		*(buffer+16), *(buffer+17), *(buffer+18), *(buffer+19),
		*(buffer+20), *(buffer+21), *(buffer+22), *(buffer+23),
		*(buffer+24), *(buffer+25), *(buffer+26), *(buffer+27),
		*(buffer+28), *(buffer+29), *(buffer+30), *(buffer+31),
		*(buffer+32), *(buffer+33), *(buffer+34), *(buffer+35),
		*(buffer+36), *(buffer+37), *(buffer+38), *(buffer+39),
		*(buffer+40), *(buffer+41), *(buffer+42), *(buffer+43),
		*(buffer+44), *(buffer+45), *(buffer+46), *(buffer+47)
		);

#if FW_DOWNLOAD_BIN_SAVE == 1
	fd = filp_open(BGSH_FW_SAVED_NAME, O_CREAT|O_RDWR, 0);
	if (IS_ERR(fd)) {
		PERR("open file for bin saving fail");
		kfree(filebuffer);
		return -EIO;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	filesize = vfs_write(fd, buffer, BGSH_FW_BIN_SIZE, &pos);
	set_fs(old_fs);
	filp_close(fd, NULL);
	if (filesize <= 0) {
		PERR("write file for bin saving fail");
		kfree(filebuffer);
		return -EIO;
	} else {
		PINFO("write file for bin saving, OK");
	}
#endif

	kfree(filebuffer);
	return SUCCESS;
}

static int32_t fwdl_sendpkg(struct bgsh_client_data *client_data,
	char *buffer, uint16_t length)
{
	BGSH_RETURN_FUNCTION_TYPE comres = SUCCESS;
	if (&(client_data->device) == BGSH_Zero_U8X) {
		return E_NULL_PTR;
	} else {
		comres = client_data->device.BGSH_BUS_WRITE_NOREG_FUNC
			(client_data->device.dev_addr, buffer, length);
	}
	return comres;
}

static int32_t fwdl_getackpkg(struct bgsh_client_data *client_data,
	char *buffer, uint16_t length)
{
	BGSH_RETURN_FUNCTION_TYPE comres = SUCCESS;
	if (&(client_data->device) == BGSH_Zero_U8X) {
		return E_NULL_PTR;
	} else {
		comres = client_data->device.BGSH_BUS_READ_NOREG_FUNC
			(client_data->device.dev_addr, buffer, length);
	}
	return comres;
}

static void fwdl_buildcrc_16bits(char *buffer, uint32_t length,
	uint8_t mode)
{
	uint32_t byteArrayIndex = 0;
	uint16_t data;
	uint16_t crc_register = 0xffff;
	uint16_t poly;
	uint8_t index = 0;

	if (mode == true)
		poly = REVPOLY;
	else
		poly = POLY;

	while (byteArrayIndex < length) {
		data = (uint16_t)(buffer[byteArrayIndex] & 0x00FF);
		data <<= 8;

		for (index = 0; index < 8; index++) {
			if (((crc_register ^ data) & 0x8000) != 0)
				crc_register =
					(uint16_t)((crc_register << 1) ^ poly);
			else
				crc_register <<= 1;
			data <<= 1;
		}
		byteArrayIndex++;
	}
	/* Initialized with 0xFFFF, So need to be XORed with 0xFFFF.
	   For XORing with 0xFFFF, it is bitwise NOTed*/
	crc_register = ~crc_register;

	buffer[length] = (uint8_t)((~crc_register&0xff00)>>8);
	buffer[length+1] = (uint8_t)(~crc_register&0x00ff);

#if 0
	PDEBUG_FWDL("Len:%d, pkg:%x %x %x %x %x %x %x %x "
		"%x %x %x %x %x %x %x %x %x %x", length,
		buffer[0], buffer[1], buffer[2], buffer[3],
		buffer[4], buffer[5], buffer[6], buffer[7],
		buffer[8], buffer[9], buffer[10], buffer[11],
		buffer[12], buffer[13], buffer[14], buffer[15],
		buffer[16], buffer[17]);
#endif
}

static ssize_t bgsh_store_fwdl_appcode(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	int32_t ret = SUCCESS;
	char *buffer = NULL;
	int32_t binsize = 0;
	char cmdpkg[BGSH_FWDL_BUFFER_LENGTH];
	char ackpkg[2];
	uint8_t pkglen;
	uint32_t i, pos;
	uint32_t total_write_length;

	PINFO("Application download start");
	buffer = kmalloc(BGSH_FW_BIN_SIZE * sizeof(char), GFP_KERNEL);
	if (buffer == NULL) {
		PERR("malloc mem fail, buffer");
		return -EIO;
	}

	/* To flash all kinds fw file*/
	ret = fwdl_checkfile(BGSH_FW_FILE_NAME, buffer, &binsize,
		FILE_TYPE_ALL);  /* FILE_TYPE_APPCODE if necessary */
	if (ret != SUCCESS) {
		PERR("check file fail, ret=%d", ret);
		kfree(buffer);
		return -EIO;
	}

	mutex_lock(&client_data->mutex_bus_op);
	ret = bgsh_reset(client_data, RESET_TO_BOOTLOADER_MODE);
	if (ret != SUCCESS) {
		mutex_unlock(&client_data->mutex_bus_op);
		kfree(buffer);
		return -EIO;
	}

	/* need check */
	mdelay(1000);

	/* send length to BGSH */
	memset(cmdpkg, 0, sizeof(cmdpkg));
	PDEBUG_FWDL("bgsh send fw file length = 0x%x",
		binsize - BOOTLOADER_ADDRESS_SHIFT);
	cmdpkg[0] = (uint8_t)(
		(binsize - BOOTLOADER_ADDRESS_SHIFT)&0x000000ff);
	cmdpkg[1] = (uint8_t)(
		((binsize - BOOTLOADER_ADDRESS_SHIFT)&0x0000ff00)>>8);
	cmdpkg[2] = (uint8_t)(
		((binsize - BOOTLOADER_ADDRESS_SHIFT)&0x00ff0000)>>16);
	cmdpkg[3] = (uint8_t)(
		((binsize - BOOTLOADER_ADDRESS_SHIFT)&0xff000000)>>24);
	fwdl_buildcrc_16bits(cmdpkg, 4, false);

	for (i = 0; i < BGSH_FWDL_RETRY_TIMES; i++) {
		ret = fwdl_sendpkg(client_data, cmdpkg, 6);
		if (ret == SUCCESS)
			break;
		PDEBUG_FWDL("bgsh send length, retry");
	}
	/* check result */
	memset(ackpkg, 0, sizeof(ackpkg));
	ret = fwdl_getackpkg(client_data, ackpkg, 1);
	if (ret != SUCCESS) {
		PERR("get ack fail, ret=%d", ret);
		mutex_unlock(&client_data->mutex_bus_op);
		kfree(buffer);
		return -EIO;
	}
	PDEBUG_FWDL("bgsh send length, get ack OK: %d, %c",
		ackpkg[0], ackpkg[0]);
	if (ackpkg[0] != 's') {
		/* ack package, status is not success */
		PERR("ack package not ok 1, %d", ackpkg[0]);
		mutex_unlock(&client_data->mutex_bus_op);
		kfree(buffer);
		return -EIO;
	}

	/* send package length to BGSH */
	PDEBUG_FWDL("bgsh send package length = 0x%x",
		BGSH_FWDL_PACKAGE_LENGTH);
	memset(cmdpkg, 0, sizeof(cmdpkg));
	cmdpkg[0] = BGSH_FWDL_PACKAGE_LENGTH;
	fwdl_buildcrc_16bits(cmdpkg, 1, false);
	for (i = 0; i < BGSH_FWDL_RETRY_TIMES; i++) {
		ret = fwdl_sendpkg(client_data, cmdpkg, 3);
		if (ret == SUCCESS)
			break;
	}
	/* check result */
	memset(ackpkg, 0, sizeof(ackpkg));
	ret = fwdl_getackpkg(client_data, ackpkg, 1);
	if (ret != SUCCESS) {
		PERR("get ack fail, ret=%d", ret);
		mutex_unlock(&client_data->mutex_bus_op);
		kfree(buffer);
		return -EIO;
	}
	/* ack package, status is not success */
	if (ackpkg[0] != 's') {
		PERR("ack package not ok 2, %d", ackpkg[0]);
		mutex_unlock(&client_data->mutex_bus_op);
		kfree(buffer);
		return -EIO;
	}

	/* app code start from this shift */
	PDEBUG_FWDL("bgsh send fw file data");
	pos = BOOTLOADER_ADDRESS_SHIFT;
	total_write_length = 0;
	while (1) {
		/* send data to BGSH */
		pkglen = (binsize - pos > BGSH_FWDL_PACKAGE_LENGTH)
			? BGSH_FWDL_PACKAGE_LENGTH : (binsize - pos);
		memcpy(cmdpkg, buffer+pos, pkglen);
		fwdl_buildcrc_16bits(cmdpkg, pkglen, false);
		for (i = 0; i < BGSH_FWDL_RETRY_TIMES; i++) {
			ret = fwdl_sendpkg(client_data, cmdpkg, pkglen + 2);
			if (ret == SUCCESS)
				break;
		}
		total_write_length += pkglen;

		/* check result */
		ret = fwdl_getackpkg(client_data, ackpkg, 1);
		if (ret != SUCCESS) {
			PERR("get ack fail @ %x, ret=%d", pos, ret);
			mutex_unlock(&client_data->mutex_bus_op);
			kfree(buffer);
			return -EIO;
		}
		/* ack package, status is not success */
		if (ackpkg[0] != 's') {
			PERR("ack package not ok @ %x, %d", pos, ackpkg[0]);
			mutex_unlock(&client_data->mutex_bus_op);
			kfree(buffer);
			return -EIO;
		}

		pos += pkglen;
		if (pos >= binsize)
			break;
	}
	kfree(buffer);
	PINFO("Application download finish");
	PINFO("Write length = 0x%x", total_write_length);
	/* wait one second to let bgsh start after download */
	mdelay(1000);

	mutex_unlock(&client_data->mutex_bus_op);
	return count;
}

static ssize_t bgsh_store_fwdl_bootloader(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct bgsh_client_data *client_data = input_get_drvdata(input);
	int32_t ret = SUCCESS;
	char *buffer = NULL;
	int32_t binsize = 0;
	char cmdpkg[BGSH_FWDL_PACKAGE_LENGTH + 4];
	char ackpkg[2];
	uint8_t pkglen;
	uint32_t i, pos;

	PINFO("Bootloader download start");
	buffer = kmalloc(BGSH_FW_BIN_SIZE * sizeof(char), GFP_KERNEL);
	if (buffer == NULL) {
		PERR("malloc mem fail, buffer");
		return -EIO;
	}

	ret = fwdl_checkfile(BGSH_BL_FILE_NAME, buffer, &binsize,
		FILE_TYPE_BOOTLOADER);
	if (ret != SUCCESS) {
		PERR("check file fail, ret=%d", ret);
		kfree(buffer);
		return -EIO;
	}

	mutex_lock(&client_data->mutex_bus_op);
	ret = bgsh_reset(client_data, RESET_TO_BOOTLOADER_MODE);
	if (ret != SUCCESS) {
		mutex_unlock(&client_data->mutex_bus_op);
		kfree(buffer);
		return -EIO;
	}

	/* need check */
	mdelay(1000);

	/* send length to BGSH */
	PDEBUG_FWDL("bgsh send bootloader file length = 0x%x",
		binsize);
	cmdpkg[0] = (uint8_t)(binsize&0x000000ff);
	cmdpkg[1] = (uint8_t)((binsize&0x0000ff00)>>8);
	cmdpkg[2] = (uint8_t)((binsize&0x00ff0000)>>16);
	cmdpkg[3] = (uint8_t)((binsize&0xff000000)>>24);
	fwdl_buildcrc_16bits(cmdpkg, 4, false);
	for (i = 0; i < BGSH_FWDL_RETRY_TIMES; i++) {
		ret = fwdl_sendpkg(client_data, cmdpkg, 6);
		if (ret == SUCCESS)
			break;
	}
	/* check result */
	ret = fwdl_getackpkg(client_data, ackpkg, 1);
	if (ret != SUCCESS) {
		PERR("get ack fail, ret=%d", ret);
		mutex_unlock(&client_data->mutex_bus_op);
		kfree(buffer);
		return -EIO;
	}
	/* ack package, status is not success */
	if (ackpkg[0] != 's') {
		PERR("ack package not ok 1, %d", ackpkg[0]);
		mutex_unlock(&client_data->mutex_bus_op);
		kfree(buffer);
		return -EIO;
	}

	/* send package length to BGSH */
	PDEBUG_FWDL("bgsh send package length = 0x%x",
		BGSH_FWDL_PACKAGE_LENGTH);
	cmdpkg[0] = BGSH_FWDL_PACKAGE_LENGTH;
	fwdl_buildcrc_16bits(cmdpkg, 1, false);
	for (i = 0; i < BGSH_FWDL_RETRY_TIMES; i++) {
		ret = fwdl_sendpkg(client_data, cmdpkg, 3);
		if (ret == SUCCESS)
			break;
	}
	/* check result */
	ret = fwdl_getackpkg(client_data, ackpkg, 1);
	if (ret != SUCCESS) {
		PERR("get ack fail, ret=%d", ret);
		mutex_unlock(&client_data->mutex_bus_op);
		kfree(buffer);
		return -EIO;
	}
	/* ack package, status is not success */
	if (ackpkg[0] != 's') {
		PERR("ack package not ok 2, %d", ackpkg[0]);
		mutex_unlock(&client_data->mutex_bus_op);
		kfree(buffer);
		return -EIO;
	}

	/* send command upgrade bootloader to BGSH */
	PDEBUG_FWDL("bgsh send bootloader update command");
	cmdpkg[0] = COMMAND_START_BYTE;
	cmdpkg[1] = 1;
	cmdpkg[2] = UPGRADE_BOOTLOADER;
	cmdpkg[3] = COMMAND_END_BYTE;
	fwdl_buildcrc_16bits(cmdpkg, 4, true);
	for (i = 0; i < BGSH_FWDL_RETRY_TIMES; i++) {
		ret = fwdl_sendpkg(client_data, cmdpkg, 6);
		if (ret == SUCCESS)
			break;
	}
	/* check result */
	ret = fwdl_getackpkg(client_data, ackpkg, 1);
	if (ret != SUCCESS) {
		PERR("get ack fail, ret=%d", ret);
		mutex_unlock(&client_data->mutex_bus_op);
		kfree(buffer);
		return -EIO;
	}
	/* ack package, status is not success */
	if (ackpkg[0] != 's') {
		PERR("ack package not ok 3, %d", ackpkg[0]);
		mutex_unlock(&client_data->mutex_bus_op);
		kfree(buffer);
		return -EIO;
	}

	/* send data to BGSH */
	PDEBUG_FWDL("bgsh send bootloader data");
	pos = 0;
	while (1) {
		pkglen = (binsize - pos > BGSH_FWDL_PACKAGE_LENGTH)
			? BGSH_FWDL_PACKAGE_LENGTH : (binsize - pos);
		memcpy(cmdpkg, buffer+pos, pkglen);
		fwdl_buildcrc_16bits(cmdpkg, pkglen, false);
		for (i = 0; i < BGSH_FWDL_RETRY_TIMES; i++) {
			ret = fwdl_sendpkg(client_data, cmdpkg, pkglen + 2);
			if (ret == SUCCESS)
				break;
		}

		/* check result */
		ret = fwdl_getackpkg(client_data, ackpkg, 1);
		if (ret != SUCCESS) {
			PERR("get ack fail, ret=%d", ret);
			mutex_unlock(&client_data->mutex_bus_op);
			kfree(buffer);
			return -EIO;
		}
		/* ack package, status is not success */
		if (ackpkg[0] != 's') {
			PERR("ack package not ok @ %d, %d", pos, ackpkg[0]);
			mutex_unlock(&client_data->mutex_bus_op);
			kfree(buffer);
			return -EIO;
		}

		pos += pkglen;
		if (pos >= binsize)
			break;
	}
	kfree(buffer);
	PINFO("Bootloader download finish");
	/* wait one second to let bgsh start after download */
	mdelay(1000);

	mutex_unlock(&client_data->mutex_bus_op);
	return count;
}

static ssize_t bgsh_store_fwdl_checkfile(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int32_t ret = SUCCESS;
	char *buffer = NULL;
	int32_t binsize = 0;
	unsigned long data;

	ret = kstrtoul(buf, 16, &data);
	if (ret) {
		PERR("xfer str fail");
		return ret;
	}
	if (data >= FILE_TYPE_END) {
		PERR("value error");
		return -EINVAL;
	}

	buffer = kmalloc(BGSH_FW_BIN_SIZE * sizeof(char), GFP_KERNEL);
	if (buffer == NULL) {
		PERR("malloc mem fail");
		return -EIO;
	}

	PINFO("Check file start, mode = %ld", data);
	if (data == FILE_TYPE_BOOTLOADER)
		ret = fwdl_checkfile(BGSH_BL_FILE_NAME, buffer,
			&binsize, (uint8_t)data);
	else
		ret = fwdl_checkfile(BGSH_FW_FILE_NAME, buffer,
			&binsize, (uint8_t)data);
	if (ret != SUCCESS) {
		PERR("check file fail, ret=%d", ret);
		kfree(buffer);
		return -EIO;
	}

	kfree(buffer);
	return count;
}
#endif

static ssize_t bgsh_show_debug_log_level(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", get_debug_log_level());
}

#ifdef BOSCH_DRIVER_LOG_FUNC
static ssize_t bgsh_store_debug_log_level(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int32_t ret = SUCCESS;
	unsigned long data;
	uint8_t level_backup;

	/* make log valid */
	level_backup = get_debug_log_level();
	set_debug_log_level(LOG_LEVEL_D);

	ret = kstrtoul(buf, 16, &data);
	if (ret) {
		PERR("xfer str fail");
		set_debug_log_level(level_backup);
		return ret;
	}
	if (data > LOG_LEVEL_A) {
		PERR("value error");
		set_debug_log_level(level_backup);
		return -EINVAL;
	}
	set_debug_log_level((uint8_t)data);

	return count;
}
#endif

static DEVICE_ATTR(chip_id, S_IRUGO,
		bgsh_show_chip_id, NULL);
static DEVICE_ATTR(chip_ver, S_IRUGO,
		bgsh_show_chip_ver, NULL);
static DEVICE_ATTR(op_mode, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_op_mode, bgsh_store_op_mode);
static DEVICE_ATTR(selftest, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_selftest, bgsh_store_selftest);
static DEVICE_ATTR(pwr_mode, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_pwr_mode, bgsh_store_pwr_mode);
static DEVICE_ATTR(odr, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_odr, bgsh_store_odr);
static DEVICE_ATTR(acc_value, S_IRUGO,
		bgsh_show_acc_value, NULL);
static DEVICE_ATTR(mag_value, S_IRUGO,
		bgsh_show_mag_value, NULL);
static DEVICE_ATTR(gyr_value, S_IRUGO,
		bgsh_show_gyro_value, NULL);
static DEVICE_ATTR(ori_value, S_IRUGO,
		bgsh_show_euler_value, NULL);
static DEVICE_ATTR(la_value, S_IRUGO,
		bgsh_show_linear_accel_value, NULL);
static DEVICE_ATTR(gr_value, S_IRUGO,
		bgsh_show_gravity_value, NULL);
static DEVICE_ATTR(rv_value, S_IRUGO,
		bgsh_show_quat_value, NULL);
static DEVICE_ATTR(temperature, S_IRUGO,
		bgsh_show_temperature, NULL);
static DEVICE_ATTR(channels, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_channels, bgsh_store_channels);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_enable, bgsh_store_enable);
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_delay, bgsh_store_delay);
static DEVICE_ATTR(place, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_place, bgsh_store_place);
static DEVICE_ATTR(acc_unit_sel, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_data_format_acc, bgsh_store_data_format_acc);
static DEVICE_ATTR(gyr_unit_sel, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_data_format_gyro, bgsh_store_data_format_gyro);
static DEVICE_ATTR(ori_unit_sel, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_data_format_euler, bgsh_store_data_format_euler);
static DEVICE_ATTR(temp_unit_sel, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_data_format_temperature,
		bgsh_store_data_format_temperature);
static DEVICE_ATTR(acc_mode, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_acc_mode, bgsh_store_acc_mode);
static DEVICE_ATTR(acc_range, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_acc_grange, bgsh_store_acc_grange);
static DEVICE_ATTR(acc_bw, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_acc_bandwidth, bgsh_store_acc_bandwidth);
static DEVICE_ATTR(gyr_mode, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_gyro_mode, bgsh_store_gyro_mode);
static DEVICE_ATTR(gyr_range, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_gyro_range, bgsh_store_gyro_range);
static DEVICE_ATTR(gyr_bw, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_gyro_bandwidth, bgsh_store_gyro_bandwidth);
static DEVICE_ATTR(mag_mode, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_mag_mode, bgsh_store_mag_mode);
static DEVICE_ATTR(mag_dr, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_mag_dr, bgsh_store_mag_dr);
static DEVICE_ATTR(acc_enable, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_acc_enable, bgsh_store_acc_enable);
static DEVICE_ATTR(gyr_enable, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_gyro_enable, bgsh_store_gyro_enable);
static DEVICE_ATTR(mag_enable, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_mag_enable, bgsh_store_mag_enable);
static DEVICE_ATTR(ori_enable, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_euler_enable, bgsh_store_euler_enable);
static DEVICE_ATTR(rv_enable, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_quaternion_enable, bgsh_store_quaternion_enable);
static DEVICE_ATTR(la_enable, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_linear_accel_enable,
		bgsh_store_linear_accel_enable);
static DEVICE_ATTR(gr_enable, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_gravity_enable, bgsh_store_gravity_enable);
static DEVICE_ATTR(temp_enable, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_temperature_enable,
		bgsh_store_temperature_enable);
static DEVICE_ATTR(acc_offset_x, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_acc_offset_x, bgsh_store_acc_offset_x);
static DEVICE_ATTR(acc_offset_y, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_acc_offset_y, bgsh_store_acc_offset_y);
static DEVICE_ATTR(acc_offset_z, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_acc_offset_z, bgsh_store_acc_offset_z);
static DEVICE_ATTR(gyr_offset_x, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_gyro_offset_x, bgsh_store_gyro_offset_x);
static DEVICE_ATTR(gyr_offset_y, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_gyro_offset_y, bgsh_store_gyro_offset_y);
static DEVICE_ATTR(gyr_offset_z, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_gyro_offset_z, bgsh_store_gyro_offset_z);
static DEVICE_ATTR(mag_offset_x, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_mag_offset_x, bgsh_store_mag_offset_x);
static DEVICE_ATTR(mag_offset_y, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_mag_offset_y, bgsh_store_mag_offset_y);
static DEVICE_ATTR(mag_offset_z, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_mag_offset_z, bgsh_store_mag_offset_z);

#if BGSH_SIC_MATRIX_WRITE_IN_ONE_COMMAND == 1
static DEVICE_ATTR(sic_matrix, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_sic_matrix, bgsh_store_sic_matrix);
#endif
static DEVICE_ATTR(sic_matrix_m0, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_sic_matrix, bgsh_store_sic_matrix_zero);
static DEVICE_ATTR(sic_matrix_m1, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_sic_matrix, bgsh_store_sic_matrix_one);
static DEVICE_ATTR(sic_matrix_m2, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_sic_matrix, bgsh_store_sic_matrix_two);
static DEVICE_ATTR(sic_matrix_m3, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_sic_matrix, bgsh_store_sic_matrix_three);
static DEVICE_ATTR(sic_matrix_m4, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_sic_matrix, bgsh_store_sic_matrix_four);
static DEVICE_ATTR(sic_matrix_m5, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_sic_matrix, bgsh_store_sic_matrix_five);
static DEVICE_ATTR(sic_matrix_m6, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_sic_matrix, bgsh_store_sic_matrix_six);
static DEVICE_ATTR(sic_matrix_m7, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_sic_matrix, bgsh_store_sic_matrix_seven);
static DEVICE_ATTR(sic_matrix_m8, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_sic_matrix, bgsh_store_sic_matrix_eight);

#ifdef BGSH_DEBUG
static DEVICE_ATTR(register, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_register, bgsh_store_register);
static DEVICE_ATTR(reg_op_pageid, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_reg_op_pageid, bgsh_store_reg_op_pageid);
static DEVICE_ATTR(reg_op_addr, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_reg_op_addr, bgsh_store_reg_op_addr);
static DEVICE_ATTR(reg_op_value, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_reg_op_value, bgsh_store_reg_op_value);
#endif
#if FW_DOWNLOAD_SUPPORT == 1
static DEVICE_ATTR(fwdl_bootloader, S_IWUSR|S_IWGRP,
		NULL, bgsh_store_fwdl_bootloader);
static DEVICE_ATTR(fwdl_appcode, S_IWUSR|S_IWGRP,
		NULL, bgsh_store_fwdl_appcode);
static DEVICE_ATTR(fwdl_checkfile, S_IWUSR|S_IWGRP,
		NULL, bgsh_store_fwdl_checkfile);
#endif
#ifdef BOSCH_DRIVER_LOG_FUNC
static DEVICE_ATTR(debug_log_level, S_IRUGO|S_IWUSR|S_IWGRP,
		bgsh_show_debug_log_level, bgsh_store_debug_log_level);
#else
static DEVICE_ATTR(debug_log_level, S_IRUGO,
		bgsh_show_debug_log_level, NULL);
#endif

static struct attribute *bgsh_attributes[] = {
	&dev_attr_chip_id.attr,
	&dev_attr_chip_ver.attr,
	&dev_attr_temperature.attr,
	&dev_attr_op_mode.attr,
	&dev_attr_selftest.attr,
	&dev_attr_pwr_mode.attr,
	&dev_attr_odr.attr,
	&dev_attr_acc_value.attr,
	&dev_attr_mag_value.attr,
	&dev_attr_gyr_value.attr,
	&dev_attr_ori_value.attr,
	&dev_attr_la_value.attr,
	&dev_attr_gr_value.attr,
	&dev_attr_rv_value.attr,
	&dev_attr_channels.attr,
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
	&dev_attr_place.attr,
	&dev_attr_acc_unit_sel.attr,
	&dev_attr_gyr_unit_sel.attr,
	&dev_attr_ori_unit_sel.attr,
	&dev_attr_temp_unit_sel.attr,
	&dev_attr_acc_mode.attr,
	&dev_attr_acc_range.attr,
	&dev_attr_acc_bw.attr,
	&dev_attr_gyr_mode.attr,
	&dev_attr_gyr_range.attr,
	&dev_attr_gyr_bw.attr,
	&dev_attr_mag_mode.attr,
	&dev_attr_mag_dr.attr,
	&dev_attr_acc_enable.attr,
	&dev_attr_gyr_enable.attr,
	&dev_attr_mag_enable.attr,
	&dev_attr_ori_enable.attr,
	&dev_attr_rv_enable.attr,
	&dev_attr_la_enable.attr,
	&dev_attr_gr_enable.attr,
	&dev_attr_temp_enable.attr,
	&dev_attr_acc_offset_x.attr,
	&dev_attr_acc_offset_y.attr,
	&dev_attr_acc_offset_z.attr,
	&dev_attr_gyr_offset_x.attr,
	&dev_attr_gyr_offset_y.attr,
	&dev_attr_gyr_offset_z.attr,
	&dev_attr_mag_offset_x.attr,
	&dev_attr_mag_offset_y.attr,
	&dev_attr_mag_offset_z.attr,
#if BGSH_SIC_MATRIX_WRITE_IN_ONE_COMMAND == 1
	&dev_attr_sic_matrix.attr,
#endif
	&dev_attr_sic_matrix_m0.attr,
	&dev_attr_sic_matrix_m1.attr,
	&dev_attr_sic_matrix_m2.attr,
	&dev_attr_sic_matrix_m3.attr,
	&dev_attr_sic_matrix_m4.attr,
	&dev_attr_sic_matrix_m5.attr,
	&dev_attr_sic_matrix_m6.attr,
	&dev_attr_sic_matrix_m7.attr,
	&dev_attr_sic_matrix_m8.attr,
#ifdef BGSH_DEBUG
	&dev_attr_register.attr,
	&dev_attr_reg_op_pageid.attr,
	&dev_attr_reg_op_addr.attr,
	&dev_attr_reg_op_value.attr,
#endif
#if FW_DOWNLOAD_SUPPORT == 1
	&dev_attr_fwdl_bootloader.attr,
	&dev_attr_fwdl_appcode.attr,
	&dev_attr_fwdl_checkfile.attr,
#endif
	&dev_attr_debug_log_level.attr,

	NULL
};

static struct attribute_group bgsh_attribute_group = {
	.attrs = bgsh_attributes
};

/* input device, need check */
static int32_t bgsh_input_init(struct bgsh_client_data *client_data)
{
	struct input_dev *dev;
	int32_t ret = SUCCESS;

	dev = input_allocate_device();
	if (NULL == dev) {
		PERR("Allocate input device fail");
		return -ENOMEM;
	}

	dev->name = SENSOR_NAME;
	dev->id.bustype = BUS_I2C; /* I2C? */

	input_set_capability(dev, EV_ABS, ABS_MISC);
	input_set_abs_params(dev, ABS_X, MAG_VALUE_MIN, MAG_VALUE_MAX, 0, 0);
	input_set_abs_params(dev, ABS_Y, MAG_VALUE_MIN, MAG_VALUE_MAX, 0, 0);
	input_set_abs_params(dev, ABS_Z, MAG_VALUE_MIN, MAG_VALUE_MAX, 0, 0);
	input_set_drvdata(dev, client_data);

	ret = input_register_device(dev);
	if (ret < 0) {
		input_free_device(dev);
		PERR("Register input device fail");
		return ret;
	}
	client_data->input = dev;

	return 0;
}


static void bgsh_input_destroy(struct bgsh_client_data *client_data)
{
	struct input_dev *dev = client_data->input;

	input_unregister_device(dev);
	input_free_device(dev);
}

static int32_t bgsh_check_chip_id(struct bgsh_data_bus *data_bus)
{
	int32_t ret = SUCCESS;
	uint8_t chip_id = 0;
	uint8_t pg_id = 0;

	ret = data_bus->bops->bus_write(BGSH_BUS_ADDR,
				BGSH_REG_NAME(PAGE_ID), (uint8_t *)&pg_id, 1);
	if (ret < 0)
		return ret;

	ret = data_bus->bops->bus_read(BGSH_BUS_ADDR,
				BGSH_REG_NAME(CHIP_ID), (uint8_t *)&chip_id, 1);
	if (ret < 0)
		return ret;

	PINFO("read chip id result: %x", chip_id);
	if ((chip_id & 0xff) != SENSOR_CHIP_ID_BGSH)
		ret = -1;
	else
		ret = SUCCESS;

	return ret;
}

static int32_t bgsh_wakeup(struct bgsh_data_bus *data_bus)
{
	int32_t ret = SUCCESS;
	int32_t try_times = BGSH_MAX_RETRY_WAKEUP;
	const uint8_t pg_id = 0x1;
	uint8_t dummy;

	PINFO("waking up bgsh chip...");

	mdelay(BGSH_BUS_WRITE_DELAY_TIME);
	while (try_times) {
		data_bus->bops->bus_write(BGSH_BUS_ADDR,
				BGSH_REG_NAME(PAGE_ID), (uint8_t *)&pg_id, 1);
		mdelay(BGSH_BUS_WRITE_DELAY_TIME);

		dummy = 0;
		ret = data_bus->bops->bus_read(BGSH_BUS_ADDR,
			BGSH_REG_NAME(PAGE_ID), &dummy, 1);
		if (pg_id == dummy)
			break;

		try_times--;
	}

	PINFO("wake up result: %s, tried times: %d",
			(try_times > 0) ? "succeed" : "fail",
			BGSH_MAX_RETRY_WAKEUP - try_times + 1);

	ret = (try_times > 0) ? 0 : -1;

	return ret;
}

/*!
 * @brief initialize bgsh client
 *
 * @param data the pointer of bgsh client data
 *
 * @return zero success, non-zero failed
 * @retval zero success
 * @retval non-zero failed
*/
static int32_t bgsh_init_client(struct bgsh_client_data *client_data)
{
	int32_t ret = SUCCESS;
	struct reg_node node;
	uint8_t sensor_sel = 0;

	/* h/w init */
	client_data->device.bus_read = client_data->data_bus.bops->bus_read;
	client_data->device.bus_write = client_data->data_bus.bops->bus_write;
#if FW_DOWNLOAD_SUPPORT == 1
	client_data->device.bus_read_noreg =
		client_data->data_bus.bops->bus_read_noreg;
	client_data->device.bus_write_noreg =
		client_data->data_bus.bops->bus_write_noreg;
#endif
	client_data->device.delay_msec = bgsh_delay;

	ret = bgsh_init(&client_data->device);
	if (ret)
		return ret;

	mutex_init(&client_data->mutex_bus_op);
	mutex_init(&client_data->mutex_enable);
	atomic_set(&client_data->delay, BGSH_DELAY_DEFAULT);
	ret = bgsh_write_register(client_data, PAGE_ZERO,
		BGSH_DATA_SEL_ADDR, &sensor_sel, 1);
	if (ret) {
		PERR("fail to init %s, write sensor default", SENSOR_NAME);
		ret = -EIO;
		return ret;
	}

	client_data->enable = 0;
	/* now it's power on which is considered as resuming from suspend */
	client_data->op_mode = OPERATION_MODE_CONFIG;
	client_data->odr = BGSH_DEFAULT_ODR;
	client_data->acc_pw_mode = ACCEL_SUSPEND;
	client_data->gyro_pw_mode = GYRO_OPR_MODE_SUSPEND;
	client_data->mag_pw_mode = MAG_POWER_MODE_SUSPEND;
	if (NULL != client_data->bst_pd)
		client_data->place = (uint8_t)(client_data->bst_pd->place);
	else
		client_data->place = BGSH_DEFAULT_AXIS;

	ret = bgsh_set_op_mode(client_data, OPERATION_MODE_CONFIG);
	ret += bgsh_set_pwr_mode(client_data, SENSOR_POWER_MODE_LOWERPOWER);
	if (ret) {
		PERR("fail to init h/w of %s", SENSOR_NAME);
		ret = -EIO;
		return ret;
	}
	/* set output format to android mode */
	MAKE_REG_NODE(node, BGSH_DATA_OUTPUT_FORMAT);
	bgsh_write_node(client_data, node,
		ORITATION_ANDROID, &ret);
	MAKE_REG_NODE(node, BGSH_ACC_UNIT);
	bgsh_write_node(client_data, node,
		UNIT_ACC_MG, &ret);
	MAKE_REG_NODE(node, BGSH_GYR_UNIT);
	bgsh_write_node(client_data, node,
		UNIT_GYRO_RPS, &ret);
	MAKE_REG_NODE(node, BGSH_EUL_UNIT);
	bgsh_write_node(client_data, node,
		UNIT_EULER_DEG, &ret);
	MAKE_REG_NODE(node, BGSH_TEMP_UNIT);
	bgsh_write_node(client_data, node,
		UNIT_TEMPERATURE_CELSIUS, &ret);
	MAKE_REG_NODE(node, BGSH_OUTPUT_DATA_RATE);
	bgsh_write_node(client_data, node,
		client_data->odr, &ret);
	ret += bgsh_set_sensor_remap(client_data, client_data->place);

	return ret;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void bgsh_early_suspend(struct early_suspend *h)
{
	struct bgsh_client_data *client_data =
		container_of(h, struct bgsh_client_data, early_suspend_handler);

	mutex_lock(&client_data->mutex_enable);
	if (client_data->enable == 1) {
		bgsh_set_op_mode(client_data, OPERATION_MODE_CONFIG);
			cancel_delayed_work_sync(&client_data->work);
	}
	mutex_unlock(&client_data->mutex_enable);
}

static void bgsh_late_resume(struct early_suspend *h)
{
	struct bgsh_client_data *client_data =
		container_of(h, struct bgsh_client_data, early_suspend_handler);

	mutex_lock(&client_data->mutex_enable);
	if (client_data->enable == 1) {
		bgsh_set_op_mode(client_data, OPERATION_MODE_NDOF);
		schedule_delayed_work(&client_data->work,
			msecs_to_jiffies(atomic_read(&client_data->delay)));
	}
	mutex_unlock(&client_data->mutex_enable);
}
#endif

int32_t bgsh_probe(struct device *dev, struct bgsh_data_bus *data_bus)
{
	int32_t ret = SUCCESS;
	struct bgsh_client_data *client_data = NULL;
	int32_t dummy;

	PINFO("bgsh_probe function entrance");

	/* wake up the chip */
	dummy = bgsh_wakeup(data_bus);
	if (dummy < 0) {
		PERR("Cannot wake up %s, bus xfer error", SENSOR_NAME);
		ret = -EIO;
		goto exit_err_clean;
	}

	/* check chip id */
	ret = bgsh_check_chip_id(data_bus);
	if (ret == SUCCESS) {
		PNOTICE("Bosch Sensortec Device %s detected",
				SENSOR_NAME);
	} else {
		PERR("Bosch Sensortec Device not found, chip id mismatch");
		ret = -1;
		goto exit_err_clean;
	}

	client_data = kzalloc(sizeof(struct bgsh_client_data), GFP_KERNEL);
	if (NULL == client_data) {
		PERR("no memory available");
		ret = -ENOMEM;
		goto exit_err_clean;
	}

	dev_set_drvdata(dev, client_data);
	client_data->data_bus = *data_bus;
	client_data->dev = dev;

	if (NULL != dev->platform_data) {
		client_data->bst_pd = kzalloc(sizeof(*client_data->bst_pd),
				GFP_KERNEL);

		if (NULL != client_data->bst_pd) {
			memcpy(client_data->bst_pd, dev->platform_data,
					sizeof(*client_data->bst_pd));

			PINFO("platform data of bgsh %s: place: %d, irq: %d",
					client_data->bst_pd->name,
					client_data->bst_pd->place,
					client_data->bst_pd->irq);
		}
	} else
		PINFO("no platform data");

	ret = bgsh_init_client(client_data);
	if (ret) {
		PERR("fail to initial");
		ret = -EIO;
		goto exit_err_clean;
	}

	/* input device init */
	ret = bgsh_input_init(client_data);
	if (ret < 0)
		goto exit_err_clean;

	/* sysfs node creation */
	ret = sysfs_create_group(&client_data->input->dev.kobj,
			&bgsh_attribute_group);

	if (ret < 0)
		goto exit_err_sysfs;

#ifdef CONFIG_HAS_EARLYSUSPEND
	client_data->early_suspend_handler.level
		= EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	client_data->early_suspend_handler.suspend = bgsh_early_suspend;
	client_data->early_suspend_handler.resume = bgsh_late_resume;
	register_early_suspend(&client_data->early_suspend_handler);
#endif

	/* workqueue init */
	INIT_DELAYED_WORK(&client_data->work, bgsh_work_func);

	bgsh_dump_reg(client_data);

	PNOTICE("sensor %s probed successfully", SENSOR_NAME);

	return 0;

exit_err_sysfs:
	if (ret)
		bgsh_input_destroy(client_data);

exit_err_clean:
	if (ret) {
		if (client_data != NULL) {
			if (NULL != client_data->bst_pd) {
				kfree(client_data->bst_pd);
				client_data->bst_pd = NULL;
			}
			kfree(client_data);
			client_data = NULL;
		}
	}

	return ret;
}
EXPORT_SYMBOL(bgsh_probe);

static int32_t bgsh_pre_suspend(struct device *dev)
{
	int32_t ret = SUCCESS;
	struct bgsh_client_data *client_data =
		dev_get_drvdata(dev);

	PINFO("function entrance");

	mutex_lock(&client_data->mutex_enable);
	if (client_data->enable) {
		cancel_delayed_work_sync(&client_data->work);
		PDEBUG("cancel work");
	}
	mutex_unlock(&client_data->mutex_enable);

	return ret;
}

int32_t bgsh_suspend(struct device *dev)
{
	int32_t ret = SUCCESS;
	struct bgsh_client_data *client_data =
		dev_get_drvdata(dev);

	PINFO("function entrance");

	mutex_lock(&client_data->mutex_bus_op);
	if (client_data->op_mode != OPERATION_MODE_CONFIG) {
		ret = bgsh_pre_suspend(dev);
		ret += bgsh_set_op_mode(client_data, OPERATION_MODE_CONFIG);
	}
	mutex_unlock(&client_data->mutex_bus_op);

	return ret;
}
EXPORT_SYMBOL(bgsh_suspend);

static int32_t bgsh_post_resume(struct bgsh_client_data *client_data)
{
	int32_t ret = SUCCESS;

	PDEBUG("function entrance");

	mutex_lock(&client_data->mutex_enable);
	if (client_data->enable) {
		schedule_delayed_work(&client_data->work,
				msecs_to_jiffies(
					atomic_read(&client_data->delay)));
	}
	mutex_unlock(&client_data->mutex_enable);

	return ret;
}

static int32_t bgsh_restore_hw_cfg(struct bgsh_client_data *client_data)
{
	int32_t ret = SUCCESS;
	struct reg_node node;
	uint8_t op_mode;

	PDEBUG("function entrance");

	mutex_lock(&client_data->mutex_bus_op);
	op_mode = client_data->op_mode;
	ret = bgsh_set_op_mode(client_data, OPERATION_MODE_CONFIG);
	if (OPERATION_MODE_CONFIG == op_mode) {
		mutex_unlock(&client_data->mutex_bus_op);
		return ret;
	}

	if (ret == SUCCESS)
		ret = bgsh_set_op_mode(client_data, op_mode);

	PINFO("app did not close this sensor before suspend");
	MAKE_REG_NODE(node, BGSH_OUTPUT_DATA_RATE);
	bgsh_write_node(client_data, node,
		client_data->odr, &ret);
	mdelay(BGSH_BUS_WRITE_DELAY_TIME);

	PINFO("register dump after init");
	bgsh_dump_reg(client_data);
	mutex_unlock(&client_data->mutex_bus_op);

	return ret;
}

int32_t bgsh_resume(struct device *dev)
{
	int32_t ret = SUCCESS;
	struct bgsh_client_data *client_data =
		dev_get_drvdata(dev);

	PDEBUG("function entrance");

	ret = bgsh_restore_hw_cfg(client_data);
	/* post resume operation */
	bgsh_post_resume(client_data);

	return ret;
}
EXPORT_SYMBOL(bgsh_resume);

int32_t bgsh_remove(struct device *dev)
{
	int32_t ret = SUCCESS;
	struct bgsh_client_data *client_data =
		dev_get_drvdata(dev);

	if (NULL != client_data) {
		mutex_lock(&client_data->mutex_enable);
		if (NORMAL_MODE == client_data->op_mode) {
			PINFO("Disable work in remove");
			cancel_delayed_work_sync(&client_data->work);
		}
		mutex_unlock(&client_data->mutex_enable);

		mutex_lock(&client_data->mutex_bus_op);
		ret = bgsh_set_op_mode(client_data, OPERATION_MODE_CONFIG);
		mutex_unlock(&client_data->mutex_bus_op);

		mdelay(BGSH_BUS_WRITE_DELAY_TIME);

		sysfs_remove_group(&client_data->input->dev.kobj,
				&bgsh_attribute_group);
		bgsh_input_destroy(client_data);

		if (NULL != client_data->bst_pd) {
			kfree(client_data->bst_pd);
			client_data->bst_pd = NULL;
		}
		kfree(client_data);
	}

	return ret;
}
EXPORT_SYMBOL(bgsh_remove);

#ifdef CONFIG_PM
/*!
 * @brief disable power
 *
 * @param dev the pointer of device
 *
 * @return zero
 * @retval zero
*/
int32_t bgsh_disable(struct device *dev)
{
	return 0;
}
EXPORT_SYMBOL(bgsh_disable);
#endif


MODULE_AUTHOR("Contact <contact@bosch-sensortec.com>");
MODULE_DESCRIPTION("driver for " SENSOR_NAME);
MODULE_LICENSE("GPL v2");

