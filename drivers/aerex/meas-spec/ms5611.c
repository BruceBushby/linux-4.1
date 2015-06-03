/*
 * Measurement Specialties MS5611 pressure and temperature sensor driver
 *
 * Copyright (C) 2013 William Markezana <william.markezana@meas-spec.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/jiffies.h> 
#include <linux/delay.h>

/* MS5611 Commands */
#define MS5611_CONVERT_D1_OSR_4096	(0x48)
#define MS5611_CONVERT_D2_OSR_4096	(0x58)
#define MS5611_ADC_READ			(0x00)
#define MS5611_PROM_READ		(0xA0)

 static struct i2c_client *ms5611_i2c_client = NULL;

struct ms5611 {
	struct device *hwmon_dev;
	struct mutex lock;
	bool valid;
	unsigned long last_update;
	int temperature;
	int pressure;
	unsigned short calibration_words[6];
	bool got_calibration_words;
};

static int ms5611_get_calibration_word(struct i2c_client *client,
	unsigned char address, unsigned short *word)
{
	int ret = 0;
	ret = i2c_smbus_read_word_swapped(client, 
		MS5611_PROM_READ + (address << 1));
	if(ret < 0)
		return ret;
	*word = (unsigned short)ret & 0xFFFF;
	return 0;
}

static int ms5611_fill_calibration_words(struct i2c_client *client)
{
	int i, ret = 0;
	struct ms5611 *ms5611 = i2c_get_clientdata(client);

	for(i = 0; i < 6; i++)
	{
		ret = ms5611_get_calibration_word(client, i+1,
			&ms5611->calibration_words[i]);
		if(ret < 0)
		{
			dev_err(&client->dev,
				"unable to get calibration word at address %d\n", i+1);
			return ret;
		}
	}
	return 0;
}

static int ms5611_get_adc_value(struct i2c_client *client, 
	unsigned int *adc_value)
{
	int ret = 0;
	unsigned char buf[3];
	ret = i2c_smbus_read_i2c_block_data(client, MS5611_ADC_READ, 3, buf);
	if(ret < 0)
		return ret;
	*adc_value =(buf[0] << 16) + (buf[1] << 8) + buf[2];
	return 0;
}

static int ms5611_get_conversion_result(struct i2c_client *client, 
	unsigned char command, unsigned int *adc_value)
{
	int ret;

	ret = i2c_smbus_write_byte(client, command);
	if (ret < 0)
		return ret;
	msleep(18);
	ret = ms5611_get_adc_value(client, adc_value);
	if (ret < 0)
		return ret;
	return 0;
}

static int ms5611_update_measurements(struct i2c_client *client)
{
	int ret = 0;
	unsigned int d1, d2;
	int dt, temp, p;
	long long int off, sens;
	struct ms5611 *ms5611 = i2c_get_clientdata(client);

	mutex_lock(&ms5611->lock);

	if (time_after(jiffies, ms5611->last_update + HZ / 2) ||
	    !ms5611->valid) {

		if(!ms5611->got_calibration_words)
		{
			ret = ms5611_fill_calibration_words(client);
			if(ret < 0)
			{
				dev_err(&client->dev,
					"unable to get calibration words\n");
				goto out;
			}

			ms5611->got_calibration_words = true;
		}

		ret = ms5611_get_conversion_result(client, 
			MS5611_CONVERT_D1_OSR_4096, &d1);
		if (ret < 0)
		{
			dev_err(&client->dev,
				"unable to get adc conversion of D1_OSR_8192\n");
			goto out;
		}

		ret = ms5611_get_conversion_result(client, 
			MS5611_CONVERT_D2_OSR_4096, &d2);
		if (ret < 0)
		{
			dev_err(&client->dev,
				"unable to get adc conversion of D2_OSR_8192\n");
			goto out;
		}

		dt = d2 - (int)ms5611->calibration_words[4] * 256;
		temp = 20000 + div_s64((long long int)dt * 
			(long long int)ms5611->calibration_words[5], 838861);

		off = (long long int)ms5611->calibration_words[1] * 131072 +
			div_s64((long long int)ms5611->calibration_words[3] *
			(long long int)dt, 64);
		sens = (long long int)ms5611->calibration_words[0] * 65536 +
			div_s64((long long int)ms5611->calibration_words[2] *
			(long long int)dt, 128);
		p = (int)div_s64( (div_s64((long long int)d1 * sens, 2097152) - off),
			3277);
		
		ms5611->temperature = temp;
		ms5611->pressure = p;
		ms5611->last_update = jiffies;
		ms5611->valid = true;
	}
out:
	mutex_unlock(&ms5611->lock);

	return ret >= 0 ? 0 : ret;
}

struct i2c_client *get_ms5611_i2c_client(void)
{
	return ms5611_i2c_client;
}
EXPORT_SYMBOL_GPL(get_ms5611_i2c_client);

int get_ms5611_temperature(struct i2c_client *client, int *temperature)
{
	struct ms5611 *ms5611 = i2c_get_clientdata(client);
	int ret = ms5611_update_measurements(client);
	if (ret < 0)
		return ret;
	*temperature = ms5611->temperature;
	return 0;
}
EXPORT_SYMBOL_GPL(get_ms5611_temperature);

int get_ms5611_pressure(struct i2c_client *client, int *pressure)
{
	struct ms5611 *ms5611 = i2c_get_clientdata(client);
	int ret = ms5611_update_measurements(client);
	if (ret < 0)
		return ret;
	*pressure = ms5611->pressure;
	return 0;
}
EXPORT_SYMBOL_GPL(get_ms5611_pressure);

static ssize_t ms5611_show_temperature(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	int temperature = 0;
	struct i2c_client *client = to_i2c_client(dev);
	int ret = get_ms5611_temperature(client, &temperature);
	if (ret < 0)
		return ret;
	return sprintf(buf, "%d\n", temperature);
}

static ssize_t ms5611_show_pressure(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	int pressure = 0;
	struct i2c_client *client = to_i2c_client(dev);
	int ret = get_ms5611_pressure(client, &pressure);
	if (ret < 0)
		return ret;
	return sprintf(buf, "%d\n", pressure);
}

static ssize_t ms5611_show_calibration_data(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ms5611 *ms5611 = i2c_get_clientdata(client);
	return sprintf(buf, "C1 = %d\n"\
						"C2 = %d\n"\
						"C3 = %d\n"\
						"C4 = %d\n"\
						"C5 = %d\n"\
						"C6 = %d\n", 	ms5611->calibration_words[0],
										ms5611->calibration_words[1],
										ms5611->calibration_words[2],
										ms5611->calibration_words[3],
										ms5611->calibration_words[4],
										ms5611->calibration_words[5]);
}

static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO,
			  ms5611_show_temperature, NULL, 0);
static SENSOR_DEVICE_ATTR(pressure1_input, S_IRUGO,
			  ms5611_show_pressure, NULL, 0);
static SENSOR_DEVICE_ATTR(calibration_data, S_IRUGO,
			  ms5611_show_calibration_data, NULL, 0);

static struct attribute *ms5611_attributes[] = {
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_pressure1_input.dev_attr.attr,
	&sensor_dev_attr_calibration_data.dev_attr.attr,
	NULL
};

static const struct attribute_group ms5611_group = {
	.attrs = ms5611_attributes,
};

static int ms5611_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
	struct ms5611 *ms5611;
	int err;

	if (!i2c_check_functionality(client->adapter,
					I2C_FUNC_SMBUS_READ_WORD_DATA)) {
		dev_err(&client->dev,
			"adapter does not support SMBus word transactions\n");
		return -ENODEV;
	}

	if (!i2c_check_functionality(client->adapter,
				    I2C_FUNC_SMBUS_WRITE_BYTE)) {
		dev_err(&client->dev,
			"adapter does not support SMBus byte transactions\n");
		return -ENODEV;
	}

	if (!i2c_check_functionality(client->adapter,
				    I2C_FUNC_SMBUS_READ_I2C_BLOCK)) {
		dev_err(&client->dev,
			"adapter does not support SMBus block transactions\n");
		return -ENODEV;
	}

	ms5611 = devm_kzalloc(&client->dev, sizeof(*ms5611), GFP_KERNEL);
	if (!ms5611)
		return -ENOMEM;

	i2c_set_clientdata(client, ms5611);

	mutex_init(&ms5611->lock);

	err = sysfs_create_group(&client->dev.kobj, &ms5611_group);
	if (err) {
		dev_dbg(&client->dev, "could not create sysfs files\n");
		return err;
	}
	ms5611->hwmon_dev = hwmon_device_register(&client->dev);
	if (IS_ERR(ms5611->hwmon_dev)) {
		dev_dbg(&client->dev, "unable to register hwmon device\n");
		err = PTR_ERR(ms5611->hwmon_dev);
		goto error;
	}

	mutex_lock(&ms5611->lock);
	err = ms5611_fill_calibration_words(client);
	if(err >= 0)
		ms5611->got_calibration_words = true;
	mutex_unlock(&ms5611->lock);

	dev_info(&client->dev, "initialized\n");
	ms5611_i2c_client = client;

	return 0;

error:
	sysfs_remove_group(&client->dev.kobj, &ms5611_group);
	return err;
}

static int ms5611_remove(struct i2c_client *client)
{
	struct ms5611 *ms5611 = i2c_get_clientdata(client);

	hwmon_device_unregister(ms5611->hwmon_dev);
	sysfs_remove_group(&client->dev.kobj, &ms5611_group);

	ms5611_i2c_client = NULL;

	return 0;
}

static const struct i2c_device_id ms5611_id[] = {
	{ "ms5611", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ms5611_id);

static struct i2c_driver ms5611_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name	= "ms5611",
	},
	.probe       = ms5611_probe,
	.remove      = ms5611_remove,
	.id_table    = ms5611_id,
};

module_i2c_driver(ms5611_driver);

MODULE_AUTHOR("William Markezana <william.markezana@meas-spec.com>");
MODULE_DESCRIPTION("MEAS MS5611 pressure and temperature sensor driver");
MODULE_LICENSE("GPL");
