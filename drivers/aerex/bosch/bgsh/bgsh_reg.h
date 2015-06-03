/*!
 * @section LICENSE
 * (C) Copyright 2011~2014 Bosch Sensortec GmbH All Rights Reserved
 *
 * This software program is licensed subject to the GNU General
 * Public License (GPL).Version 2,June 1991,
 * available at http://www.fsf.org/copyleft/gpl.html
 *
 * @filename bgsh_reg.h
 * @date     "Tue Aug 19 16:21:59 2014 +0800"
 * @id       "a9250a8"
 * @version  v1.3
 * @brief    Bosch Generic Sensor Hub Linux Driver API
*/

/****************************************************************************/
/*! \file bgsh_reg.h
    \brief BGSH Sensor Driver Support Header File */

#ifndef __BGSH_REG_H__
#define __BGSH_REG_H__

#include <linux/limits.h> /* needed to test integer limits */

#ifdef __KERNEL__
#define BGSH_U16 unsigned short
#define BGSH_S16 signed short
#else

/* find correct data type for signed/unsigned 16 bit
   variables by checking max of unsigned variant */
#if (USHRT_MAX == 0xFFFF)
	/* 16 bit achieved with short */
	#define BGSH_U16 unsigned short
	#define BGSH_S16 signed short
#elif (UINT_MAX == 0xFFFF)
	/* 16 bit achieved with int */
	#define BGSH_U16 unsigned int
	#define BGSH_S16 signed int
#else
	#error BGSH_U16 and BGSH_S16 \
	#error could not be defined automatically, please do so manually
#endif
#endif

enum SENSOR_HANDLE {
	SENSOR_HANDLE_INVALID,
	SENSOR_HANDLE_ACCELERATION,
	SENSOR_HANDLE_GYROSCOPE,
	SENSOR_HANDLE_MAGNETIC_FIELD,
	SENSOR_HANDLE_ORIENTATION,
	SENSOR_HANDLE_GRAVITY,
	SENSOR_HANDLE_LINEAR_ACCELERATION,
	SENSOR_HANDLE_ROTATION_VECTOR,
	SENSOR_HANDLE_END
};

/* register dump end address, depends on chip */
#define BGSH_REGISTER_MAP_END        0x80
#define BGSH_REGISTER_PAGEID_MAX        1

/* For Enabling and Disabling the floating point API's */
#define ENABLE_FLOAT

#define BGSH_S32 signed int

#define BGSH_WR_FUNC_PTR int8_t (*bus_write)\
(uint8_t, uint8_t, uint8_t *, uint8_t)

#define BGSH_BUS_WRITE_FUNC(dev_addr, reg_addr, reg_data, wr_len)\
	bus_write(dev_addr, reg_addr, reg_data, wr_len)

#define BGSH_RD_FUNC_PTR int8_t (*bus_read)\
(uint8_t, uint8_t, uint8_t *, uint8_t)

#define BGSH_BUS_READ_FUNC(dev_addr, reg_addr, reg_data, r_len)\
	bus_read(dev_addr, reg_addr, reg_data, r_len)

#if FW_DOWNLOAD_SUPPORT == 1
#define BGSH_WR_NOREG_FUNC_PTR int8_t (*bus_write_noreg)\
(uint8_t, uint8_t *, uint8_t)

#define BGSH_BUS_WRITE_NOREG_FUNC(dev_addr, reg_data, wr_len)\
	bus_write_noreg(dev_addr, reg_data, wr_len)

#define BGSH_RD_NOREG_FUNC_PTR int8_t (*bus_read_noreg)\
(uint8_t, uint8_t *, uint8_t)

#define BGSH_BUS_READ_NOREG_FUNC(dev_addr, reg_data, r_len)\
	bus_read_noreg(dev_addr, reg_data, r_len)
#endif

#define BGSH_DELAY_RETURN_TYPE void

#define BGSH_DELAY_PARAM_TYPES unsigned int

#define BGSH_DELAY_FUNC(delay_in_msec)\
	delay_func(delay_in_msec)

/* bgsh BUS Address */
#define BGSH_BUS_ADDR1                0x28
#define BGSH_BUS_ADDR2                0x29
#define BGSH_BUS_ADDR                 BGSH_BUS_ADDR2

/* PAGE0 REGISTER DEFINITION START*/
#define BGSH_CHIP_ID_ADDR                 0x00
#define BGSH_ACC_REV_ID_ADDR              0x01
#define BGSH_MAG_REV_ID_ADDR              0x02
#define BGSH_GYR_REV_ID_ADDR              0x03
#define BGSH_SW_REV_ID_LSB_ADDR           0x04
#define BGSH_SW_REV_ID_MSB_ADDR           0x05
#define BGSH_BL_REV_ID_ADDR               0X06
#define BGSH_PAGE_ID_ADDR                 0X07

/* Accel data register*/
#define BGSH_ACC_DATA_X_LSB_ADDR            0X08
#define BGSH_ACC_DATA_X_MSB_ADDR            0X09
#define BGSH_ACC_DATA_Y_LSB_ADDR            0X0A
#define BGSH_ACC_DATA_Y_MSB_ADDR            0X0B
#define BGSH_ACC_DATA_Z_LSB_ADDR            0X0C
#define BGSH_ACC_DATA_Z_MSB_ADDR            0X0D

/*Mag data register*/
#define BGSH_MAG_DATA_X_LSB_ADDR            0X0E
#define BGSH_MAG_DATA_X_MSB_ADDR            0X0F
#define BGSH_MAG_DATA_Y_LSB_ADDR            0X10
#define BGSH_MAG_DATA_Y_MSB_ADDR            0X11
#define BGSH_MAG_DATA_Z_LSB_ADDR            0X12
#define BGSH_MAG_DATA_Z_MSB_ADDR            0X13

/*Gyro data registers*/
#define BGSH_GYR_DATA_X_LSB_ADDR            0X14
#define BGSH_GYR_DATA_X_MSB_ADDR            0X15
#define BGSH_GYR_DATA_Y_LSB_ADDR            0X16
#define BGSH_GYR_DATA_Y_MSB_ADDR            0X17
#define BGSH_GYR_DATA_Z_LSB_ADDR            0X18
#define BGSH_GYR_DATA_Z_MSB_ADDR            0X19

/*Euler data registers*/
#define BGSH_EUL_HEADING_LSB_ADDR           0X1A
#define BGSH_EUL_HEADING_MSB_ADDR           0X1B

#define BGSH_EUL_ROLL_LSB_ADDR              0X1C
#define BGSH_EUL_ROLL_MSB_ADDR              0X1D

#define BGSH_EUL_PITCH_LSB_ADDR             0X1E
#define BGSH_EUL_PITCH_MSB_ADDR             0X1F

/*Quaternion data registers*/
#define BGSH_QUA_DATA_W_LSB_ADDR            0X20
#define BGSH_QUA_DATA_W_MSB_ADDR            0X21
#define BGSH_QUA_DATA_X_LSB_ADDR            0X22
#define BGSH_QUA_DATA_X_MSB_ADDR            0X23
#define BGSH_QUA_DATA_Y_LSB_ADDR            0X24
#define BGSH_QUA_DATA_Y_MSB_ADDR            0X25
#define BGSH_QUA_DATA_Z_LSB_ADDR            0X26
#define BGSH_QUA_DATA_Z_MSB_ADDR            0X27

/* Linear acceleration data registers*/
#define BGSH_LIA_DATA_X_LSB_ADDR            0X28
#define BGSH_LIA_DATA_X_MSB_ADDR            0X29
#define BGSH_LIA_DATA_Y_LSB_ADDR            0X2A
#define BGSH_LIA_DATA_Y_MSB_ADDR            0X2B
#define BGSH_LIA_DATA_Z_LSB_ADDR            0X2C
#define BGSH_LIA_DATA_Z_MSB_ADDR            0X2D

/*Gravity data registers*/
#define BGSH_GRV_DATA_X_LSB_ADDR            0X2E
#define BGSH_GRV_DATA_X_MSB_ADDR            0X2F
#define BGSH_GRV_DATA_Y_LSB_ADDR            0X30
#define BGSH_GRV_DATA_Y_MSB_ADDR            0X31
#define BGSH_GRV_DATA_Z_LSB_ADDR            0X32
#define BGSH_GRV_DATA_Z_MSB_ADDR            0X33

/* Temperature data register*/

#define BGSH_TEMP_ADDR                      0X34

/* Status registers*/
#define BGSH_CALIB_STAT_ADDR                0X35
#define BGSH_ST_RESULT_ADDR                 0X36
#define BGSH_INT_STA_ADDR                   0X37
#define BGSH_SYS_STATUS_ADDR                0X39
#define BGSH_SYS_ERR_ADDR                   0X3A

/* Unit selection register*/
#define BGSH_UNIT_SEL_ADDR                 0X3B
#define BGSH_DATA_SEL_ADDR                 0X3C

/* Mode registers*/
#define BGSH_OPR_MODE_ADDR                 0X3D
#define BGSH_PWR_MODE_ADDR                 0X3E

#define BGSH_SYS_TRIGGER_ADDR              0X3F
#define BGSH_TEMP_SOURCE_ADDR              0X40
/* Axis remap registers*/
#define BGSH_AXIS_MAP_CONFIG_ADDR          0X41
#define BGSH_AXIS_MAP_SIGN_ADDR            0X42

/* SIC registers*/
#define BGSH_SIC_MATRIX_0_LSB_ADDR        0X43
#define BGSH_SIC_MATRIX_0_MSB_ADDR        0X44
#define BGSH_SIC_MATRIX_1_LSB_ADDR        0X45
#define BGSH_SIC_MATRIX_1_MSB_ADDR        0X46
#define BGSH_SIC_MATRIX_2_LSB_ADDR        0X47
#define BGSH_SIC_MATRIX_2_MSB_ADDR        0X48
#define BGSH_SIC_MATRIX_3_LSB_ADDR        0X49
#define BGSH_SIC_MATRIX_3_MSB_ADDR        0X4A
#define BGSH_SIC_MATRIX_4_LSB_ADDR        0X4B
#define BGSH_SIC_MATRIX_4_MSB_ADDR        0X4C
#define BGSH_SIC_MATRIX_5_LSB_ADDR        0X4D
#define BGSH_SIC_MATRIX_5_MSB_ADDR        0X4E
#define BGSH_SIC_MATRIX_6_LSB_ADDR        0X4F
#define BGSH_SIC_MATRIX_6_MSB_ADDR        0X50
#define BGSH_SIC_MATRIX_7_LSB_ADDR        0X51
#define BGSH_SIC_MATRIX_7_MSB_ADDR        0X52
#define BGSH_SIC_MATRIX_8_LSB_ADDR        0X53
#define BGSH_SIC_MATRIX_8_MSB_ADDR        0X54

/* Accelerometer Offset registers*/
#define ACC_OFFSET_X_LSB_ADDR                0X55
#define ACC_OFFSET_X_MSB_ADDR                0X56
#define ACC_OFFSET_Y_LSB_ADDR                0X57
#define ACC_OFFSET_Y_MSB_ADDR                0X58
#define ACC_OFFSET_Z_LSB_ADDR                0X59
#define ACC_OFFSET_Z_MSB_ADDR                0X5A

/* Magnetometer Offset registers*/
#define MAG_OFFSET_X_LSB_ADDR                0X5B
#define MAG_OFFSET_X_MSB_ADDR                0X5C
#define MAG_OFFSET_Y_LSB_ADDR                0X5D
#define MAG_OFFSET_Y_MSB_ADDR                0X5E
#define MAG_OFFSET_Z_LSB_ADDR                0X5F
#define MAG_OFFSET_Z_MSB_ADDR                0X60

/* Gyroscope Offset registers*/
#define GYRO_OFFSET_X_LSB_ADDR               0X61
#define GYRO_OFFSET_X_MSB_ADDR               0X62
#define GYRO_OFFSET_Y_LSB_ADDR               0X63
#define GYRO_OFFSET_Y_MSB_ADDR               0X64
#define GYRO_OFFSET_Z_LSB_ADDR               0X65
#define GYRO_OFFSET_Z_MSB_ADDR               0X66

/* Boot config*/
#define BGSH_BOOT_CONFIG_ADDR                   0x7f
/* PAGE0 REGISTERS DEFINITION END*/

/* PAGE1 REGISTERS DEFINITION START*/
/* Configuration registers*/
#define ACC_CONFIG_ADDR                      0X08
#define MAG_CONFIG_ADDR                      0X09
#define GYRO_CONFIG_ADDR                     0X0A
#define GYRO_MODE_CONFIG_ADDR                0X0B
#define ACC_SLEEP_CONFIG_ADDR                0X0C
#define GYR_SLEEP_CONFIG_ADDR                0X0D
#define MAG_SLEEP_CONFIG_ADDR                0x0E

/* Interrupt registers*/
#define INT_MSK_ADDR                         0X0F
#define INT_ADDR                             0X10
#define ACC_AM_THRES_ADDR                    0X11
#define ACC_INT_SETTINGS_ADDR                0X12
#define ACC_HG_DURATION_ADDR                 0X13
#define ACC_HG_THRES_ADDR                    0X14
#define ACC_NM_THRES_ADDR                    0X15
#define ACC_NM_SET_ADDR                      0X16
#define GYR_INT_SETING_ADDR                  0X17
#define GYR_HR_X_SET_ADDR                    0X18
#define GYR_DUR_X_ADDR                       0X19
#define GYR_HR_Y_SET_ADDR                    0X1A
#define GYR_DUR_Y_ADDR                       0X1B
#define GYR_HR_Z_SET_ADDR                    0X1C
#define GYR_DUR_Z_ADDR                       0X1D
#define GYR_AM_THRES_ADDR                    0X1E
#define GYR_AM_SET_ADDR                      0X1F
/* PAGE1 REGISTERS DEFINITION END*/


#define BGSH_MDELAY_DATA_TYPE           uint32_t

/*< This refers BGSH return type as char */
#define BGSH_RETURN_FUNCTION_TYPE        int32_t


/*BGSH-STRUCT*/
struct bgsh_t {
uint8_t chip_id;
uint16_t sw_revision_id;
uint8_t page_id;
uint8_t accel_revision_id;
uint8_t mag_revision_id;
uint8_t gyro_revision_id;
uint8_t bootloder_revision_id;
uint8_t dev_addr;
BGSH_WR_FUNC_PTR;
BGSH_RD_FUNC_PTR;
#if FW_DOWNLOAD_SUPPORT == 1
BGSH_WR_NOREG_FUNC_PTR;
BGSH_RD_NOREG_FUNC_PTR;
#endif

void (*delay_msec)(BGSH_MDELAY_DATA_TYPE);
};

/*BGSH-Accel x,y,z*/
struct bgsh_accel {
BGSH_S16 x;
BGSH_S16 y;
BGSH_S16 z;
};

/*BGSH-mag x,y,z*/
struct bgsh_mag {
BGSH_S16 x;
BGSH_S16 y;
BGSH_S16 z;
};

/*BGSH-Gyro x,y,z*/
struct bgsh_gyro {
BGSH_S16 x;
BGSH_S16 y;
BGSH_S16 z;
};

/*BGSH-Euler h,r,p*/
struct bgsh_euler {
BGSH_S16 h;
BGSH_S16 r;
BGSH_S16 p;
};

/*BGSH-quaternion w,x,y,z*/
struct bgsh_quaternion {
BGSH_S16 w;
BGSH_S16 x;
BGSH_S16 y;
BGSH_S16 z;
};

/*BGSH-Linear Accel x,y,z*/
struct bgsh_linear_accel {
BGSH_S16 x;
BGSH_S16 y;
BGSH_S16 z;
};

/*BGSH-Gravity x,y,z*/
struct bgsh_gravity {
BGSH_S16 x;
BGSH_S16 y;
BGSH_S16 z;
};

#define         BGSH_Zero_U8X           (uint8_t)0
#define         BGSH_Two_U8X            (uint8_t)2
#define         BGSH_Four_U8X           (uint8_t)4
#define         BGSH_Five_U8X           (uint8_t)5
#define         BGSH_Six_U8X            (uint8_t)6
#define         BGSH_Seven_U8X          (uint8_t)7
#define         BGSH_Eight_U8X          (uint8_t)8
#define         BGSH_Eleven_U8X         (uint8_t)11
#define         BGSH_Sixteen_U8X        (uint8_t)16

#define         BGSH_SFT_8_POS   (uint8_t)8
#define         BGSH_SFT_16_POS   (uint8_t)16
#define         BGSH_SFT_24_POS   (uint8_t)24

/*  BGSH API error codes */
#define E_NULL_PTR                  (int8_t)(-127)
#define E_BGSH_OUT_OF_RANGE         (int8_t)(-2)
#define SUCCESS                     (uint8_t)0
#define ERROR                       (int8_t)(-1)

enum {
	SENSOR_POWER_MODE_NORMAL = 0,
	SENSOR_POWER_MODE_LOWERPOWER = 1,
	SENSOR_POWER_MODE_SUSPEND = 2,
};

/* Page ID */
#define PAGE_ZERO        0X00
#define PAGE_ONE         0X01
#define PAGE_TWO         0X02
#define PAGE_THREE       0X03

/* Operation mode settings*/
#define OPERATION_MODE_CONFIG            0X00
#define OPERATION_MODE_ACCONLY           0X01
#define OPERATION_MODE_MAGONLY           0X02
#define OPERATION_MODE_GYRONLY           0X03
#define OPERATION_MODE_ACCMAG            0X04
#define OPERATION_MODE_ACCGYRO           0X05
#define OPERATION_MODE_MAGGYRO           0X06
#define OPERATION_MODE_AMG               0X07
#define OPERATION_MODE_IMUPLUS           0X08
#define OPERATION_MODE_COMPASS           0X09
#define OPERATION_MODE_M4G               0X0A
#define OPERATION_MODE_NDOF_FMC_OFF      0X0B
#define OPERATION_MODE_NDOF              0X0C

/* Data output rates*/
#define FASTEST_MODE_1        0X00
#define FASTEST_MODE_2        0X01
#define GAME_MODE             0X02
#define UI_MODE               0X04
#define NORMAL_MODE           0X05

/* Unit select */
#define ORITATION_ANDROID    1
#define UNIT_TEMPERATURE_CELSIUS       0
#define UNIT_TEMPERATURE_FAHRENHEIT    1
#define UNIT_EULER_DEG    0
#define UNIT_EULER_RAD    1
#define UNIT_GYRO_DPS     0
#define UNIT_GYRO_RPS     1
#define UNIT_ACC_MS2      0
#define UNIT_ACC_MG       1

/* Accel Range */
#define ACCEL_RANGE_2G        0X00
#define ACCEL_RANGE_4G        0X01
#define ACCEL_RANGE_8G        0X02
#define ACCEL_RANGE_16G       0X03

/* Accel Bandwidth*/
#define ACCEL_BW_7_81Hz       0x00
#define ACCEL_BW_15_63Hz      0x01
#define ACCEL_BW_31_25Hz      0x02
#define ACCEL_BW_62_5Hz       0X03
#define ACCEL_BW_125Hz        0X04
#define ACCEL_BW_250Hz        0X05
#define ACCEL_BW_500Hz        0X06
#define ACCEL_BW_1000Hz       0X07

/* Accel Power mode*/
#define ACCEL_NORMAL            0X00
#define ACCEL_SUSPEND           0X01
#define ACCEL_LOWPOWER_1        0X02
#define ACCEL_STANDBY           0X03
#define ACCEL_LOWPOWER_2        0X04
#define ACCEL_DEEPSUSPEND       0X05

/* Mag data output rate*/
#define MAG_DATA_OUTRATE_2Hz        0X00
#define MAG_DATA_OUTRATE_6Hz        0X01
#define MAG_DATA_OUTRATE_8Hz        0X02
#define MAG_DATA_OUTRATE_10Hz       0X03
#define MAG_DATA_OUTRATE_15Hz       0X04
#define MAG_DATA_OUTRATE_20Hz       0X05
#define MAG_DATA_OUTRATE_25Hz       0X06
#define MAG_DATA_OUTRATE_30Hz       0X07

/* Mag Operation mode*/
#define MAG_OPR_MODE_LOWPOWER                0X00
#define MAG_OPR_MODE_REGULAR                 0X01
#define MAG_OPR_MODE_ENHANCED_REGULAR        0X02
#define MAG_OPR_MODE_HIGH_ACCURACY           0X03

/* Mag power mode*/
#define MAG_POWER_MODE_NORMAL                   0X00
#define MAG_POWER_MODE_SLEEP                    0X01
#define MAG_POWER_MODE_SUSPEND                  0X02
#define MAG_POWER_MODE_FORCE_MODE               0X03

/* Gyro range*/
#define GYRO_RANGE_2000rps        0x00
#define GYRO_RANGE_1000rps        0x01
#define GYRO_RANGE_500rps         0x02
#define GYRO_RANGE_250rps         0x03
#define GYRO_RANGE_125rps         0x04

/* Gyro Bandwidth*/
#define GYRO_BW_523Hz        0x00
#define GYRO_BW_230Hz        0x01
#define GYRO_BW_116Hz        0x02
#define GYRO_BW_47Hz         0x03
#define GYRO_BW_23Hz         0x04
#define GYRO_BW_12Hz         0x05
#define GYRO_BW_64Hz         0x06
#define GYRO_BW_32Hz         0x07

/* Gyro Operation mode*/
#define GYRO_OPR_MODE_NORMAL                 0X00
#define GYRO_OPR_MODE_FASTPOWERUP            0X01
#define GYRO_OPR_MODE_DEEPSUSPEND            0X02
#define GYRO_OPR_MODE_SUSPEND                0X03
#define GYRO_OPR_MODE_ADVANCE_POWERSAVE      0X04

/* Accel Sleep Duration */
#define BGSH_ACCEL_SLEEP_DUR_0_5MS        0x05
/* sets sleep duration to 0.5 ms  */
#define BGSH_ACCEL_SLEEP_DUR_1MS          0x06
/* sets sleep duration to 1 ms */
#define BGSH_ACCEL_SLEEP_DUR_2MS          0x07
/* sets sleep duration to 2 ms */
#define BGSH_ACCEL_SLEEP_DUR_4MS          0x08
/* sets sleep duration to 4 ms */
#define BGSH_ACCEL_SLEEP_DUR_6MS          0x09
/* sets sleep duration to 6 ms*/
#define BGSH_ACCEL_SLEEP_DUR_10MS         0x0A
/* sets sleep duration to 10 ms */
#define BGSH_ACCEL_SLEEP_DUR_25MS         0x0B
 /* sets sleep duration to 25 ms */
#define BGSH_ACCEL_SLEEP_DUR_50MS         0x0C
 /* sets sleep duration to 50 ms */
#define BGSH_ACCEL_SLEEP_DUR_100MS        0x0D
 /* sets sleep duration to 100 ms */
#define BGSH_ACCEL_SLEEP_DUR_500MS        0x0E
 /* sets sleep duration to 500 ms */
#define BGSH_ACCEL_SLEEP_DUR_1S           0x0F
/* sets sleep duration to 1 s */

/* Gyro Auto sleep duration*/
#define BGSH_GYRO_No_AuSlpDur        0x00
#define BGSH_GYRO_4ms_AuSlpDur       0x01
#define BGSH_GYRO_5ms_AuSlpDur       0x02
#define BGSH_GYRO_8ms_AuSlpDur       0x03
#define BGSH_GYRO_10ms_AuSlpDur      0x04
#define BGSH_GYRO_15ms_AuSlpDur      0x05
#define BGSH_GYRO_20ms_AuSlpDur      0x06
#define BGSH_GYRO_40ms_AuSlpDur      0x07

/* Accel Any/No motion axis selection*/
#define BGSH_ACCEL_AM_NM_X_AXIS        0
#define BGSH_ACCEL_AM_NM_Y_AXIS        1
#define BGSH_ACCEL_AM_NM_Z_AXIS        2

/* Accel High g axis selection*/
#define BGSH_ACCEL_HIGH_G_X_AXIS       0
#define BGSH_ACCEL_HIGH_G_Y_AXIS       1
#define BGSH_ACCEL_HIGH_G_Z_AXIS       2

/* Gyro Any motion axis selection*/
#define BGSH_GYRO_AM_X_AXIS        0
#define BGSH_GYRO_AM_Y_AXIS        1
#define BGSH_GYRO_AM_Z_AXIS        2

/* Gyro High rate axis selection*/
#define BGSH_GYRO_HR_X_AXIS        0
#define BGSH_GYRO_HR_Y_AXIS        1
#define BGSH_GYRO_HR_Z_AXIS        2

/* Axis remap values*/
#define REMAP_X_Y            0X21
#define REMAP_Y_Z            0X18
#define REMAP_Z_X            0X06
#define REMAP_X_Y_Z_TYPE0    0X12
#define REMAP_X_Y_Z_TYPE1    0X09
#define DEFAULT_AXIS         0X24

#define DISABLE 0
#define ENABLE 1

/* Reset mode */
enum {
	RESET_TO_BOOTLOADER_MODE = 0,
	RESET_TO_NORMAL_MODE = 1,
};

/* PAGE0 DATA REGISTERS DEFINITION START*/
/* Chip ID*/
#define BGSH_CHIP_ID__POS        0
#define BGSH_CHIP_ID__MSK        0xFF
#define BGSH_CHIP_ID__LEN        8
#define BGSH_CHIP_ID__PID        PAGE_ZERO
#define BGSH_CHIP_ID__REG        BGSH_CHIP_ID_ADDR

/* Accel revision id*/
#define BGSH_ACC_REV_ID__POS             0
#define BGSH_ACC_REV_ID__MSK             0xFF
#define BGSH_ACC_REV_ID__LEN             8
#define BGSH_ACC_REV_ID__PID             PAGE_ZERO
#define BGSH_ACC_REV_ID__REG             BGSH_ACC_REV_ID_ADDR

/* Mag revision id*/
#define BGSH_MAG_REV_ID__POS             0
#define BGSH_MAG_REV_ID__MSK             0xFF
#define BGSH_MAG_REV_ID__LEN             8
#define BGSH_MAG_REV_ID__PID             PAGE_ZERO
#define BGSH_MAG_REV_ID__REG             BGSH_MAG_REV_ID_ADDR

/* Gyro revision id*/
#define BGSH_GYR_REV_ID__POS             0
#define BGSH_GYR_REV_ID__MSK             0xFF
#define BGSH_GYR_REV_ID__LEN             8
#define BGSH_GYR_REV_ID__PID             PAGE_ZERO
#define BGSH_GYR_REV_ID__REG             BGSH_GYR_REV_ID_ADDR

/*Software revision id LSB*/
#define BGSH_SW_REV_ID_LSB__POS             0
#define BGSH_SW_REV_ID_LSB__MSK             0xFF
#define BGSH_SW_REV_ID_LSB__LEN             8
#define BGSH_SW_REV_ID_LSB__PID             PAGE_ZERO
#define BGSH_SW_REV_ID_LSB__REG             BGSH_SW_REV_ID_LSB_ADDR

/*Software revision id MSB*/
#define BGSH_SW_REV_ID_MSB__POS             0
#define BGSH_SW_REV_ID_MSB__MSK             0xFF
#define BGSH_SW_REV_ID_MSB__LEN             8
#define BGSH_SW_REV_ID_MSB__PID             PAGE_ZERO
#define BGSH_SW_REV_ID_MSB__REG             BGSH_SW_REV_ID_MSB_ADDR

/* BOOTLODER revision id*/
#define BGSH_BL_REV_ID__POS             0
#define BGSH_BL_REV_ID__MSK             0xFF
#define BGSH_BL_REV_ID__LEN             8
#define BGSH_BL_REV_ID__PID             PAGE_ZERO
#define BGSH_BL_REV_ID__REG             BGSH_BL_REV_ID_ADDR

/*Page id*/
#define BGSH_PAGE_ID__POS             0
#define BGSH_PAGE_ID__MSK             0xFF
#define BGSH_PAGE_ID__LEN             8
#define BGSH_PAGE_ID__PID             PAGE_ZERO
#define BGSH_PAGE_ID__REG             BGSH_PAGE_ID_ADDR

/* Accel data X-LSB register*/
#define BGSH_ACC_DATA_X_LSB_VALUEX__POS        0
#define BGSH_ACC_DATA_X_LSB_VALUEX__MSK        0xFF
#define BGSH_ACC_DATA_X_LSB_VALUEX__LEN        8
#define BGSH_ACC_DATA_X_LSB_VALUEX__PID        PAGE_ZERO
#define BGSH_ACC_DATA_X_LSB_VALUEX__REG        BGSH_ACC_DATA_X_LSB_ADDR

/* Accel data X-MSB register*/
#define BGSH_ACC_DATA_X_MSB_VALUEX__POS        0
#define BGSH_ACC_DATA_X_MSB_VALUEX__MSK        0xFF
#define BGSH_ACC_DATA_X_MSB_VALUEX__LEN        8
#define BGSH_ACC_DATA_X_MSB_VALUEX__PID        PAGE_ZERO
#define BGSH_ACC_DATA_X_MSB_VALUEX__REG        BGSH_ACC_DATA_X_MSB_ADDR

/* Accel data Y-LSB register*/
#define BGSH_ACC_DATA_Y_LSB_VALUEY__POS        0
#define BGSH_ACC_DATA_Y_LSB_VALUEY__MSK        0xFF
#define BGSH_ACC_DATA_Y_LSB_VALUEY__LEN        8
#define BGSH_ACC_DATA_Y_LSB_VALUEY__PID        PAGE_ZERO
#define BGSH_ACC_DATA_Y_LSB_VALUEY__REG        BGSH_ACC_DATA_Y_LSB_ADDR

/* Accel data Y-MSB register*/
#define BGSH_ACC_DATA_Y_MSB_VALUEY__POS        0
#define BGSH_ACC_DATA_Y_MSB_VALUEY__MSK        0xFF
#define BGSH_ACC_DATA_Y_MSB_VALUEY__LEN        8
#define BGSH_ACC_DATA_Y_MSB_VALUEY__PID        PAGE_ZERO
#define BGSH_ACC_DATA_Y_MSB_VALUEY__REG        BGSH_ACC_DATA_Y_MSB_ADDR

/* Accel data Z-LSB register*/
#define BGSH_ACC_DATA_Z_LSB_VALUEZ__POS        0
#define BGSH_ACC_DATA_Z_LSB_VALUEZ__MSK        0xFF
#define BGSH_ACC_DATA_Z_LSB_VALUEZ__LEN        8
#define BGSH_ACC_DATA_Z_LSB_VALUEZ__PID        PAGE_ZERO
#define BGSH_ACC_DATA_Z_LSB_VALUEZ__REG        BGSH_ACC_DATA_Z_LSB_ADDR

/* Accel data Z-MSB register*/
#define BGSH_ACC_DATA_Z_MSB_VALUEZ__POS        0
#define BGSH_ACC_DATA_Z_MSB_VALUEZ__MSK        0xFF
#define BGSH_ACC_DATA_Z_MSB_VALUEZ__LEN        8
#define BGSH_ACC_DATA_Z_MSB_VALUEZ__PID        PAGE_ZERO
#define BGSH_ACC_DATA_Z_MSB_VALUEZ__REG        BGSH_ACC_DATA_Z_MSB_ADDR

/* Mag data X-LSB register*/
#define BGSH_MAG_DATA_X_LSB_VALUEX__POS        0
#define BGSH_MAG_DATA_X_LSB_VALUEX__MSK        0xFF
#define BGSH_MAG_DATA_X_LSB_VALUEX__LEN        8
#define BGSH_MAG_DATA_X_LSB_VALUEX__PID        PAGE_ZERO
#define BGSH_MAG_DATA_X_LSB_VALUEX__REG        BGSH_MAG_DATA_X_LSB_ADDR

/* Mag data X-MSB register*/
#define BGSH_MAG_DATA_X_MSB_VALUEX__POS        0
#define BGSH_MAG_DATA_X_MSB_VALUEX__MSK        0xFF
#define BGSH_MAG_DATA_X_MSB_VALUEX__LEN        8
#define BGSH_MAG_DATA_X_MSB_VALUEX__PID        PAGE_ZERO
#define BGSH_MAG_DATA_X_MSB_VALUEX__REG        BGSH_MAG_DATA_X_MSB_ADDR

/* Mag data Y-LSB register*/
#define BGSH_MAG_DATA_Y_LSB_VALUEY__POS        0
#define BGSH_MAG_DATA_Y_LSB_VALUEY__MSK        0xFF
#define BGSH_MAG_DATA_Y_LSB_VALUEY__LEN        8
#define BGSH_MAG_DATA_Y_LSB_VALUEY__PID        PAGE_ZERO
#define BGSH_MAG_DATA_Y_LSB_VALUEY__REG        BGSH_MAG_DATA_Y_LSB_ADDR

/* Mag data Y-MSB register*/
#define BGSH_MAG_DATA_Y_MSB_VALUEY__POS        0
#define BGSH_MAG_DATA_Y_MSB_VALUEY__MSK        0xFF
#define BGSH_MAG_DATA_Y_MSB_VALUEY__LEN        8
#define BGSH_MAG_DATA_Y_MSB_VALUEY__PID        PAGE_ZERO
#define BGSH_MAG_DATA_Y_MSB_VALUEY__REG        BGSH_MAG_DATA_Y_MSB_ADDR

/* Mag data Z-LSB register*/
#define BGSH_MAG_DATA_Z_LSB_VALUEZ__POS        0
#define BGSH_MAG_DATA_Z_LSB_VALUEZ__MSK        0xFF
#define BGSH_MAG_DATA_Z_LSB_VALUEZ__LEN        8
#define BGSH_MAG_DATA_Z_LSB_VALUEZ__PID        PAGE_ZERO
#define BGSH_MAG_DATA_Z_LSB_VALUEZ__REG        BGSH_MAG_DATA_Z_LSB_ADDR

/* Mag data Z-MSB register*/
#define BGSH_MAG_DATA_Z_MSB_VALUEZ__POS        0
#define BGSH_MAG_DATA_Z_MSB_VALUEZ__MSK        0xFF
#define BGSH_MAG_DATA_Z_MSB_VALUEZ__LEN        8
#define BGSH_MAG_DATA_Z_MSB_VALUEZ__PID        PAGE_ZERO
#define BGSH_MAG_DATA_Z_MSB_VALUEZ__REG        BGSH_MAG_DATA_Z_MSB_ADDR

/* Gyro data X-LSB register*/
#define BGSH_GYR_DATA_X_LSB_VALUEX__POS        0
#define BGSH_GYR_DATA_X_LSB_VALUEX__MSK        0xFF
#define BGSH_GYR_DATA_X_LSB_VALUEX__LEN        8
#define BGSH_GYR_DATA_X_LSB_VALUEX__PID        PAGE_ZERO
#define BGSH_GYR_DATA_X_LSB_VALUEX__REG        BGSH_GYR_DATA_X_LSB_ADDR

/* Gyro data X-MSB register*/
#define BGSH_GYR_DATA_X_MSB_VALUEX__POS        0
#define BGSH_GYR_DATA_X_MSB_VALUEX__MSK        0xFF
#define BGSH_GYR_DATA_X_MSB_VALUEX__LEN        8
#define BGSH_GYR_DATA_X_MSB_VALUEX__PID        PAGE_ZERO
#define BGSH_GYR_DATA_X_MSB_VALUEX__REG        BGSH_GYR_DATA_X_MSB_ADDR

/* Gyro data Y-LSB register*/
#define BGSH_GYR_DATA_Y_LSB_VALUEY__POS        0
#define BGSH_GYR_DATA_Y_LSB_VALUEY__MSK        0xFF
#define BGSH_GYR_DATA_Y_LSB_VALUEY__LEN        8
#define BGSH_GYR_DATA_Y_LSB_VALUEY__PID        PAGE_ZERO
#define BGSH_GYR_DATA_Y_LSB_VALUEY__REG        BGSH_GYR_DATA_Y_LSB_ADDR

/* Gyro data Y-MSB register*/
#define BGSH_GYR_DATA_Y_MSB_VALUEY__POS        0
#define BGSH_GYR_DATA_Y_MSB_VALUEY__MSK        0xFF
#define BGSH_GYR_DATA_Y_MSB_VALUEY__LEN        8
#define BGSH_GYR_DATA_Y_MSB_VALUEY__PID        PAGE_ZERO
#define BGSH_GYR_DATA_Y_MSB_VALUEY__REG        BGSH_GYR_DATA_Y_MSB_ADDR

/* Gyro data Z-LSB register*/
#define BGSH_GYR_DATA_Z_LSB_VALUEZ__POS        0
#define BGSH_GYR_DATA_Z_LSB_VALUEZ__MSK        0xFF
#define BGSH_GYR_DATA_Z_LSB_VALUEZ__LEN        8
#define BGSH_GYR_DATA_Z_LSB_VALUEZ__PID        PAGE_ZERO
#define BGSH_GYR_DATA_Z_LSB_VALUEZ__REG        BGSH_GYR_DATA_Z_LSB_ADDR

/* Gyro data Z-MSB register*/
#define BGSH_GYR_DATA_Z_MSB_VALUEZ__POS        0
#define BGSH_GYR_DATA_Z_MSB_VALUEZ__MSK        0xFF
#define BGSH_GYR_DATA_Z_MSB_VALUEZ__LEN        8
#define BGSH_GYR_DATA_Z_MSB_VALUEZ__PID        PAGE_ZERO
#define BGSH_GYR_DATA_Z_MSB_VALUEZ__REG        BGSH_GYR_DATA_Z_MSB_ADDR

/* Euler data HEADING-LSB register*/
#define BGSH_EUL_HEADING_LSB_VALUEH__POS        0
#define BGSH_EUL_HEADING_LSB_VALUEH__MSK        0xFF
#define BGSH_EUL_HEADING_LSB_VALUEH__LEN        8
#define BGSH_EUL_HEADING_LSB_VALUEH__PID        PAGE_ZERO
#define BGSH_EUL_HEADING_LSB_VALUEH__REG        BGSH_EUL_HEADING_LSB_ADDR

/* Euler data HEADING-MSB register*/
#define BGSH_EUL_HEADING_MSB_VALUEH__POS        0
#define BGSH_EUL_HEADING_MSB_VALUEH__MSK        0xFF
#define BGSH_EUL_HEADING_MSB_VALUEH__LEN        8
#define BGSH_EUL_HEADING_MSB_VALUEH__PID        PAGE_ZERO
#define BGSH_EUL_HEADING_MSB_VALUEH__REG        BGSH_EUL_HEADING_MSB_ADDR

/* Euler data ROLL-LSB register*/
#define BGSH_EUL_ROLL_LSB_VALUER__POS        0
#define BGSH_EUL_ROLL_LSB_VALUER__MSK        0xFF
#define BGSH_EUL_ROLL_LSB_VALUER__LEN        8
#define BGSH_EUL_ROLL_LSB_VALUER__PID        PAGE_ZERO
#define BGSH_EUL_ROLL_LSB_VALUER__REG        BGSH_EUL_ROLL_LSB_ADDR

/* Euler data ROLL-MSB register*/
#define BGSH_EUL_ROLL_MSB_VALUER__POS        0
#define BGSH_EUL_ROLL_MSB_VALUER__MSK        0xFF
#define BGSH_EUL_ROLL_MSB_VALUER__LEN        8
#define BGSH_EUL_ROLL_MSB_VALUER__PID        PAGE_ZERO
#define BGSH_EUL_ROLL_MSB_VALUER__REG        BGSH_EUL_ROLL_MSB_ADDR

/* Euler data PITCH-LSB register*/
#define BGSH_EUL_PITCH_LSB_VALUEP__POS        0
#define BGSH_EUL_PITCH_LSB_VALUEP__MSK        0xFF
#define BGSH_EUL_PITCH_LSB_VALUEP__LEN        8
#define BGSH_EUL_PITCH_LSB_VALUEP__PID        PAGE_ZERO
#define BGSH_EUL_PITCH_LSB_VALUEP__REG        BGSH_EUL_PITCH_LSB_ADDR

/* Euler data HEADING-MSB register*/
#define BGSH_EUL_PITCH_MSB_VALUEP__POS        0
#define BGSH_EUL_PITCH_MSB_VALUEP__MSK        0xFF
#define BGSH_EUL_PITCH_MSB_VALUEP__LEN        8
#define BGSH_EUL_PITCH_MSB_VALUEP__PID        PAGE_ZERO
#define BGSH_EUL_PITCH_MSB_VALUEP__REG        BGSH_EUL_PITCH_MSB_ADDR

/* Quaternion data W-LSB register*/
#define BGSH_QUA_DATA_W_LSB_VALUEW__POS        0
#define BGSH_QUA_DATA_W_LSB_VALUEW__MSK        0xFF
#define BGSH_QUA_DATA_W_LSB_VALUEW__LEN        8
#define BGSH_QUA_DATA_W_LSB_VALUEW__PID        PAGE_ZERO
#define BGSH_QUA_DATA_W_LSB_VALUEW__REG        BGSH_QUA_DATA_W_LSB_ADDR

/* Quaternion data W-MSB register*/
#define BGSH_QUA_DATA_W_MSB_VALUEW__POS        0
#define BGSH_QUA_DATA_W_MSB_VALUEW__MSK        0xFF
#define BGSH_QUA_DATA_W_MSB_VALUEW__LEN        8
#define BGSH_QUA_DATA_W_MSB_VALUEW__PID        PAGE_ZERO
#define BGSH_QUA_DATA_W_MSB_VALUEW__REG        BGSH_QUA_DATA_W_MSB_ADDR

/* Quaternion data X-LSB register*/
#define BGSH_QUA_DATA_X_LSB_VALUEX__POS        0
#define BGSH_QUA_DATA_X_LSB_VALUEX__MSK        0xFF
#define BGSH_QUA_DATA_X_LSB_VALUEX__LEN        8
#define BGSH_QUA_DATA_X_LSB_VALUEX__PID        PAGE_ZERO
#define BGSH_QUA_DATA_X_LSB_VALUEX__REG        BGSH_QUA_DATA_X_LSB_ADDR

/* Quaternion data X-MSB register*/
#define BGSH_QUA_DATA_X_MSB_VALUEX__POS        0
#define BGSH_QUA_DATA_X_MSB_VALUEX__MSK        0xFF
#define BGSH_QUA_DATA_X_MSB_VALUEX__LEN        8
#define BGSH_QUA_DATA_X_MSB_VALUEX__PID        PAGE_ZERO
#define BGSH_QUA_DATA_X_MSB_VALUEX__REG        BGSH_QUA_DATA_X_MSB_ADDR

/* Quaternion data Y-LSB register*/
#define BGSH_QUA_DATA_Y_LSB_VALUEY__POS        0
#define BGSH_QUA_DATA_Y_LSB_VALUEY__MSK        0xFF
#define BGSH_QUA_DATA_Y_LSB_VALUEY__LEN        8
#define BGSH_QUA_DATA_Y_LSB_VALUEY__PID        PAGE_ZERO
#define BGSH_QUA_DATA_Y_LSB_VALUEY__REG        BGSH_QUA_DATA_Y_LSB_ADDR

/* Quaternion data Y-MSB register*/
#define BGSH_QUA_DATA_Y_MSB_VALUEY__POS        0
#define BGSH_QUA_DATA_Y_MSB_VALUEY__MSK        0xFF
#define BGSH_QUA_DATA_Y_MSB_VALUEY__LEN        8
#define BGSH_QUA_DATA_Y_MSB_VALUEY__PID        PAGE_ZERO
#define BGSH_QUA_DATA_Y_MSB_VALUEY__REG        BGSH_QUA_DATA_Y_MSB_ADDR

/* Quaternion data Z-LSB register*/
#define BGSH_QUA_DATA_Z_LSB_VALUEZ__POS        0
#define BGSH_QUA_DATA_Z_LSB_VALUEZ__MSK        0xFF
#define BGSH_QUA_DATA_Z_LSB_VALUEZ__LEN        8
#define BGSH_QUA_DATA_Z_LSB_VALUEZ__PID        PAGE_ZERO
#define BGSH_QUA_DATA_Z_LSB_VALUEZ__REG        BGSH_QUA_DATA_Z_LSB_ADDR

/* Quaternion data Z-MSB register*/
#define BGSH_QUA_DATA_Z_MSB_VALUEZ__POS        0
#define BGSH_QUA_DATA_Z_MSB_VALUEZ__MSK        0xFF
#define BGSH_QUA_DATA_Z_MSB_VALUEZ__LEN        8
#define BGSH_QUA_DATA_Z_MSB_VALUEZ__PID        PAGE_ZERO
#define BGSH_QUA_DATA_Z_MSB_VALUEZ__REG        BGSH_QUA_DATA_Z_MSB_ADDR

/* Linear acceleration data X-LSB register*/
#define BGSH_LIA_DATA_X_LSB_VALUEX__POS        0
#define BGSH_LIA_DATA_X_LSB_VALUEX__MSK        0xFF
#define BGSH_LIA_DATA_X_LSB_VALUEX__LEN        8
#define BGSH_LIA_DATA_X_LSB_VALUEX__PID        PAGE_ZERO
#define BGSH_LIA_DATA_X_LSB_VALUEX__REG        BGSH_LIA_DATA_X_LSB_ADDR

/* Linear acceleration data X-MSB register*/
#define BGSH_LIA_DATA_X_MSB_VALUEX__POS        0
#define BGSH_LIA_DATA_X_MSB_VALUEX__MSK        0xFF
#define BGSH_LIA_DATA_X_MSB_VALUEX__LEN        8
#define BGSH_LIA_DATA_X_MSB_VALUEX__PID        PAGE_ZERO
#define BGSH_LIA_DATA_X_MSB_VALUEX__REG        BGSH_LIA_DATA_X_MSB_ADDR

/* Linear acceleration data Y-LSB register*/
#define BGSH_LIA_DATA_Y_LSB_VALUEY__POS        0
#define BGSH_LIA_DATA_Y_LSB_VALUEY__MSK        0xFF
#define BGSH_LIA_DATA_Y_LSB_VALUEY__LEN        8
#define BGSH_LIA_DATA_Y_LSB_VALUEY__PID        PAGE_ZERO
#define BGSH_LIA_DATA_Y_LSB_VALUEY__REG        BGSH_LIA_DATA_Y_LSB_ADDR

/* Linear acceleration data Y-MSB register*/
#define BGSH_LIA_DATA_Y_MSB_VALUEY__POS        0
#define BGSH_LIA_DATA_Y_MSB_VALUEY__MSK        0xFF
#define BGSH_LIA_DATA_Y_MSB_VALUEY__LEN        8
#define BGSH_LIA_DATA_Y_MSB_VALUEY__PID        PAGE_ZERO
#define BGSH_LIA_DATA_Y_MSB_VALUEY__REG        BGSH_LIA_DATA_Y_MSB_ADDR

/* Linear acceleration data Z-LSB register*/
#define BGSH_LIA_DATA_Z_LSB_VALUEZ__POS        0
#define BGSH_LIA_DATA_Z_LSB_VALUEZ__MSK        0xFF
#define BGSH_LIA_DATA_Z_LSB_VALUEZ__LEN        8
#define BGSH_LIA_DATA_Z_LSB_VALUEZ__PID        PAGE_ZERO
#define BGSH_LIA_DATA_Z_LSB_VALUEZ__REG        BGSH_LIA_DATA_Z_LSB_ADDR

/* Linear acceleration data Z-MSB register*/
#define BGSH_LIA_DATA_Z_MSB_VALUEZ__POS        0
#define BGSH_LIA_DATA_Z_MSB_VALUEZ__MSK        0xFF
#define BGSH_LIA_DATA_Z_MSB_VALUEZ__LEN        8
#define BGSH_LIA_DATA_Z_MSB_VALUEZ__PID        PAGE_ZERO
#define BGSH_LIA_DATA_Z_MSB_VALUEZ__REG        BGSH_LIA_DATA_Z_MSB_ADDR

/* Gravity data X-LSB register*/
#define BGSH_GRV_DATA_X_LSB_VALUEX__POS        0
#define BGSH_GRV_DATA_X_LSB_VALUEX__MSK        0xFF
#define BGSH_GRV_DATA_X_LSB_VALUEX__LEN        8
#define BGSH_GRV_DATA_X_LSB_VALUEX__PID        PAGE_ZERO
#define BGSH_GRV_DATA_X_LSB_VALUEX__REG        BGSH_GRV_DATA_X_LSB_ADDR

/* Gravity data X-MSB register*/
#define BGSH_GRV_DATA_X_MSB_VALUEX__POS        0
#define BGSH_GRV_DATA_X_MSB_VALUEX__MSK        0xFF
#define BGSH_GRV_DATA_X_MSB_VALUEX__LEN        8
#define BGSH_GRV_DATA_X_MSB_VALUEX__PID        PAGE_ZERO
#define BGSH_GRV_DATA_X_MSB_VALUEX__REG        BGSH_GRV_DATA_X_MSB_ADDR

/* Gravity data Y-LSB register*/
#define BGSH_GRV_DATA_Y_LSB_VALUEY__POS        0
#define BGSH_GRV_DATA_Y_LSB_VALUEY__MSK        0xFF
#define BGSH_GRV_DATA_Y_LSB_VALUEY__LEN        8
#define BGSH_GRV_DATA_Y_LSB_VALUEY__PID        PAGE_ZERO
#define BGSH_GRV_DATA_Y_LSB_VALUEY__REG        BGSH_GRV_DATA_Y_LSB_ADDR

/* Gravity data Y-MSB register*/
#define BGSH_GRV_DATA_Y_MSB_VALUEY__POS        0
#define BGSH_GRV_DATA_Y_MSB_VALUEY__MSK        0xFF
#define BGSH_GRV_DATA_Y_MSB_VALUEY__LEN        8
#define BGSH_GRV_DATA_Y_MSB_VALUEY__PID        PAGE_ZERO
#define BGSH_GRV_DATA_Y_MSB_VALUEY__REG        BGSH_GRV_DATA_Y_MSB_ADDR

/* Gravity data Z-LSB register*/
#define BGSH_GRV_DATA_Z_LSB_VALUEZ__POS        0
#define BGSH_GRV_DATA_Z_LSB_VALUEZ__MSK        0xFF
#define BGSH_GRV_DATA_Z_LSB_VALUEZ__LEN        8
#define BGSH_GRV_DATA_Z_LSB_VALUEZ__PID        PAGE_ZERO
#define BGSH_GRV_DATA_Z_LSB_VALUEZ__REG        BGSH_GRV_DATA_Z_LSB_ADDR

/* Gravity data Z-MSB register*/
#define BGSH_GRV_DATA_Z_MSB_VALUEZ__POS        0
#define BGSH_GRV_DATA_Z_MSB_VALUEZ__MSK        0xFF
#define BGSH_GRV_DATA_Z_MSB_VALUEZ__LEN        8
#define BGSH_GRV_DATA_Z_MSB_VALUEZ__PID        PAGE_ZERO
#define BGSH_GRV_DATA_Z_MSB_VALUEZ__REG        BGSH_GRV_DATA_Z_MSB_ADDR

/* Temperature register*/
#define BGSH_TEMP__POS             0
#define BGSH_TEMP__MSK             0xFF
#define BGSH_TEMP__LEN             8
#define BGSH_TEMP__PID             PAGE_ZERO
#define BGSH_TEMP__REG             BGSH_TEMP_ADDR

/*Mag_Calib status register*/
#define BGSH_MAG_CALIB_STAT__POS        0
#define BGSH_MAG_CALIB_STAT__MSK        0X03
#define BGSH_MAG_CALIB_STAT__LEN        2
#define BGSH_MAG_CALIB_STAT__PID        PAGE_ZERO
#define BGSH_MAG_CALIB_STAT__REG        BGSH_CALIB_STAT_ADDR

/*Acc_Calib status register*/
#define BGSH_ACC_CALIB_STAT__POS        2
#define BGSH_ACC_CALIB_STAT__MSK        0X0C
#define BGSH_ACC_CALIB_STAT__LEN        2
#define BGSH_ACC_CALIB_STAT__PID        PAGE_ZERO
#define BGSH_ACC_CALIB_STAT__REG        BGSH_CALIB_STAT_ADDR

/*Gyro_Calib status register*/
#define BGSH_GYR_CALIB_STAT__POS        4
#define BGSH_GYR_CALIB_STAT__MSK        0X30
#define BGSH_GYR_CALIB_STAT__LEN        2
#define BGSH_GYR_CALIB_STAT__PID        PAGE_ZERO
#define BGSH_GYR_CALIB_STAT__REG        BGSH_CALIB_STAT_ADDR

/*Sys_Calib status register*/
#define BGSH_SYS_CALIB_STAT__POS        6
#define BGSH_SYS_CALIB_STAT__MSK        0XC0
#define BGSH_SYS_CALIB_STAT__LEN        2
#define BGSH_SYS_CALIB_STAT__PID        PAGE_ZERO
#define BGSH_SYS_CALIB_STAT__REG        BGSH_CALIB_STAT_ADDR

/*ST_ACCEL register*/
#define BGSH_ST_ACC__POS        0
#define BGSH_ST_ACC__MSK        0X01
#define BGSH_ST_ACC__LEN        1
#define BGSH_ST_ACC__PID        PAGE_ZERO
#define BGSH_ST_ACC__REG        BGSH_ST_RESULT_ADDR

/*ST_MAG register*/
#define BGSH_ST_MAG__POS        1
#define BGSH_ST_MAG__MSK        0X02
#define BGSH_ST_MAG__LEN        1
#define BGSH_ST_MAG__PID        PAGE_ZERO
#define BGSH_ST_MAG__REG        BGSH_ST_RESULT_ADDR

/*ST_GYRO register*/
#define BGSH_ST_GYR__POS        2
#define BGSH_ST_GYR__MSK        0X04
#define BGSH_ST_GYR__LEN        1
#define BGSH_ST_GYR__PID        PAGE_ZERO
#define BGSH_ST_GYR__REG        BGSH_ST_RESULT_ADDR

/*ST_MCU register*/
#define BGSH_ST_MCU__POS        3
#define BGSH_ST_MCU__MSK        0X08
#define BGSH_ST_MCU__LEN        1
#define BGSH_ST_MCU__PID        PAGE_ZERO
#define BGSH_ST_MCU__REG        BGSH_ST_RESULT_ADDR

/*Interrupt status registers*/
#define BGSH_INT_STAT_GYRO_AM__POS    2
#define BGSH_INT_STAT_GYRO_AM__MSK    0X04
#define BGSH_INT_STAT_GYRO_AM__LEN    1
#define BGSH_INT_STAT_GYRO_AM__PID    PAGE_ZERO
#define BGSH_INT_STAT_GYRO_AM__REG    BGSH_INT_STA_ADDR

#define BGSH_INT_STAT_GYRO_HIGH_RATE__POS    3
#define BGSH_INT_STAT_GYRO_HIGH_RATE__MSK    0X08
#define BGSH_INT_STAT_GYRO_HIGH_RATE__LEN    1
#define BGSH_INT_STAT_GYRO_HIGH_RATE__PID    PAGE_ZERO
#define BGSH_INT_STAT_GYRO_HIGH_RATE__REG    BGSH_INT_STA_ADDR

#define BGSH_INT_STAT_ACC_HIGH_G__POS    5
#define BGSH_INT_STAT_ACC_HIGH_G__MSK    0X20
#define BGSH_INT_STAT_ACC_HIGH_G__LEN    1
#define BGSH_INT_STAT_ACC_HIGH_G__PID    PAGE_ZERO
#define BGSH_INT_STAT_ACC_HIGH_G__REG    BGSH_INT_STA_ADDR

#define BGSH_INT_STAT_ACC_AM__POS    6
#define BGSH_INT_STAT_ACC_AM__MSK    0X40
#define BGSH_INT_STAT_ACC_AM__LEN    1
#define BGSH_INT_STAT_ACC_AM__PID    PAGE_ZERO
#define BGSH_INT_STAT_ACC_AM__REG    BGSH_INT_STA_ADDR

#define BGSH_INT_STAT_ACC_NM__POS    7
#define BGSH_INT_STAT_ACC_NM__MSK    0X80
#define BGSH_INT_STAT_ACC_NM__LEN    1
#define BGSH_INT_STAT_ACC_NM__PID    PAGE_ZERO
#define BGSH_INT_STAT_ACC_NM__REG    BGSH_INT_STA_ADDR

/* System registers*/
#define BGSH_SYSTEM_STATUS_CODE__POS    0
#define BGSH_SYSTEM_STATUS_CODE__MSK    0XFF
#define BGSH_SYSTEM_STATUS_CODE__LEN    8
#define BGSH_SYSTEM_STATUS_CODE__PID    PAGE_ZERO
#define BGSH_SYSTEM_STATUS_CODE__REG    BGSH_SYS_STATUS_ADDR

#define BGSH_SYSTEM_ERROR_CODE__POS    0
#define BGSH_SYSTEM_ERROR_CODE__MSK    0XFF
#define BGSH_SYSTEM_ERROR_CODE__LEN    8
#define BGSH_SYSTEM_ERROR_CODE__PID    PAGE_ZERO
#define BGSH_SYSTEM_ERROR_CODE__REG    BGSH_SYS_ERR_ADDR

/* Accel_Unit register*/
#define BGSH_ACC_UNIT__POS             0
#define BGSH_ACC_UNIT__MSK             0X01
#define BGSH_ACC_UNIT__LEN             1
#define BGSH_ACC_UNIT__PID             PAGE_ZERO
#define BGSH_ACC_UNIT__REG             BGSH_UNIT_SEL_ADDR

/* Gyro_Unit register*/
#define BGSH_GYR_UNIT__POS             1
#define BGSH_GYR_UNIT__MSK             0X02
#define BGSH_GYR_UNIT__LEN             1
#define BGSH_GYR_UNIT__PID             PAGE_ZERO
#define BGSH_GYR_UNIT__REG             BGSH_UNIT_SEL_ADDR

/* Euler_Unit register*/
#define BGSH_EUL_UNIT__POS             2
#define BGSH_EUL_UNIT__MSK             0X04
#define BGSH_EUL_UNIT__LEN             1
#define BGSH_EUL_UNIT__PID             PAGE_ZERO
#define BGSH_EUL_UNIT__REG             BGSH_UNIT_SEL_ADDR

/* Tilt_Unit register*/
#define BGSH_TILT_UNIT__POS             3
#define BGSH_TILT_UNIT__MSK             0X08
#define BGSH_TILT_UNIT__LEN             1
#define BGSH_TILT_UNIT__PID             PAGE_ZERO
#define BGSH_TILT_UNIT__REG             BGSH_UNIT_SEL_ADDR

/* Temperature_Unit register*/
#define BGSH_TEMP_UNIT__POS             4
#define BGSH_TEMP_UNIT__MSK             0X10
#define BGSH_TEMP_UNIT__LEN             1
#define BGSH_TEMP_UNIT__PID             PAGE_ZERO
#define BGSH_TEMP_UNIT__REG             BGSH_UNIT_SEL_ADDR

/* ORI android-windows register*/
#define BGSH_DATA_OUTPUT_FORMAT__POS             7
#define BGSH_DATA_OUTPUT_FORMAT__MSK             0X80
#define BGSH_DATA_OUTPUT_FORMAT__LEN             1
#define BGSH_DATA_OUTPUT_FORMAT__PID             PAGE_ZERO
#define BGSH_DATA_OUTPUT_FORMAT__REG             BGSH_UNIT_SEL_ADDR

/*Data Select register*/
/* Accel data select*/
#define BGSH_DATA_SEL_ACC__POS    0
#define BGSH_DATA_SEL_ACC__MSK    0X01
#define BGSH_DATA_SEL_ACC__LEN    1
#define BGSH_DATA_SEL_ACC__PID    PAGE_ZERO
#define BGSH_DATA_SEL_ACC__REG    BGSH_DATA_SEL_ADDR

/* Mag data select*/
#define BGSH_DATA_SEL_MAG__POS    1
#define BGSH_DATA_SEL_MAG__MSK    0X02
#define BGSH_DATA_SEL_MAG__LEN    1
#define BGSH_DATA_SEL_MAG__PID    PAGE_ZERO
#define BGSH_DATA_SEL_MAG__REG    BGSH_DATA_SEL_ADDR

/* Gyro data select*/
#define BGSH_DATA_SEL_GYR__POS    2
#define BGSH_DATA_SEL_GYR__MSK    0X04
#define BGSH_DATA_SEL_GYR__LEN    1
#define BGSH_DATA_SEL_GYR__PID    PAGE_ZERO
#define BGSH_DATA_SEL_GYR__REG    BGSH_DATA_SEL_ADDR

/* Euler data select*/
#define BGSH_DATA_SEL_EUL__POS    3
#define BGSH_DATA_SEL_EUL__MSK    0X08
#define BGSH_DATA_SEL_EUL__LEN    1
#define BGSH_DATA_SEL_EUL__PID    PAGE_ZERO
#define BGSH_DATA_SEL_EUL__REG    BGSH_DATA_SEL_ADDR

/* Quaternion data select*/
#define BGSH_DATA_SEL_QUA__POS    4
#define BGSH_DATA_SEL_QUA__MSK    0X10
#define BGSH_DATA_SEL_QUA__LEN    1
#define BGSH_DATA_SEL_QUA__PID    PAGE_ZERO
#define BGSH_DATA_SEL_QUA__REG    BGSH_DATA_SEL_ADDR

/* Linear Accel data select*/
#define BGSH_DATA_SEL_LINEAR_ACC__POS    5
#define BGSH_DATA_SEL_LINEAR_ACC__MSK    0X20
#define BGSH_DATA_SEL_LINEAR_ACC__LEN    1
#define BGSH_DATA_SEL_LINEAR_ACC__PID    PAGE_ZERO
#define BGSH_DATA_SEL_LINEAR_ACC__REG    BGSH_DATA_SEL_ADDR

/* Gravity data select*/
#define BGSH_DATA_SEL_GRV__POS    6
#define BGSH_DATA_SEL_GRV__MSK    0X40
#define BGSH_DATA_SEL_GRV__LEN    1
#define BGSH_DATA_SEL_GRV__PID    PAGE_ZERO
#define BGSH_DATA_SEL_GRV__REG    BGSH_DATA_SEL_ADDR

/* Temperature data select*/
#define BGSH_DATA_SEL_TEMP__POS    7
#define BGSH_DATA_SEL_TEMP__MSK    0X80
#define BGSH_DATA_SEL_TEMP__LEN    1
#define BGSH_DATA_SEL_TEMP__PID    PAGE_ZERO
#define BGSH_DATA_SEL_TEMP__REG    BGSH_DATA_SEL_ADDR

/*Operation Mode data register*/
#define BGSH_OPERATION_MODE__POS    0
#define BGSH_OPERATION_MODE__MSK    0X0F
#define BGSH_OPERATION_MODE__LEN    4
#define BGSH_OPERATION_MODE__PID    PAGE_ZERO
#define BGSH_OPERATION_MODE__REG    BGSH_OPR_MODE_ADDR

/* output data rate register*/
#define BGSH_OUTPUT_DATA_RATE__POS    4
#define BGSH_OUTPUT_DATA_RATE__MSK    0X70
#define BGSH_OUTPUT_DATA_RATE__LEN    3
#define BGSH_OUTPUT_DATA_RATE__PID    PAGE_ZERO
#define BGSH_OUTPUT_DATA_RATE__REG    BGSH_OPR_MODE_ADDR

/* Power Mode register*/
#define BGSH_POWER_MODE__POS             0
#define BGSH_POWER_MODE__MSK             0X03
#define BGSH_POWER_MODE__LEN             2
#define BGSH_POWER_MODE__PID             PAGE_ZERO
#define BGSH_POWER_MODE__REG             BGSH_PWR_MODE_ADDR

/*Self Test register*/
#define BGSH_SELF_TEST__POS    0
#define BGSH_SELF_TEST__MSK    0X01
#define BGSH_SELF_TEST__LEN    1
#define BGSH_SELF_TEST__PID    PAGE_ZERO
#define BGSH_SELF_TEST__REG    BGSH_SYS_TRIGGER_ADDR

/* RST_SYS register*/
#define BGSH_RST_SYS__POS             5
#define BGSH_RST_SYS__MSK             0X20
#define BGSH_RST_SYS__LEN             1
#define BGSH_RST_SYS__PID             PAGE_ZERO
#define BGSH_RST_SYS__REG             BGSH_SYS_TRIGGER_ADDR

/* RST_INT register*/
#define BGSH_RST_INT__POS             6
#define BGSH_RST_INT__MSK             0X40
#define BGSH_RST_INT__LEN             1
#define BGSH_RST_INT__PID             PAGE_ZERO
#define BGSH_RST_INT__REG             BGSH_SYS_TRIGGER_ADDR

/* Temp source register*/
#define BGSH_TEMP_SOURCE__POS    0
#define BGSH_TEMP_SOURCE__MSK    0X03
#define BGSH_TEMP_SOURCE__LEN    2
#define BGSH_TEMP_SOURCE__PID    PAGE_ZERO
#define BGSH_TEMP_SOURCE__REG    BGSH_TEMP_SOURCE_ADDR

/* Axis remap value register*/
#define BGSH_REMAP_AXIS_VALUE__POS    0
#define BGSH_REMAP_AXIS_VALUE__MSK    0X3F
#define BGSH_REMAP_AXIS_VALUE__LEN    6
#define BGSH_REMAP_AXIS_VALUE__PID    PAGE_ZERO
#define BGSH_REMAP_AXIS_VALUE__REG    BGSH_AXIS_MAP_CONFIG_ADDR

/* Axis sign value register*/
#define BGSH_REMAP_Z_SIGN__POS    0
#define BGSH_REMAP_Z_SIGN__MSK    0X01
#define BGSH_REMAP_Z_SIGN__LEN    1
#define BGSH_REMAP_Z_SIGN__PID    PAGE_ZERO
#define BGSH_REMAP_Z_SIGN__REG    BGSH_AXIS_MAP_SIGN_ADDR

#define BGSH_REMAP_Y_SIGN__POS    1
#define BGSH_REMAP_Y_SIGN__MSK    0X02
#define BGSH_REMAP_Y_SIGN__LEN    1
#define BGSH_REMAP_Y_SIGN__PID    PAGE_ZERO
#define BGSH_REMAP_Y_SIGN__REG    BGSH_AXIS_MAP_SIGN_ADDR

#define BGSH_REMAP_X_SIGN__POS    2
#define BGSH_REMAP_X_SIGN__MSK    0X04
#define BGSH_REMAP_X_SIGN__LEN    1
#define BGSH_REMAP_X_SIGN__PID    PAGE_ZERO
#define BGSH_REMAP_X_SIGN__REG    BGSH_AXIS_MAP_SIGN_ADDR

/* Soft Iron Calibration matrix register*/
#define BGSH_SIC_MATRIX_0_LSB__POS    0
#define BGSH_SIC_MATRIX_0_LSB__MSK    0XFF
#define BGSH_SIC_MATRIX_0_LSB__LEN    8
#define BGSH_SIC_MATRIX_0_LSB__PID    PAGE_ZERO
#define BGSH_SIC_MATRIX_0_LSB__REG    BGSH_SIC_MATRIX_0_LSB_ADDR

#define BGSH_SIC_MATRIX_0_MSB__POS    0
#define BGSH_SIC_MATRIX_0_MSB__MSK    0XFF
#define BGSH_SIC_MATRIX_0_MSB__LEN    8
#define BGSH_SIC_MATRIX_0_MSB__PID    PAGE_ZERO
#define BGSH_SIC_MATRIX_0_MSB__REG    BGSH_SIC_MATRIX_0_MSB_ADDR

#define BGSH_SIC_MATRIX_1_LSB__POS    0
#define BGSH_SIC_MATRIX_1_LSB__MSK    0XFF
#define BGSH_SIC_MATRIX_1_LSB__LEN    8
#define BGSH_SIC_MATRIX_1_LSB__PID    PAGE_ZERO
#define BGSH_SIC_MATRIX_1_LSB__REG    BGSH_SIC_MATRIX_1_LSB_ADDR

#define BGSH_SIC_MATRIX_1_MSB__POS    0
#define BGSH_SIC_MATRIX_1_MSB__MSK    0XFF
#define BGSH_SIC_MATRIX_1_MSB__LEN    8
#define BGSH_SIC_MATRIX_1_MSB__PID    PAGE_ZERO
#define BGSH_SIC_MATRIX_1_MSB__REG    BGSH_SIC_MATRIX_1_MSB_ADDR

#define BGSH_SIC_MATRIX_2_LSB__POS    0
#define BGSH_SIC_MATRIX_2_LSB__MSK    0XFF
#define BGSH_SIC_MATRIX_2_LSB__LEN    8
#define BGSH_SIC_MATRIX_2_LSB__PID    PAGE_ZERO
#define BGSH_SIC_MATRIX_2_LSB__REG    BGSH_SIC_MATRIX_2_LSB_ADDR

#define BGSH_SIC_MATRIX_2_MSB__POS    0
#define BGSH_SIC_MATRIX_2_MSB__MSK    0XFF
#define BGSH_SIC_MATRIX_2_MSB__LEN    8
#define BGSH_SIC_MATRIX_2_MSB__PID    PAGE_ZERO
#define BGSH_SIC_MATRIX_2_MSB__REG    BGSH_SIC_MATRIX_2_MSB_ADDR

#define BGSH_SIC_MATRIX_3_LSB__POS    0
#define BGSH_SIC_MATRIX_3_LSB__MSK    0XFF
#define BGSH_SIC_MATRIX_3_LSB__LEN    8
#define BGSH_SIC_MATRIX_3_LSB__PID    PAGE_ZERO
#define BGSH_SIC_MATRIX_3_LSB__REG    BGSH_SIC_MATRIX_3_LSB_ADDR

#define BGSH_SIC_MATRIX_3_MSB__POS    0
#define BGSH_SIC_MATRIX_3_MSB__MSK    0XFF
#define BGSH_SIC_MATRIX_3_MSB__LEN    8
#define BGSH_SIC_MATRIX_3_MSB__PID    PAGE_ZERO
#define BGSH_SIC_MATRIX_3_MSB__REG    BGSH_SIC_MATRIX_3_MSB_ADDR

#define BGSH_SIC_MATRIX_4_LSB__POS    0
#define BGSH_SIC_MATRIX_4_LSB__MSK    0XFF
#define BGSH_SIC_MATRIX_4_LSB__LEN    8
#define BGSH_SIC_MATRIX_4_LSB__PID    PAGE_ZERO
#define BGSH_SIC_MATRIX_4_LSB__REG    BGSH_SIC_MATRIX_4_LSB_ADDR

#define BGSH_SIC_MATRIX_4_MSB__POS        0
#define BGSH_SIC_MATRIX_4_MSB__MSK        0XFF
#define BGSH_SIC_MATRIX_4_MSB__LEN        8
#define BGSH_SIC_MATRIX_4_MSB__PID        PAGE_ZERO
#define BGSH_SIC_MATRIX_4_MSB__REG        BGSH_SIC_MATRIX_4_MSB_ADDR

#define BGSH_SIC_MATRIX_5_LSB__POS        0
#define BGSH_SIC_MATRIX_5_LSB__MSK        0XFF
#define BGSH_SIC_MATRIX_5_LSB__LEN        8
#define BGSH_SIC_MATRIX_5_LSB__PID        PAGE_ZERO
#define BGSH_SIC_MATRIX_5_LSB__REG        BGSH_SIC_MATRIX_5_LSB_ADDR

#define BGSH_SIC_MATRIX_5_MSB__POS        0
#define BGSH_SIC_MATRIX_5_MSB__MSK        0XFF
#define BGSH_SIC_MATRIX_5_MSB__LEN        8
#define BGSH_SIC_MATRIX_5_MSB__PID        PAGE_ZERO
#define BGSH_SIC_MATRIX_5_MSB__REG        BGSH_SIC_MATRIX_5_MSB_ADDR

#define BGSH_SIC_MATRIX_6_LSB__POS    0
#define BGSH_SIC_MATRIX_6_LSB__MSK    0XFF
#define BGSH_SIC_MATRIX_6_LSB__LEN    8
#define BGSH_SIC_MATRIX_6_LSB__PID    PAGE_ZERO
#define BGSH_SIC_MATRIX_6_LSB__REG    BGSH_SIC_MATRIX_6_LSB_ADDR

#define BGSH_SIC_MATRIX_6_MSB__POS    0
#define BGSH_SIC_MATRIX_6_MSB__MSK    0XFF
#define BGSH_SIC_MATRIX_6_MSB__LEN    8
#define BGSH_SIC_MATRIX_6_MSB__PID    PAGE_ZERO
#define BGSH_SIC_MATRIX_6_MSB__REG    BGSH_SIC_MATRIX_6_MSB_ADDR

#define BGSH_SIC_MATRIX_7_LSB__POS    0
#define BGSH_SIC_MATRIX_7_LSB__MSK    0XFF
#define BGSH_SIC_MATRIX_7_LSB__LEN    8
#define BGSH_SIC_MATRIX_7_LSB__PID    PAGE_ZERO
#define BGSH_SIC_MATRIX_7_LSB__REG    BGSH_SIC_MATRIX_7_LSB_ADDR

#define BGSH_SIC_MATRIX_7_MSB__POS    0
#define BGSH_SIC_MATRIX_7_MSB__MSK    0XFF
#define BGSH_SIC_MATRIX_7_MSB__LEN    8
#define BGSH_SIC_MATRIX_7_MSB__PID    PAGE_ZERO
#define BGSH_SIC_MATRIX_7_MSB__REG    BGSH_SIC_MATRIX_7_MSB_ADDR

#define BGSH_SIC_MATRIX_8_LSB__POS    0
#define BGSH_SIC_MATRIX_8_LSB__MSK    0XFF
#define BGSH_SIC_MATRIX_8_LSB__LEN    8
#define BGSH_SIC_MATRIX_8_LSB__PID    PAGE_ZERO
#define BGSH_SIC_MATRIX_8_LSB__REG    BGSH_SIC_MATRIX_8_LSB_ADDR

#define BGSH_SIC_MATRIX_8_MSB__POS    0
#define BGSH_SIC_MATRIX_8_MSB__MSK    0XFF
#define BGSH_SIC_MATRIX_8_MSB__LEN    8
#define BGSH_SIC_MATRIX_8_MSB__PID    PAGE_ZERO
#define BGSH_SIC_MATRIX_8_MSB__REG    BGSH_SIC_MATRIX_8_MSB_ADDR

/*Accel Offset registers*/
#define BGSH_ACC_OFFSET_X_LSB__POS    0
#define BGSH_ACC_OFFSET_X_LSB__MSK    0XFF
#define BGSH_ACC_OFFSET_X_LSB__LEN    8
#define BGSH_ACC_OFFSET_X_LSB__PID    PAGE_ZERO
#define BGSH_ACC_OFFSET_X_LSB__REG    ACC_OFFSET_X_LSB_ADDR

#define BGSH_ACC_OFFSET_X_MSB__POS    0
#define BGSH_ACC_OFFSET_X_MSB__MSK    0XFF
#define BGSH_ACC_OFFSET_X_MSB__LEN    8
#define BGSH_ACC_OFFSET_X_MSB__PID    PAGE_ZERO
#define BGSH_ACC_OFFSET_X_MSB__REG    ACC_OFFSET_X_MSB_ADDR

#define BGSH_ACC_OFFSET_Y_LSB__POS    0
#define BGSH_ACC_OFFSET_Y_LSB__MSK    0XFF
#define BGSH_ACC_OFFSET_Y_LSB__LEN    8
#define BGSH_ACC_OFFSET_Y_LSB__PID    PAGE_ZERO
#define BGSH_ACC_OFFSET_Y_LSB__REG    ACC_OFFSET_Y_LSB_ADDR

#define BGSH_ACC_OFFSET_Y_MSB__POS    0
#define BGSH_ACC_OFFSET_Y_MSB__MSK    0XFF
#define BGSH_ACC_OFFSET_Y_MSB__LEN    8
#define BGSH_ACC_OFFSET_Y_MSB__PID    PAGE_ZERO
#define BGSH_ACC_OFFSET_Y_MSB__REG    ACC_OFFSET_Y_MSB_ADDR

#define BGSH_ACC_OFFSET_Z_LSB__POS    0
#define BGSH_ACC_OFFSET_Z_LSB__MSK    0XFF
#define BGSH_ACC_OFFSET_Z_LSB__LEN    8
#define BGSH_ACC_OFFSET_Z_LSB__PID    PAGE_ZERO
#define BGSH_ACC_OFFSET_Z_LSB__REG    ACC_OFFSET_Z_LSB_ADDR

#define BGSH_ACC_OFFSET_Z_MSB__POS    0
#define BGSH_ACC_OFFSET_Z_MSB__MSK    0XFF
#define BGSH_ACC_OFFSET_Z_MSB__LEN    8
#define BGSH_ACC_OFFSET_Z_MSB__PID    PAGE_ZERO
#define BGSH_ACC_OFFSET_Z_MSB__REG    ACC_OFFSET_Z_MSB_ADDR

/*Mag Offset registers*/
#define BGSH_MAG_OFFSET_X_LSB__POS    0
#define BGSH_MAG_OFFSET_X_LSB__MSK    0XFF
#define BGSH_MAG_OFFSET_X_LSB__LEN    8
#define BGSH_MAG_OFFSET_X_LSB__PID    PAGE_ZERO
#define BGSH_MAG_OFFSET_X_LSB__REG    MAG_OFFSET_X_LSB_ADDR

#define BGSH_MAG_OFFSET_X_MSB__POS    0
#define BGSH_MAG_OFFSET_X_MSB__MSK    0XFF
#define BGSH_MAG_OFFSET_X_MSB__LEN    8
#define BGSH_MAG_OFFSET_X_MSB__PID    PAGE_ZERO
#define BGSH_MAG_OFFSET_X_MSB__REG    MAG_OFFSET_X_MSB_ADDR

#define BGSH_MAG_OFFSET_Y_LSB__POS    0
#define BGSH_MAG_OFFSET_Y_LSB__MSK    0XFF
#define BGSH_MAG_OFFSET_Y_LSB__LEN    8
#define BGSH_MAG_OFFSET_Y_LSB__PID    PAGE_ZERO
#define BGSH_MAG_OFFSET_Y_LSB__REG    MAG_OFFSET_Y_LSB_ADDR

#define BGSH_MAG_OFFSET_Y_MSB__POS    0
#define BGSH_MAG_OFFSET_Y_MSB__MSK    0XFF
#define BGSH_MAG_OFFSET_Y_MSB__LEN    8
#define BGSH_MAG_OFFSET_Y_MSB__PID    PAGE_ZERO
#define BGSH_MAG_OFFSET_Y_MSB__REG    MAG_OFFSET_Y_MSB_ADDR

#define BGSH_MAG_OFFSET_Z_LSB__POS    0
#define BGSH_MAG_OFFSET_Z_LSB__MSK    0XFF
#define BGSH_MAG_OFFSET_Z_LSB__LEN    8
#define BGSH_MAG_OFFSET_Z_LSB__PID    PAGE_ZERO
#define BGSH_MAG_OFFSET_Z_LSB__REG    MAG_OFFSET_Z_LSB_ADDR

#define BGSH_MAG_OFFSET_Z_MSB__POS    0
#define BGSH_MAG_OFFSET_Z_MSB__MSK    0XFF
#define BGSH_MAG_OFFSET_Z_MSB__LEN    8
#define BGSH_MAG_OFFSET_Z_MSB__PID    PAGE_ZERO
#define BGSH_MAG_OFFSET_Z_MSB__REG    MAG_OFFSET_Z_MSB_ADDR

/* Gyro Offset registers*/
#define BGSH_GYR_OFFSET_X_LSB__POS    0
#define BGSH_GYR_OFFSET_X_LSB__MSK    0XFF
#define BGSH_GYR_OFFSET_X_LSB__LEN    8
#define BGSH_GYR_OFFSET_X_LSB__PID    PAGE_ZERO
#define BGSH_GYR_OFFSET_X_LSB__REG    GYRO_OFFSET_X_LSB_ADDR

#define BGSH_GYR_OFFSET_X_MSB__POS    0
#define BGSH_GYR_OFFSET_X_MSB__MSK    0XFF
#define BGSH_GYR_OFFSET_X_MSB__LEN    8
#define BGSH_GYR_OFFSET_X_MSB__PID    PAGE_ZERO
#define BGSH_GYR_OFFSET_X_MSB__REG    GYRO_OFFSET_X_MSB_ADDR

#define BGSH_GYR_OFFSET_Y_LSB__POS    0
#define BGSH_GYR_OFFSET_Y_LSB__MSK    0XFF
#define BGSH_GYR_OFFSET_Y_LSB__LEN    8
#define BGSH_GYR_OFFSET_Y_LSB__PID    PAGE_ZERO
#define BGSH_GYR_OFFSET_Y_LSB__REG    GYRO_OFFSET_Y_LSB_ADDR

#define BGSH_GYR_OFFSET_Y_MSB__POS    0
#define BGSH_GYR_OFFSET_Y_MSB__MSK    0XFF
#define BGSH_GYR_OFFSET_Y_MSB__LEN    8
#define BGSH_GYR_OFFSET_Y_MSB__PID    PAGE_ZERO
#define BGSH_GYR_OFFSET_Y_MSB__REG    GYRO_OFFSET_Y_MSB_ADDR

#define BGSH_GYR_OFFSET_Z_LSB__POS    0
#define BGSH_GYR_OFFSET_Z_LSB__MSK    0XFF
#define BGSH_GYR_OFFSET_Z_LSB__LEN    8
#define BGSH_GYR_OFFSET_Z_LSB__PID    PAGE_ZERO
#define BGSH_GYR_OFFSET_Z_LSB__REG    GYRO_OFFSET_Z_LSB_ADDR

#define BGSH_GYR_OFFSET_Z_MSB__POS    0
#define BGSH_GYR_OFFSET_Z_MSB__MSK    0XFF
#define BGSH_GYR_OFFSET_Z_MSB__LEN    8
#define BGSH_GYR_OFFSET_Z_MSB__PID    PAGE_ZERO
#define BGSH_GYR_OFFSET_Z_MSB__REG    GYRO_OFFSET_Z_MSB_ADDR

/* Bootloader mode */
#define BGSH_SENOSR_BL_MODE__POS    7
#define BGSH_SENOSR_BL_MODE__MSK    0x80
#define BGSH_SENOSR_BL_MODE__LEN    1
#define BGSH_SENOSR_BL_MODE__PID    PAGE_ZERO
#define BGSH_SENOSR_BL_MODE__REG    BGSH_BOOT_CONFIG_ADDR

/* PAGE0 DATA REGISTERS DEFINITION END*/

/* PAGE1 DATA REGISTERS DEFINITION START*/
/* Configuration registers*/
/* Accel range configuration register*/
#define BGSH_ACC_RANGE__POS    0
#define BGSH_ACC_RANGE__MSK    0X03
#define BGSH_ACC_RANGE__LEN    2
#define BGSH_ACC_RANGE__PID    PAGE_ONE
#define BGSH_ACC_RANGE__REG    ACC_CONFIG_ADDR

/* Accel bandwidth configuration register*/
#define BGSH_ACC_BW__POS        2
#define BGSH_ACC_BW__MSK        0X1C
#define BGSH_ACC_BW__LEN        3
#define BGSH_ACC_BW__PID        PAGE_ONE
#define BGSH_ACC_BW__REG        ACC_CONFIG_ADDR

/* Accel power mode configuration register*/
#define BGSH_ACC_PWR_MODE__POS    5
#define BGSH_ACC_PWR_MODE__MSK    0XE0
#define BGSH_ACC_PWR_MODE__LEN    3
#define BGSH_ACC_PWR_MODE__PID    PAGE_ONE
#define BGSH_ACC_PWR_MODE__REG    ACC_CONFIG_ADDR

/* Mag data output rate configuration register*/
#define BGSH_MAG_DATA_OUTRATE__POS    0
#define BGSH_MAG_DATA_OUTRATE__MSK    0X07
#define BGSH_MAG_DATA_OUTRATE__LEN    3
#define BGSH_MAG_DATA_OUTRATE__PID    PAGE_ONE
#define BGSH_MAG_DATA_OUTRATE__REG    MAG_CONFIG_ADDR

/* Mag operation mode configuration register*/
#define BGSH_MAG_OPR_MODE__POS    3
#define BGSH_MAG_OPR_MODE__MSK    0X18
#define BGSH_MAG_OPR_MODE__LEN    2
#define BGSH_MAG_OPR_MODE__PID    PAGE_ONE
#define BGSH_MAG_OPR_MODE__REG    MAG_CONFIG_ADDR

/* Mag power mode configuration register*/
#define BGSH_MAG_POWER_MODE__POS    5
#define BGSH_MAG_POWER_MODE__MSK    0X60
#define BGSH_MAG_POWER_MODE__LEN    2
#define BGSH_MAG_POWER_MODE__PID    PAGE_ONE
#define BGSH_MAG_POWER_MODE__REG    MAG_CONFIG_ADDR

/* Gyro range configuration register*/
#define BGSH_GYR_RANGE__POS    0
#define BGSH_GYR_RANGE__MSK    0X07
#define BGSH_GYR_RANGE__LEN    3
#define BGSH_GYR_RANGE__PID    PAGE_ONE
#define BGSH_GYR_RANGE__REG    GYRO_CONFIG_ADDR

/* Gyro bandwidth configuration register*/
#define BGSH_GYR_BANDWIDTH__POS    3
#define BGSH_GYR_BANDWIDTH__MSK    0X38
#define BGSH_GYR_BANDWIDTH__LEN    3
#define BGSH_GYR_BANDWIDTH__PID    PAGE_ONE
#define BGSH_GYR_BANDWIDTH__REG    GYRO_CONFIG_ADDR

/* Gyro power mode configuration register*/
#define BGSH_GYR_POWER_MODE__POS    0
#define BGSH_GYR_POWER_MODE__MSK    0X07
#define BGSH_GYR_POWER_MODE__LEN    3
#define BGSH_GYR_POWER_MODE__PID    PAGE_ONE
#define BGSH_GYR_POWER_MODE__REG    GYRO_MODE_CONFIG_ADDR

/* Sleep configuration registers*/
/* Accel sleep mode configuration register*/
#define BGSH_ACC_SLEEP_MODE__POS    0
#define BGSH_ACC_SLEEP_MODE__MSK    0X01
#define BGSH_ACC_SLEEP_MODE__LEN    1
#define BGSH_ACC_SLEEP_MODE__PID    PAGE_ONE
#define BGSH_ACC_SLEEP_MODE__REG    ACC_SLEEP_CONFIG_ADDR

/* Accel sleep duration configuration register*/
#define BGSH_ACC_SLEEP_DUR__POS    1
#define BGSH_ACC_SLEEP_DUR__MSK    0X1E
#define BGSH_ACC_SLEEP_DUR__LEN    4
#define BGSH_ACC_SLEEP_DUR__PID    PAGE_ONE
#define BGSH_ACC_SLEEP_DUR__REG    ACC_SLEEP_CONFIG_ADDR

/* Gyro sleep duration configuration register*/
#define BGSH_GYR_SLEEP_DUR__POS    0
#define BGSH_GYR_SLEEP_DUR__MSK    0X07
#define BGSH_GYR_SLEEP_DUR__LEN    3
#define BGSH_GYR_SLEEP_DUR__PID    PAGE_ONE
#define BGSH_GYR_SLEEP_DUR__REG    GYR_SLEEP_CONFIG_ADDR

/* Gyro auto sleep duration configuration register*/
#define BGSH_GYR_AUTO_SLEEP_DUR__POS    3
#define BGSH_GYR_AUTO_SLEEP_DUR__MSK    0X38
#define BGSH_GYR_AUTO_SLEEP_DUR__LEN    3
#define BGSH_GYR_AUTO_SLEEP_DUR__PID    PAGE_ONE
#define BGSH_GYR_AUTO_SLEEP_DUR__REG    GYR_SLEEP_CONFIG_ADDR

/* Mag sleep mode configuration register*/
#define BGSH_MAG_SLEEP_MODE__POS    0
#define BGSH_MAG_SLEEP_MODE__MSK    0X01
#define BGSH_MAG_SLEEP_MODE__LEN    1
#define BGSH_MAG_SLEEP_MODE__PID    PAGE_ONE
#define BGSH_MAG_SLEEP_MODE__REG    MAG_SLEEP_CONFIG_ADDR

/* Mag sleep duration configuration register*/
#define BGSH_MAG_SLEEP_DUR__POS    1
#define BGSH_MAG_SLEEP_DUR__MSK    0X1E
#define BGSH_MAG_SLEEP_DUR__LEN    4
#define BGSH_MAG_SLEEP_DUR__PID    PAGE_ONE
#define BGSH_MAG_SLEEP_DUR__REG    MAG_SLEEP_CONFIG_ADDR

/* Interrupt registers*/
/* Gyro any motion interrupt msk register*/
#define BGSH_GYR_AM_INTMSK__POS    2
#define BGSH_GYR_AM_INTMSK__MSK    0X04
#define BGSH_GYR_AM_INTMSK__LEN    1
#define BGSH_GYR_AM_INTMSK__PID    PAGE_ONE
#define BGSH_GYR_AM_INTMSK__REG    INT_MSK_ADDR

/* Gyro high rate interrupt msk register*/
#define BGSH_GYR_HIGH_RATE_INTMSK__POS    3
#define BGSH_GYR_HIGH_RATE_INTMSK__MSK    0X08
#define BGSH_GYR_HIGH_RATE_INTMSK__LEN    1
#define BGSH_GYR_HIGH_RATE_INTMSK__PID    PAGE_ONE
#define BGSH_GYR_HIGH_RATE_INTMSK__REG    INT_MSK_ADDR

/* Accel high g interrupt msk register*/
#define BGSH_ACC_HIGH_G_INTMSK__POS    5
#define BGSH_ACC_HIGH_G_INTMSK__MSK    0X20
#define BGSH_ACC_HIGH_G_INTMSK__LEN    1
#define BGSH_ACC_HIGH_G_INTMSK__PID    PAGE_ONE
#define BGSH_ACC_HIGH_G_INTMSK__REG    INT_MSK_ADDR

/* Accel any motion interrupt msk register*/
#define BGSH_ACC_AM_INTMSK__POS    6
#define BGSH_ACC_AM_INTMSK__MSK    0X40
#define BGSH_ACC_AM_INTMSK__LEN    1
#define BGSH_ACC_AM_INTMSK__PID    PAGE_ONE
#define BGSH_ACC_AM_INTMSK__REG    INT_MSK_ADDR

/* Accel any motion interrupt msk register*/
#define BGSH_ACC_NM_INTMSK__POS    7
#define BGSH_ACC_NM_INTMSK__MSK    0X80
#define BGSH_ACC_NM_INTMSK__LEN    1
#define BGSH_ACC_NM_INTMSK__PID    PAGE_ONE
#define BGSH_ACC_NM_INTMSK__REG    INT_MSK_ADDR

/* Gyro any motion interrupt register*/
#define BGSH_GYR_AM_INT__POS    2
#define BGSH_GYR_AM_INT__MSK    0X04
#define BGSH_GYR_AM_INT__LEN    1
#define BGSH_GYR_AM_INT__PID    PAGE_ONE
#define BGSH_GYR_AM_INT__REG    INT_ADDR

/* Gyro high rate interrupt register*/
#define BGSH_GYR_HIGH_RATE_INT__POS    3
#define BGSH_GYR_HIGH_RATE_INT__MSK    0X08
#define BGSH_GYR_HIGH_RATE_INT__LEN    1
#define BGSH_GYR_HIGH_RATE_INT__PID    PAGE_ONE
#define BGSH_GYR_HIGH_RATE_INT__REG    INT_ADDR

/* Accel high g interrupt register*/
#define BGSH_ACC_HIGH_G_INT__POS    5
#define BGSH_ACC_HIGH_G_INT__MSK    0X20
#define BGSH_ACC_HIGH_G_INT__LEN    1
#define BGSH_ACC_HIGH_G_INT__PID    PAGE_ONE
#define BGSH_ACC_HIGH_G_INT__REG    INT_ADDR

/* Accel any motion interrupt register*/
#define BGSH_ACC_AM_INT__POS    6
#define BGSH_ACC_AM_INT__MSK    0X40
#define BGSH_ACC_AM_INT__LEN    1
#define BGSH_ACC_AM_INT__PID    PAGE_ONE
#define BGSH_ACC_AM_INT__REG    INT_ADDR

/*Accel any motion interrupt register*/
#define BGSH_ACC_NM_INT__POS    7
#define BGSH_ACC_NM_INT__MSK    0X80
#define BGSH_ACC_NM_INT__LEN    1
#define BGSH_ACC_NM_INT__PID    PAGE_ONE
#define BGSH_ACC_NM_INT__REG    INT_ADDR

/*Accel any motion threshold setting*/
#define BGSH_ACC_AM_THRES__POS    0
#define BGSH_ACC_AM_THRES__MSK    0XFF
#define BGSH_ACC_AM_THRES__LEN    8
#define BGSH_ACC_AM_THRES__PID    PAGE_ONE
#define BGSH_ACC_AM_THRES__REG    ACC_AM_THRES_ADDR

/*Accel interrupt setting register*/
#define BGSH_ACC_AM_DUR_SET__POS    0
#define BGSH_ACC_AM_DUR_SET__MSK    0X03
#define BGSH_ACC_AM_DUR_SET__LEN    2
#define BGSH_ACC_AM_DUR_SET__PID    PAGE_ONE
#define BGSH_ACC_AM_DUR_SET__REG    ACC_INT_SETTINGS_ADDR

/* Accel AM/NM axis selection register*/
#define BGSH_ACC_AM_X_AXIS__POS    2
#define BGSH_ACC_AM_X_AXIS__MSK    0X04
#define BGSH_ACC_AM_X_AXIS__LEN    1
#define BGSH_ACC_AM_X_AXIS__PID    PAGE_ONE
#define BGSH_ACC_AM_X_AXIS__REG    ACC_INT_SETTINGS_ADDR

#define BGSH_ACC_AM_Y_AXIS__POS    3
#define BGSH_ACC_AM_Y_AXIS__MSK    0X08
#define BGSH_ACC_AM_Y_AXIS__LEN    1
#define BGSH_ACC_AM_Y_AXIS__PID    PAGE_ONE
#define BGSH_ACC_AM_Y_AXIS__REG    ACC_INT_SETTINGS_ADDR

#define BGSH_ACC_AM_Z_AXIS__POS    4
#define BGSH_ACC_AM_Z_AXIS__MSK    0X10
#define BGSH_ACC_AM_Z_AXIS__LEN    1
#define BGSH_ACC_AM_Z_AXIS__PID    PAGE_ONE
#define BGSH_ACC_AM_Z_AXIS__REG    ACC_INT_SETTINGS_ADDR

/* Accel high g axis selection register*/
#define BGSH_ACC_HIGH_G_X_AXIS__POS    5
#define BGSH_ACC_HIGH_G_X_AXIS__MSK    0X20
#define BGSH_ACC_HIGH_G_X_AXIS__LEN    1
#define BGSH_ACC_HIGH_G_X_AXIS__PID    PAGE_ONE
#define BGSH_ACC_HIGH_G_X_AXIS__REG    ACC_INT_SETTINGS_ADDR

#define BGSH_ACC_HIGH_G_Y_AXIS__POS    6
#define BGSH_ACC_HIGH_G_Y_AXIS__MSK    0X40
#define BGSH_ACC_HIGH_G_Y_AXIS__LEN    1
#define BGSH_ACC_HIGH_G_Y_AXIS__PID    PAGE_ONE
#define BGSH_ACC_HIGH_G_Y_AXIS__REG    ACC_INT_SETTINGS_ADDR

#define BGSH_ACC_HIGH_G_Z_AXIS__POS    7
#define BGSH_ACC_HIGH_G_Z_AXIS__MSK    0X80
#define BGSH_ACC_HIGH_G_Z_AXIS__LEN    1
#define BGSH_ACC_HIGH_G_Z_AXIS__PID    PAGE_ONE
#define BGSH_ACC_HIGH_G_Z_AXIS__REG    ACC_INT_SETTINGS_ADDR

/* Accel High g duration setting register*/
#define BGSH_ACC_HIGH_G_DURATION__POS    0
#define BGSH_ACC_HIGH_G_DURATION__MSK    0XFF
#define BGSH_ACC_HIGH_G_DURATION__LEN    8
#define BGSH_ACC_HIGH_G_DURATION__PID    PAGE_ONE
#define BGSH_ACC_HIGH_G_DURATION__REG    ACC_HG_DURATION_ADDR

/* Accel High g threshold setting register*/
#define BGSH_ACC_HIG_THRSH__POS    0
#define BGSH_ACC_HIG_THRSH__MSK    0XFF
#define BGSH_ACC_HIG_THRSH__LEN    8
#define BGSH_ACC_HIG_THRSH__PID    PAGE_ONE
#define BGSH_ACC_HIG_THRSH__REG    ACC_HG_THRES_ADDR

/* Accel no/slow motion threshold setting*/
#define BGSH_ACC_NS_THRESHOLD__POS    0
#define BGSH_ACC_NS_THRESHOLD__MSK    0XFF
#define BGSH_ACC_NS_THRESHOLD__LEN    8
#define BGSH_ACC_NS_THRESHOLD__PID    PAGE_ONE
#define BGSH_ACC_NS_THRESHOLD__REG    ACC_NM_THRES_ADDR

/* Accel no/slow motion enable setting*/
#define BGSH_ACC_NM_SM_ENABLE__POS    0
#define BGSH_ACC_NM_SM_ENABLE__MSK    0X01
#define BGSH_ACC_NM_SM_ENABLE__LEN    1
#define BGSH_ACC_NM_SM_ENABLE__PID    PAGE_ONE
#define BGSH_ACC_NM_SM_ENABLE__REG    ACC_NM_SET_ADDR

/* Accel no/slow motion duration setting*/
#define BGSH_ACC_NS_DURATION__POS    1
#define BGSH_ACC_NS_DURATION__MSK    0X7E
#define BGSH_ACC_NS_DURATION__LEN    6
#define BGSH_ACC_NS_DURATION__PID    PAGE_ONE
#define BGSH_ACC_NS_DURATION__REG    ACC_NM_SET_ADDR

/*Gyro interrupt setting register*/
/*Gyro any motion axis setting*/
#define BGSH_GYR_AM_X_AXIS__POS    0
#define BGSH_GYR_AM_X_AXIS__MSK    0X01
#define BGSH_GYR_AM_X_AXIS__LEN    1
#define BGSH_GYR_AM_X_AXIS__PID    PAGE_ONE
#define BGSH_GYR_AM_X_AXIS__REG    GYR_INT_SETING_ADDR

#define BGSH_GYR_AM_Y_AXIS__POS    1
#define BGSH_GYR_AM_Y_AXIS__MSK    0X02
#define BGSH_GYR_AM_Y_AXIS__LEN    1
#define BGSH_GYR_AM_Y_AXIS__PID    PAGE_ONE
#define BGSH_GYR_AM_Y_AXIS__REG    GYR_INT_SETING_ADDR

#define BGSH_GYR_AM_Z_AXIS__POS    2
#define BGSH_GYR_AM_Z_AXIS__MSK    0X04
#define BGSH_GYR_AM_Z_AXIS__LEN    1
#define BGSH_GYR_AM_Z_AXIS__PID    PAGE_ONE
#define BGSH_GYR_AM_Z_AXIS__REG    GYR_INT_SETING_ADDR

/*Gyro high rate axis setting*/
#define BGSH_GYR_HR_X_AXIS__POS    3
#define BGSH_GYR_HR_X_AXIS__MSK    0X08
#define BGSH_GYR_HR_X_AXIS__LEN    1
#define BGSH_GYR_HR_X_AXIS__PID    PAGE_ONE
#define BGSH_GYR_HR_X_AXIS__REG    GYR_INT_SETING_ADDR

#define BGSH_GYR_HR_Y_AXIS__POS    4
#define BGSH_GYR_HR_Y_AXIS__MSK    0X10
#define BGSH_GYR_HR_Y_AXIS__LEN    1
#define BGSH_GYR_HR_Y_AXIS__PID    PAGE_ONE
#define BGSH_GYR_HR_Y_AXIS__REG    GYR_INT_SETING_ADDR

#define BGSH_GYR_HR_Z_AXIS__POS    5
#define BGSH_GYR_HR_Z_AXIS__MSK    0X20
#define BGSH_GYR_HR_Z_AXIS__LEN    1
#define BGSH_GYR_HR_Z_AXIS__PID    PAGE_ONE
#define BGSH_GYR_HR_Z_AXIS__REG    GYR_INT_SETING_ADDR

/* Gyro filter setting*/
#define BGSH_GYR_AM_FILT__POS    6
#define BGSH_GYR_AM_FILT__MSK    0X40
#define BGSH_GYR_AM_FILT__LEN    1
#define BGSH_GYR_AM_FILT__PID    PAGE_ONE
#define BGSH_GYR_AM_FILT__REG    GYR_INT_SETING_ADDR

#define BGSH_GYR_HR_FILT__POS    7
#define BGSH_GYR_HR_FILT__MSK    0X80
#define BGSH_GYR_HR_FILT__LEN    1
#define BGSH_GYR_HR_FILT__PID    PAGE_ONE
#define BGSH_GYR_HR_FILT__REG    GYR_INT_SETING_ADDR

/* Gyro high rate X axis settings*/
#define BGSH_GYR_HR_X_THRESH__POS    0
#define BGSH_GYR_HR_X_THRESH__MSK    0X1F
#define BGSH_GYR_HR_X_THRESH__LEN    5
#define BGSH_GYR_HR_X_THRESH__PID    PAGE_ONE
#define BGSH_GYR_HR_X_THRESH__REG    GYR_HR_X_SET_ADDR

#define BGSH_GYR_HR_X_HYST__POS    5
#define BGSH_GYR_HR_X_HYST__MSK    0X60
#define BGSH_GYR_HR_X_HYST__LEN    2
#define BGSH_GYR_HR_X_HYST__PID    PAGE_ONE
#define BGSH_GYR_HR_X_HYST__REG    GYR_HR_X_SET_ADDR

#define BGSH_GYR_HR_X_DUR__POS    0
#define BGSH_GYR_HR_X_DUR__MSK    0XFF
#define BGSH_GYR_HR_X_DUR__LEN    8
#define BGSH_GYR_HR_X_DUR__PID    PAGE_ONE
#define BGSH_GYR_HR_X_DUR__REG    GYR_DUR_X_ADDR

/* Gyro high rate Y axis settings*/
#define BGSH_GYR_HR_Y_THRESH__POS    0
#define BGSH_GYR_HR_Y_THRESH__MSK    0X1F
#define BGSH_GYR_HR_Y_THRESH__LEN    5
#define BGSH_GYR_HR_Y_THRESH__PID    PAGE_ONE
#define BGSH_GYR_HR_Y_THRESH__REG    GYR_HR_Y_SET_ADDR

#define BGSH_GYR_HR_Y_HYST__POS    5
#define BGSH_GYR_HR_Y_HYST__MSK    0X60
#define BGSH_GYR_HR_Y_HYST__LEN    2
#define BGSH_GYR_HR_Y_HYST__PID    PAGE_ONE
#define BGSH_GYR_HR_Y_HYST__REG    GYR_HR_Y_SET_ADDR

#define BGSH_GYR_HR_Y_DUR__POS    0
#define BGSH_GYR_HR_Y_DUR__MSK    0XFF
#define BGSH_GYR_HR_Y_DUR__LEN    8
#define BGSH_GYR_HR_Y_DUR__PID    PAGE_ONE
#define BGSH_GYR_HR_Y_DUR__REG    GYR_DUR_Y_ADDR

/* Gyro high rate Z axis settings*/
#define BGSH_GYR_HR_Z_THRESH__POS    0
#define BGSH_GYR_HR_Z_THRESH__MSK    0X1F
#define BGSH_GYR_HR_Z_THRESH__LEN    5
#define BGSH_GYR_HR_Z_THRESH__PID    PAGE_ONE
#define BGSH_GYR_HR_Z_THRESH__REG    GYR_HR_Z_SET_ADDR

#define BGSH_GYR_HR_Z_HYST__POS    5
#define BGSH_GYR_HR_Z_HYST__MSK    0X60
#define BGSH_GYR_HR_Z_HYST__LEN    2
#define BGSH_GYR_HR_Z_HYST__PID    PAGE_ONE
#define BGSH_GYR_HR_Z_HYST__REG    GYR_HR_Z_SET_ADDR

#define BGSH_GYR_HR_Z_DUR__POS    0
#define BGSH_GYR_HR_Z_DUR__MSK    0XFF
#define BGSH_GYR_HR_Z_DUR__LEN    8
#define BGSH_GYR_HR_Z_DUR__PID    PAGE_ONE
#define BGSH_GYR_HR_Z_DUR__REG    GYR_DUR_Z_ADDR

/*Gyro any motion threshold setting*/
#define BGSH_GYR_AM_THRES__POS    0
#define BGSH_GYR_AM_THRES__MSK    0X7F
#define BGSH_GYR_AM_THRES__LEN    7
#define BGSH_GYR_AM_THRES__PID    PAGE_ONE
#define BGSH_GYR_AM_THRES__REG    GYR_AM_THRES_ADDR

/* Gyro any motion slope sample setting*/
#define BGSH_GYR_SLOPE_SAMPLES__POS    0
#define BGSH_GYR_SLOPE_SAMPLES__MSK    0X03
#define BGSH_GYR_SLOPE_SAMPLES__LEN    2
#define BGSH_GYR_SLOPE_SAMPLES__PID    PAGE_ONE
#define BGSH_GYR_SLOPE_SAMPLES__REG    GYR_AM_SET_ADDR

/* Gyro awake duration setting*/
#define BGSH_GYR_AWAKE_DUR__POS    2
#define BGSH_GYR_AWAKE_DUR__MSK    0X0C
#define BGSH_GYR_AWAKE_DUR__LEN    2
#define BGSH_GYR_AWAKE_DUR__PID    PAGE_ONE
#define BGSH_GYR_AWAKE_DUR__REG    GYR_AM_SET_ADDR

/* PAGE1 DATA REGISTERS DEFINITION END*/


#define TO_REG(item) item##__REG
#define TO_MSK(item) item##__MSK
#define TO_LEN(item) item##__LEN
#define TO_POS(item) item##__POS
#define TO_PAGEID(item) item##__PID

#define BGSH_GET_BITSLICE(regvar, bitname)\
((regvar & bitname##__MSK) >> bitname##__POS)

#define BGSH_SET_BITSLICE(regvar, bitname, val)\
((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))

#endif

