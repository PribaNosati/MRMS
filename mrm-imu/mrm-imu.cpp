#include "mrm-imu.h"

extern char errorMessage[];

/**Add a BNO05
@param defautI2CAddress - If true, 0x29. Otherwise 0x28.
*/
void Mrm_imu::add(bool defaultI2CAddress) {
	if (nextFree >= MAX_MRM_IMU) {
		strcpy(errorMessage, "Too many Bosch IMUs.");//Todo - enabling more sensors by changing bno055Initialize() call.
		return;
	}

	defaultI2CAddresses[nextFree] = defaultI2CAddress;
	nextFree++;
	bno055Initialize(defaultI2CAddress);
}

/**Compass
@return - North direction.
*/
float Mrm_imu::heading() {
	struct bno055_euler_float_t eulerData;
	bno055_convert_float_euler_hpr_deg(&eulerData);
	return eulerData.h;
}

/**Pitch
@return - Pitch in degrees. Inclination forwards or backwards.
*/
float Mrm_imu::pitch() {
	struct bno055_euler_float_t eulerData;
	bno055_convert_float_euler_hpr_deg(&eulerData);
	return eulerData.p;
}

/**Roll
@return - Roll in degrees. Inclination to the left or right.
*/
float Mrm_imu::roll() {
	struct bno055_euler_float_t eulerData;
	bno055_convert_float_euler_hpr_deg(&eulerData);
	return eulerData.r;
}

/** Acceleration calibration
@return - Calibration
*/
uint8_t Mrm_imu::accelerationCalibration() {
	uint8_t acc;
	BNO055_RETURN_FUNCTION_TYPE ok = bno055_get_accel_calib_stat(&acc);
	if (ok == BNO055_SUCCESS)
		return acc;
	else {
		strcpy(errorMessage, "No accelerometer");
		return 0;
	}
}

/** Gyro calibration
@return - Calibration
*/
uint8_t Mrm_imu::gyroCalibration() {
	uint8_t cal;
	BNO055_RETURN_FUNCTION_TYPE ok = bno055_get_gyro_calib_stat(&cal);
	if (ok == BNO055_SUCCESS)
		return cal;
	else {
		strcpy(errorMessage, "No gyroscope");
		return 0;
	}
}

/** Magnetic calibration
@return - Calibration
*/
uint8_t Mrm_imu::magneticCalibration() {
	uint8_t cal;
	BNO055_RETURN_FUNCTION_TYPE ok = bno055_get_mag_calib_stat(&cal);
	if (ok == BNO055_SUCCESS)
		return cal;
	else {
		strcpy(errorMessage, "No magnetometer.");
		return 0;
	}
}

/** Print to all serial ports
@param fmt - C format string
@param ... - variable arguments
*/
void Mrm_imu::print(const char* fmt, ...) {
	va_list argp;
	va_start(argp, fmt);
	vprint(fmt, argp);
	va_end(argp);
}

/** System calibration
@return - Calibration
*/
uint8_t Mrm_imu::systemCalibration() {
	uint8_t cal;
	BNO055_RETURN_FUNCTION_TYPE ok = bno055_get_sys_calib_stat(&cal);
	if (ok == BNO055_SUCCESS)
		return cal;
	else {
		strcpy(errorMessage, "mrm-imu calibration error");
		return 0;
	}
}

/**Test
@param breakWhen - A function returning bool, without arguments. If it returns true, the test() will be interrupted.
*/
void Mrm_imu::test(BreakCondition breakWhen) {
	while (breakWhen == 0 || !(*breakWhen)()) {
		for (int i = 0; i < nextFree; i++) 
			print("Y:%3i P:%3i R:%3i", (int)round(heading()), (int)round(pitch()), (int)round(roll()));
		print("\n\r");
		delay(200);
	}
}

/*----------------------------------------------------------------------------*
*  The following APIs are used for reading and writing of
*	sensor data using I2C communication
*----------------------------------------------------------------------------*/
#define BNO055_API
#ifdef	BNO055_API
#define	BNO055_I2C_BUS_WRITE_ARRAY_INDEX	((u8)1)
/*	\Brief: The API is used as I2C bus read
*	\Return : Status of the I2C read
*	\param dev_addr : The device address of the sensor
*	\param reg_addr : Address of the first register,
*   will data is going to be read
*	\param reg_data : This data read from the sensor,
*   which is hold in an array
*	\param cnt : The no of byte of data to be read
*/
s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
/*	\Brief: The API is used as SPI bus write
*	\Return : Status of the SPI write
*	\param dev_addr : The device address of the sensor
*	\param reg_addr : Address of the first register,
*   will data is going to be written
*	\param reg_data : It is a value hold in the array,
*	will be used for write the value into the register
*	\param cnt : The no of byte of data to be write
*/
s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
/*
* \Brief: I2C init routine
*/
s8 I2C_routine(void);
#endif
/********************End of I2C APIs declarations***********************/
/*	Brief : The delay routine
*	\param : delay in ms
*/
void BNO055_delay_msek(u32 msek);
/* This API is an example for reading sensor data
*	\param: None
*	\return: communication result
*/
s32 bno055_data_readout_template(void);

/* This API is an example for reading sensor data
*	\param: None
*	\return: communication result
*/
s32 Mrm_imu::bno055_data_readout_template(void)
{
	/* Variable used to return value of
	communication routine*/
	s32 comres = BNO055_ERROR;
	/* variable used to set the power mode of the sensor*/
	u8 power_mode = BNO055_INIT_VALUE;
	/*********read raw accel data***********/
	/* variable used to read the accel x data */
	s16 accel_datax = BNO055_INIT_VALUE;
	/* variable used to read the accel y data */
	s16 accel_datay = BNO055_INIT_VALUE;
	/* variable used to read the accel z data */
	s16 accel_dataz = BNO055_INIT_VALUE;
	/* variable used to read the accel xyz data */
	struct bno055_accel_t accel_xyz;

	/*********read raw mag data***********/
	/* variable used to read the mag x data */
	s16 mag_datax = BNO055_INIT_VALUE;
	/* variable used to read the mag y data */
	s16 mag_datay = BNO055_INIT_VALUE;
	/* variable used to read the mag z data */
	s16 mag_dataz = BNO055_INIT_VALUE;
	/* structure used to read the mag xyz data */
	struct bno055_mag_t mag_xyz;

	/***********read raw gyro data***********/
	/* variable used to read the gyro x data */
	s16 gyro_datax = BNO055_INIT_VALUE;
	/* variable used to read the gyro y data */
	s16 gyro_datay = BNO055_INIT_VALUE;
	/* variable used to read the gyro z data */
	s16 gyro_dataz = BNO055_INIT_VALUE;
	/* structure used to read the gyro xyz data */
	struct bno055_gyro_t gyro_xyz;

	/*************read raw Euler data************/
	/* variable used to read the euler h data */
	s16 euler_data_h = BNO055_INIT_VALUE;
	/* variable used to read the euler r data */
	s16 euler_data_r = BNO055_INIT_VALUE;
	/* variable used to read the euler p data */
	s16 euler_data_p = BNO055_INIT_VALUE;
	/* structure used to read the euler hrp data */
	struct bno055_euler_t euler_hrp;

	/************read raw quaternion data**************/
	/* variable used to read the quaternion w data */
	s16 quaternion_data_w = BNO055_INIT_VALUE;
	/* variable used to read the quaternion x data */
	s16 quaternion_data_x = BNO055_INIT_VALUE;
	/* variable used to read the quaternion y data */
	s16 quaternion_data_y = BNO055_INIT_VALUE;
	/* variable used to read the quaternion z data */
	s16 quaternion_data_z = BNO055_INIT_VALUE;
	/* structure used to read the quaternion wxyz data */
	struct bno055_quaternion_t quaternion_wxyz;

	/************read raw linear acceleration data***********/
	/* variable used to read the linear accel x data */
	s16 linear_accel_data_x = BNO055_INIT_VALUE;
	/* variable used to read the linear accel y data */
	s16 linear_accel_data_y = BNO055_INIT_VALUE;
	/* variable used to read the linear accel z data */
	s16 linear_accel_data_z = BNO055_INIT_VALUE;
	/* structure used to read the linear accel xyz data */
	struct bno055_linear_accel_t linear_acce_xyz;

	/*****************read raw gravity sensor data****************/
	/* variable used to read the gravity x data */
	s16 gravity_data_x = BNO055_INIT_VALUE;
	/* variable used to read the gravity y data */
	s16 gravity_data_y = BNO055_INIT_VALUE;
	/* variable used to read the gravity z data */
	s16 gravity_data_z = BNO055_INIT_VALUE;
	/* structure used to read the gravity xyz data */
	struct bno055_gravity_t gravity_xyz;

	/*************read accel converted data***************/
	/* variable used to read the accel x data output as m/s2 or mg */
	double d_accel_datax = BNO055_INIT_VALUE;
	/* variable used to read the accel y data output as m/s2 or mg */
	double d_accel_datay = BNO055_INIT_VALUE;
	/* variable used to read the accel z data output as m/s2 or mg */
	double d_accel_dataz = BNO055_INIT_VALUE;
	/* structure used to read the accel xyz data output as m/s2 or mg */
	struct bno055_accel_double_t d_accel_xyz;

	/******************read mag converted data********************/
	/* variable used to read the mag x data output as uT*/
	double d_mag_datax = BNO055_INIT_VALUE;
	/* variable used to read the mag y data output as uT*/
	double d_mag_datay = BNO055_INIT_VALUE;
	/* variable used to read the mag z data output as uT*/
	double d_mag_dataz = BNO055_INIT_VALUE;
	/* structure used to read the mag xyz data output as uT*/
	struct bno055_mag_double_t d_mag_xyz;

	/*****************read gyro converted data************************/
	/* variable used to read the gyro x data output as dps or rps */
	double d_gyro_datax = BNO055_INIT_VALUE;
	/* variable used to read the gyro y data output as dps or rps */
	double d_gyro_datay = BNO055_INIT_VALUE;
	/* variable used to read the gyro z data output as dps or rps */
	double d_gyro_dataz = BNO055_INIT_VALUE;
	/* structure used to read the gyro xyz data output as dps or rps */
	struct bno055_gyro_double_t d_gyro_xyz;

	/*******************read euler converted data*******************/
	/* variable used to read the euler h data output
	as degree or radians*/
	double d_euler_data_h = BNO055_INIT_VALUE;
	/* variable used to read the euler r data output
	as degree or radians*/
	double d_euler_data_r = BNO055_INIT_VALUE;
	/* variable used to read the euler p data output
	as degree or radians*/
	double d_euler_data_p = BNO055_INIT_VALUE;
	/* structure used to read the euler hrp data output
	as as degree or radians */
	struct bno055_euler_double_t d_euler_hpr;

	/*********read linear acceleration converted data**********/
	/* variable used to read the linear accel x data output as m/s2*/
	double d_linear_accel_datax = BNO055_INIT_VALUE;
	/* variable used to read the linear accel y data output as m/s2*/
	double d_linear_accel_datay = BNO055_INIT_VALUE;
	/* variable used to read the linear accel z data output as m/s2*/
	double d_linear_accel_dataz = BNO055_INIT_VALUE;
	/* structure used to read the linear accel xyz data output as m/s2*/
	struct bno055_linear_accel_double_t d_linear_accel_xyz;

	/********************Gravity converted data**********************/
	/* variable used to read the gravity sensor x data output as m/s2*/
	double d_gravity_data_x = BNO055_INIT_VALUE;
	/* variable used to read the gravity sensor y data output as m/s2*/
	double d_gravity_data_y = BNO055_INIT_VALUE;
	/* variable used to read the gravity sensor z data output as m/s2*/
	double d_gravity_data_z = BNO055_INIT_VALUE;
	/* structure used to read the gravity xyz data output as m/s2*/
	struct bno055_gravity_double_t d_gravity_xyz;
	/*---------------------------------------------------------------------------*
	*********************** START INITIALIZATION ************************
	*--------------------------------------------------------------------------*/
#ifdef	BNO055_API
	/*	Based on the user need configure I2C interface.
	*	It is example code to explain how to use the bno055 API*/
	I2C_routine();
#endif
	/*--------------------------------------------------------------------------*
	*  This API used to assign the value/reference of
	*	the following parameters
	*	I2C address
	*	Bus Write
	*	Bus read
	*	Chip id
	*	Page id
	*	Accel revision id
	*	Mag revision id
	*	Gyro revision id
	*	Boot loader revision id
	*	Software revision id
	*-------------------------------------------------------------------------*/
	comres = bno055_init(&bno055);

	/*	For initializing the BNO sensor it is required to the operation mode
	of the sensor as NORMAL
	Normal mode can set from the register
	Page - page0
	register - 0x3E
	bit positions - 0 and 1*/
	power_mode = BNO055_POWER_MODE_NORMAL;
	/* set the power mode as NORMAL*/
	comres += bno055_set_power_mode(power_mode);
	/*----------------------------------------------------------------*
	************************* END INITIALIZATION *************************
	*-----------------------------------------------------------------*/

	/************************* START READ RAW SENSOR DATA****************/

	/*	Using BNO055 sensor we can read the following sensor data and
	virtual sensor data
	Sensor data:
	Accel
	Mag
	Gyro
	Virtual sensor data
	Euler
	Quaternion
	Linear acceleration
	Gravity sensor */
	/*	For reading sensor raw data it is required to set the
	operation modes of the sensor
	operation mode can set from the register
	page - page0
	register - 0x3D
	bit - 0 to 3
	for sensor data read following operation mode have to set
	* SENSOR MODE
	*0x01 - BNO055_OPERATION_MODE_ACCONLY
	*0x02 - BNO055_OPERATION_MODE_MAGONLY
	*0x03 - BNO055_OPERATION_MODE_GYRONLY
	*0x04 - BNO055_OPERATION_MODE_ACCMAG
	*0x05 - BNO055_OPERATION_MODE_ACCGYRO
	*0x06 - BNO055_OPERATION_MODE_MAGGYRO
	*0x07 - BNO055_OPERATION_MODE_AMG
	based on the user need configure the operation mode*/
	comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_AMG);
	/*	Raw accel X, Y and Z data can read from the register
	page - page 0
	register - 0x08 to 0x0D*/
	comres += bno055_read_accel_x(&accel_datax);
	comres += bno055_read_accel_y(&accel_datay);
	comres += bno055_read_accel_z(&accel_dataz);
	comres += bno055_read_accel_xyz(&accel_xyz);
	/*	Raw mag X, Y and Z data can read from the register
	page - page 0
	register - 0x0E to 0x13*/
	comres += bno055_read_mag_x(&mag_datax);
	comres += bno055_read_mag_y(&mag_datay);
	comres += bno055_read_mag_z(&mag_dataz);
	comres += bno055_read_mag_xyz(&mag_xyz);
	/*	Raw gyro X, Y and Z data can read from the register
	page - page 0
	register - 0x14 to 0x19*/
	comres += bno055_read_gyro_x(&gyro_datax);
	comres += bno055_read_gyro_y(&gyro_datay);
	comres += bno055_read_gyro_z(&gyro_dataz);
	comres += bno055_read_gyro_xyz(&gyro_xyz);

	/************************* END READ RAW SENSOR DATA****************/

	/************************* START READ RAW FUSION DATA ********
	For reading fusion data it is required to set the
	operation modes of the sensor
	operation mode can set from the register
	page - page0
	register - 0x3D
	bit - 0 to 3
	for sensor data read following operation mode have to set
	*FUSION MODE
	*0x08 - BNO055_OPERATION_MODE_IMUPLUS
	*0x09 - BNO055_OPERATION_MODE_COMPASS
	*0x0A - BNO055_OPERATION_MODE_M4G
	*0x0B - BNO055_OPERATION_MODE_NDOF_FMC_OFF
	*0x0C - BNO055_OPERATION_MODE_NDOF
	based on the user need configure the operation mode*/
	comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
	/*	Raw Euler H, R and P data can read from the register
	page - page 0
	register - 0x1A to 0x1E */
	comres += bno055_read_euler_h(&euler_data_h);
	comres += bno055_read_euler_r(&euler_data_r);
	comres += bno055_read_euler_p(&euler_data_p);
	comres += bno055_read_euler_hrp(&euler_hrp);
	/*	Raw Quaternion W, X, Y and Z data can read from the register
	page - page 0
	register - 0x20 to 0x27 */
	comres += bno055_read_quaternion_w(&quaternion_data_w);
	comres += bno055_read_quaternion_x(&quaternion_data_x);
	comres += bno055_read_quaternion_y(&quaternion_data_y);
	comres += bno055_read_quaternion_z(&quaternion_data_z);
	comres += bno055_read_quaternion_wxyz(&quaternion_wxyz);
	/*	Raw Linear accel X, Y and Z data can read from the register
	page - page 0
	register - 0x28 to 0x2D */
	comres += bno055_read_linear_accel_x(&linear_accel_data_x);
	comres += bno055_read_linear_accel_y(&linear_accel_data_y);
	comres += bno055_read_linear_accel_z(&linear_accel_data_z);
	comres += bno055_read_linear_accel_xyz(&linear_acce_xyz);
	/*	Raw Gravity sensor X, Y and Z data can read from the register
	page - page 0
	register - 0x2E to 0x33 */
	comres += bno055_read_gravity_x(&gravity_data_x);
	comres += bno055_read_gravity_y(&gravity_data_y);
	comres += bno055_read_gravity_z(&gravity_data_z);
	comres += bno055_read_gravity_xyz(&gravity_xyz);
	/************************* END READ RAW FUSION DATA  ************/

	/******************START READ CONVERTED SENSOR DATA****************/
	/*	API used to read accel data output as double  - m/s2 and mg
	float functions also available in the BNO055 API */
	comres += bno055_convert_double_accel_x_msq(&d_accel_datax);
	comres += bno055_convert_double_accel_x_mg(&d_accel_datax);
	comres += bno055_convert_double_accel_y_msq(&d_accel_datay);
	comres += bno055_convert_double_accel_y_mg(&d_accel_datay);
	comres += bno055_convert_double_accel_z_msq(&d_accel_dataz);
	comres += bno055_convert_double_accel_z_mg(&d_accel_dataz);
	comres += bno055_convert_double_accel_xyz_msq(&d_accel_xyz);
	comres += bno055_convert_double_accel_xyz_mg(&d_accel_xyz);

	/*	API used to read mag data output as double  - uT(micro Tesla)
	float functions also available in the BNO055 API */
	comres += bno055_convert_double_mag_x_uT(&d_mag_datax);
	comres += bno055_convert_double_mag_y_uT(&d_mag_datay);
	comres += bno055_convert_double_mag_z_uT(&d_mag_dataz);
	comres += bno055_convert_double_mag_xyz_uT(&d_mag_xyz);

	/*	API used to read gyro data output as double  - dps and rps
	float functions also available in the BNO055 API */
	comres += bno055_convert_double_gyro_x_dps(&d_gyro_datax);
	comres += bno055_convert_double_gyro_y_dps(&d_gyro_datay);
	comres += bno055_convert_double_gyro_z_dps(&d_gyro_dataz);
	comres += bno055_convert_double_gyro_x_rps(&d_gyro_datax);
	comres += bno055_convert_double_gyro_y_rps(&d_gyro_datay);
	comres += bno055_convert_double_gyro_z_rps(&d_gyro_dataz);
	comres += bno055_convert_double_gyro_xyz_dps(&d_gyro_xyz);
	comres += bno055_convert_double_gyro_xyz_rps(&d_gyro_xyz);

	/*	API used to read Euler data output as double  - degree and radians
	float functions also available in the BNO055 API */
	comres += bno055_convert_double_euler_h_deg(&d_euler_data_h);
	comres += bno055_convert_double_euler_r_deg(&d_euler_data_r);
	comres += bno055_convert_double_euler_p_deg(&d_euler_data_p);
	comres += bno055_convert_double_euler_h_rad(&d_euler_data_h);
	comres += bno055_convert_double_euler_r_rad(&d_euler_data_r);
	comres += bno055_convert_double_euler_p_rad(&d_euler_data_p);
	comres += bno055_convert_double_euler_hpr_deg(&d_euler_hpr);
	comres += bno055_convert_double_euler_hpr_rad(&d_euler_hpr);

	/*	API used to read Linear acceleration data output as m/s2
	float functions also available in the BNO055 API */
	comres += bno055_convert_double_linear_accel_x_msq(
		&d_linear_accel_datax);
	comres += bno055_convert_double_linear_accel_y_msq(
		&d_linear_accel_datay);
	comres += bno055_convert_double_linear_accel_z_msq(
		&d_linear_accel_dataz);
	comres += bno055_convert_double_linear_accel_xyz_msq(
		&d_linear_accel_xyz);

	/*	API used to read Gravity sensor data output as m/s2
	float functions also available in the BNO055 API */
	comres += bno055_convert_gravity_double_x_msq(&d_gravity_data_x);
	comres += bno055_convert_gravity_double_y_msq(&d_gravity_data_y);
	comres += bno055_convert_gravity_double_z_msq(&d_gravity_data_z);
	comres += bno055_convert_double_gravity_xyz_msq(&d_gravity_xyz);
	/*-----------------------------------------------------------------------*
	************************* START DE-INITIALIZATION ***********************
	*-------------------------------------------------------------------------*/
	/*	For de - initializing the BNO sensor it is required
	to the operation mode of the sensor as SUSPEND
	Suspend mode can set from the register
	Page - page0
	register - 0x3E
	bit positions - 0 and 1*/
	power_mode = BNO055_POWER_MODE_SUSPEND;
	/* set the power mode as SUSPEND*/
	comres += bno055_set_power_mode(power_mode);

	/*---------------------------------------------------------------------*
	************************* END DE-INITIALIZATION **********************
	*---------------------------------------------------------------------*/
	return comres;
}

#ifdef	BNO055_API


/************** I2C buffer length******/

#define	I2C_BUFFER_LEN 8
#define I2C0 5
/*-------------------------------------------------------------------*
*
*	This is a sample code for read and write the data by using I2C
*	Use either I2C  based on your need
*	The device address defined in the bno055.h file
*
*--------------------------------------------------------------------*/

/*	\Brief: The API is used as I2C bus write
*	\Return : Status of the I2C write
*	\param dev_addr : The device address of the sensor
*	\param reg_addr : Address of the first register,
*   will data is going to be written
*	\param reg_data : It is a value hold in the array,
*		will be used for write the value into the register
*	\param cnt : The no of byte of data to be write
*/
s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 BNO055_iERROR = BNO055_INIT_VALUE;
	u8 array[I2C_BUFFER_LEN];
	u8 stringpos = BNO055_INIT_VALUE;

	array[BNO055_INIT_VALUE] = reg_addr;
	for (stringpos = BNO055_INIT_VALUE; stringpos < cnt; stringpos++)
		array[stringpos + BNO055_I2C_BUS_WRITE_ARRAY_INDEX] =
		*(reg_data + stringpos);

	Wire.beginTransmission(dev_addr);
	for (int i = 0; i < cnt + 1; i++)
		Wire.write(array[i]);
	Wire.endTransmission();
	/*
	* Please take the below APIs as your reference for
	* write the data using I2C communication
	* "BNO055_iERROR = I2C_WRITE_STRING(DEV_ADDR, ARRAY, CNT+1)"
	* add your I2C write APIs here
	* BNO055_iERROR is an return value of I2C read API
	* Please select your valid return value
	* In the driver BNO055_SUCCESS defined as 0
	* and FAILURE defined as -1
	* Note :
	* This is a full duplex operation,
	* The first read data is discarded, for that extra write operation
	* have to be initiated. For that cnt+1 operation done
	* in the I2C write string function
	* For more information please refer data sheet SPI communication:
	*/
	return (s8)BNO055_iERROR;
}

/*	\Brief: The API is used as I2C bus read
*	\Return : Status of the I2C read
*	\param dev_addr : The device address of the sensor
*	\param reg_addr : Address of the first register,
*  will data is going to be read
*	\param reg_data : This data read from the sensor,
*   which is hold in an array
*	\param cnt : The no of byte of data to be read
*/
s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{

	//BNO055_BUS_READ_FUNC(p_bno055->dev_addr, BNO055_CHIP_ID_REG, &data_u8, BNO055_GEN_READ_WRITE_LENGTH);
	//*chip_id_u8 = data_u8;

	Wire.beginTransmission(dev_addr);
	Wire.write((uint8_t)reg_addr);
	Wire.endTransmission();


	s32 BNO055_iERROR = BNO055_INIT_VALUE;
	u8 array[I2C_BUFFER_LEN] = { BNO055_INIT_VALUE };
	u8 stringpos = BNO055_INIT_VALUE;

	array[BNO055_INIT_VALUE] = reg_addr;

	Wire.requestFrom(dev_addr, cnt);
	Wire.readBytes(array, cnt);

	/* Please take the below API as your reference
	* for read the data using I2C communication
	* add your I2C read API here.
	* "BNO055_iERROR = I2C_WRITE_READ_STRING(DEV_ADDR, ARRAY, ARRAY, 1, CNT)"
	* BNO055_iERROR is an return value of SPI write API
	* Please select your valid return value
	* In the driver BNO055_SUCCESS defined as 0
	* and FAILURE defined as -1
	*/
	for (stringpos = BNO055_INIT_VALUE; stringpos < cnt; stringpos++)
		*(reg_data + stringpos) = array[stringpos];
	return (s8)BNO055_iERROR;
}
/*	Brief : The delay routine
*	\param : delay in ms
*/
void BNO055_delay_msek(u32 msek)
{
	/*Here you can write your own delay routine*/
	delay(msek);
}

#endif


void Mrm_imu::bno055Initialize(bool defaultI2CAddress)
{
	s32 comres = BNO055_ERROR;

	bno055.bus_write = BNO055_I2C_bus_write;
	bno055.bus_read = BNO055_I2C_bus_read;
	bno055.delay_msec = BNO055_delay_msek;
	bno055.dev_addr = defaultI2CAddress ? BNO055_I2C_ADDR2 : BNO055_I2C_ADDR1;
	comres = bno055_init(&bno055);
	if (comres != BNO055_SUCCESS) {
		strcpy(errorMessage, "mrm-imu not initialized");
		return;
	}

	//comres = bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
	//if (comres != BNO055_SUCCESS)
	//	errorHandler();
	bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
	if (comres != BNO055_SUCCESS)
		strcpy(errorMessage, "mrm-imu not initialized");
}

/** Print to all serial ports, pointer to list
*/
void Mrm_imu::vprint(const char* fmt, va_list argp) {

	static char buffer[100]; // Caution !!! No checking if longer than 100!
	vsprintf(buffer, fmt, argp);

	Serial.print(buffer);
	if (serial != 0)
		serial->print(buffer);
}


/**Constructor
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
*/
Mrm_imu::Mrm_imu(BluetoothSerial * hardwareSerial) {
	serial = hardwareSerial;
	nextFree = 0;
}

Mrm_imu::~Mrm_imu(){}