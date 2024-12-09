/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
/* STEP 3 - Include the header file of the I2C API */
#include <zephyr/drivers/i2c.h>
/* STEP 4.1 - Include the header file of printk() */
#include <zephyr/sys/printk.h>
/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 1000

/* STEP 8 - Define the I2C slave device address and the addresses of relevant registers */
#define BME680_TEMP_xlsb 	0x24 /*MUST USE*/
#define BME680_TEMP_lsb  	0x23 /*MUST USE*/
#define BME680_TEMP_msb  	0x22 /*MUST USE*/
#define BME680_SLAVE    	0x76
#define BME680_SLAVE_W    	0xEC
#define BME680_SLAVE_R    	0xED
#define BME680_par_t1_lsb  		0xE9
#define BME680_par_t1_msb  		0xEA
#define BME680_par_t2_lsb  		0x8A
#define BME680_par_t2_msb  		0x8B
#define BME680_par_t3  			0x8C
#define CTRL_MEAS			0x74 /*MUST USE*/
#define STATUS				0x1D /*MUST USE*/


/* STEP 6 - Get the node identifier of the sensor */
#define I2C_NODE DT_NODELABEL(mysensor)

uint16_t par_t1;
uint16_t par_t2;
uint8_t  par_t3;

// Function to compensate raw temperature
float compensate_temperature(uint32_t raw_temp) {
    // Extract calibration parameters (replace with actual device values)
    double var1, var2, temperature;

    // Compensation formula as per the datasheet
    var1 = (((double)raw_temp / 16384.0) - ((double)par_t1 / 1024.0)) * (double)par_t2;
    var2 = ((((double)raw_temp / 131072.0) - ((double)par_t1 / 8192.0)) *
            (((double)raw_temp / 131072.0) - ((double)par_t1 / 8192.0))) *
           ((double)par_t3 * 16.0);
    temperature = (var1 + var2) / 5120.0;

    return temperature;
}

int main(void)
{

	int ret;
	uint8_t cal_regs[5] = {BME680_par_t1_lsb, BME680_par_t1_msb, BME680_par_t2_lsb, BME680_par_t2_msb, BME680_par_t3};
	uint8_t cal_reading[5] = {0};

	/* STEP 7 - Retrieve the API-specific device structure and make sure that the device is
	 * ready to use  */
	static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE);
	if (!device_is_ready(dev_i2c.bus)) {
		printk("I2C bus %s is not ready!\n", dev_i2c.bus->name);
		return -1;
	}

	/* READ CALIBRRATION PARAMETERS */
	for (int i = 0; i < 5; i++) {
        ret = i2c_write_read_dt(&dev_i2c, &cal_regs[i], 1, &cal_reading[i], 1);
        if (ret != 0) {
            printk("Failed to read calibration register %x\n", cal_regs[i]);
        }
    }

	/* STEP 9 - Setup the sensor by writing to measurement control address */
	uint8_t config[2] = {CTRL_MEAS, 0x41};
	ret = i2c_write_dt(&dev_i2c, config, sizeof(config));
	if (ret != 0) {
		printk("Failed to write to I2C device address %x at Reg. %x \n", dev_i2c.addr,
		       config[0]);
		return -1;
	}

	par_t1 = (uint16_t)((cal_reading[1] << 8) | cal_reading[0]);
    par_t2 = (int16_t)((cal_reading[3] << 8) | cal_reading[2]);
    par_t3 = (int8_t)cal_reading[4];

	printk("Calibration Parameters:\n");
    printk("par_t1: %u\n", par_t1);
    printk("par_t2: %d\n", par_t2);
    printk("par_t3: %d\n", par_t3);

	while (1) {

		ret = i2c_write_dt(&dev_i2c, config, sizeof(config));
		if (ret != 0) {
		printk("Failed to write to I2C device address %x at Reg. %x \n", dev_i2c.addr,
		       config[0]);
		return -1;
	}

		uint8_t status_reg;
		uint8_t stat = {STATUS};
		do {
			ret = i2c_write_read_dt(&dev_i2c, &stat, 1, &status_reg, 1);  // Read STATUS register
		} while ((status_reg & 0x80) == 0);

		/* Read the temperature from the sensor */
		uint8_t temp_reading[3] = {0};
		uint8_t sensor_regs[3] = {BME680_TEMP_msb, BME680_TEMP_lsb, BME680_TEMP_xlsb};
		for (int i = 0; i < 3; i++) {
            ret = i2c_write_read_dt(&dev_i2c, &sensor_regs[i], 1, &temp_reading[i], 1);
            if (ret != 0) {
                printk("Failed to read temperature register %x\n", sensor_regs[i]);
            }
        }

		uint32_t raw_temp = (temp_reading[0] << 12) | (temp_reading[1] << 4) | (temp_reading[2] >> 4);
		float temp_celsius = compensate_temperature(raw_temp);
		double fTemp = temp_celsius * 1.8 + 32;

		// Print reading to console
		printk("Temperature in Celsius : %.2f C \n", temp_celsius);
		printk("Temperature in Fahrenheit : %.2f F \n", fTemp);
		k_msleep(SLEEP_TIME_MS);
	}
}