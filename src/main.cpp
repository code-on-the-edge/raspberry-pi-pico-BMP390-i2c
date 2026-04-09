#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "stdio.h"
#include "math.h"

// PICO I2C
#define I2C_PORT i2c0
#define I2C_SDA 0
#define I2C_SCL 1
#define I2C_CLK_FREQ 400*1000

// BMP390 I2C Address 
#define BMP390_I2C_ADDR              0x77
#define BMP390_REG_CHIP_ID           0x00
#define BMP390_CHIP_ID               0x60

// Calibration data registers
#define BMP390_CALIB_DATA_START      0x31
#define BMP390_CALIB_DATA_LENGTH     21

// Status registers
#define BMP390_REG_STATUS            0x03

// Data registers
#define BMP390_REG_PRESS_MSB         0x04
#define BMP390_REG_PRESS_LSB         0x05
#define BMP390_REG_PRESS_XLSB        0x06
#define BMP390_REG_TEMP_MSB          0x07
#define BMP390_REG_TEMP_LSB          0x08
#define BMP390_REG_TEMP_XLSB         0x09

// Control registers
#define BMP390_REG_CMD               0x7E
#define BMP390_REG_CONFIG            0x1F
#define BMP390_REG_ODR               0x1D
#define BMP390_REG_OSR               0x1C
#define BMP390_REG_PWR_CTRL          0x1B
#define BMP390_REG_IF_CONF           0x1A
#define BMP390_REG_INT_CTRL          0x19


// Configuration options
#define BMP390_OSR_PRESSURE_x1       0x00
#define BMP390_OSR_PRESSURE_x2       0x01
#define BMP390_OSR_PRESSURE_x4       0x02
#define BMP390_OSR_PRESSURE_x8       0x03
#define BMP390_OSR_PRESSURE_x16      0x04
#define BMP390_OSR_PRESSURE_x32      0x05

#define BMP390_OSR_TEMPERATURE_x1    (0x00 << 3)
#define BMP390_OSR_TEMPERATURE_x2    (0x01 << 3)
#define BMP390_OSR_TEMPERATURE_x4    (0x02 << 3)
#define BMP390_OSR_TEMPERATURE_x8    (0x03 << 3)
#define BMP390_OSR_TEMPERATURE_x16   (0x04 << 3)
#define BMP390_OSR_TEMPERATURE_x32   (0x05 << 3)

#define BMP390_FILTER_COEFF_OFF      (0x00 << 1)
#define BMP390_FILTER_COEFF_1        (0x01 << 1)
#define BMP390_FILTER_COEFF_3        (0x02 << 1)
#define BMP390_FILTER_COEFF_7        (0x03 << 1)
#define BMP390_FILTER_COEFF_15       (0x04 << 1)
#define BMP390_FILTER_COEFF_31       (0x05 << 1)
#define BMP390_FILTER_COEFF_63       (0x06 << 1)
#define BMP390_FILTER_COEFF_127      (0x07 << 1)

#define BMP390_ODR_50      			 0x02
#define BMP390_ODR_25      			 0x03

#define BMP390_MODE_SLEEP            (0x00 << 4)
#define BMP390_MODE_FORCED           (0x01 << 4)
#define BMP390_MODE_NORMAL           (0x03 << 4)

#define BMP390_PRESSURE_ON           0x1
#define BMP390_TEMP_ON               (0x1 << 1)

typedef struct {
    int8_t nvm_par_p11;
    int8_t nvm_par_p10;
    int16_t nvm_par_p9;
    int8_t nvm_par_p8;
    int8_t nvm_par_p7;
    uint16_t nvm_par_p6;
    uint16_t nvm_par_p5;
    int8_t nvm_par_p4;
    int8_t nvm_par_p3;
    int16_t nvm_par_p2;
    int16_t nvm_par_p1;
    int8_t nvm_par_t3;
    uint16_t nvm_par_t2;
    uint16_t nvm_par_t1;
} bmp390_nvm_par;

typedef struct {
    double par_p11;
    double par_p10;
    double par_p9;
    double par_p8;
    double par_p7;
    double par_p6;
    double par_p5;
    double par_p4;
    double par_p3;
    double par_p2;
    double par_p1;
    double par_t3;
    double par_t2;
    double par_t1;
    double t_lin;
} BMP390_calib_par;

bmp390_nvm_par bmp390_calib;
BMP390_calib_par calib_data;
uint8_t bmp390_chipID;
uint32_t bmp390_tRaw, bmp390_pRaw;

float bmp390_Temperature_C, bmp390_Pressure_Pa, bmp390_Pressure_hPa;

void i2c_core_init(i2c_inst_t *i2c_inst, uint8_t sda, uint8_t scl, uint32_t i2c_clk_freq) {
    // I2C Initialisation. Using it at 400Khz.
    i2c_init(i2c_inst, i2c_clk_freq);
    
    gpio_set_function(sda, GPIO_FUNC_I2C);
    gpio_set_function(scl, GPIO_FUNC_I2C);
    gpio_pull_up(sda);
    gpio_pull_up(scl);
}

// Write 1 byte to the specified register
int reg_write(i2c_inst_t *i2c, const uint addr, const uint8_t reg, uint8_t *buf, const uint8_t nbytes) {

    int num_bytes_read = 0;
    uint8_t msg[nbytes + 1];

    // Check to make sure caller is sending 1 or more bytes
    if (nbytes < 1) {
        return 0;
    }

    // Append register address to front of data packet
    msg[0] = reg;
    for (int i = 0; i < nbytes; i++) {
        msg[i + 1] = buf[i];
    }

    // Write data to register(s) over I2C
    num_bytes_read = i2c_write_blocking(i2c, addr, msg, (nbytes + 1), false);

    return num_bytes_read;
}

// Read byte(s) from specified register. If nbytes > 1, read from consecutive
// registers.
int reg_read(i2c_inst_t *i2c, const uint addr, const uint8_t reg, uint8_t *buf, const uint8_t nbytes) {

    int num_bytes_read = 0;

    // Check to make sure caller is asking for 1 or more bytes
    if (nbytes < 1) {
        return 0;
    }

    // Read data from register(s) over I2C
    i2c_write_blocking(i2c, addr, &reg, 1, true);
    num_bytes_read = i2c_read_blocking(i2c, addr, buf, nbytes, false);

    return num_bytes_read;
}

void bmp390_trim_param() {
	uint8_t trimdata[BMP390_CALIB_DATA_LENGTH]={0};
	// Read NVM from 0x31 to 0x45
	// reg_read(spicore->spi_inst, spicore->csn_bmp390_pin, BMP390_CALIB_DATA_START, trimdata, BMP390_CALIB_DATA_LENGTH);
    reg_read(I2C_PORT, BMP390_I2C_ADDR, BMP390_CALIB_DATA_START, trimdata, BMP390_CALIB_DATA_LENGTH);

    for (int i = 0; i < BMP390_CALIB_DATA_LENGTH; i++)
    {
        printf("trimdata[%d] : %x\n\r", i, trimdata[i]);
    }
    

	bmp390_calib.nvm_par_t1 = ((uint16_t)trimdata[1]<<8) | trimdata[0];
	bmp390_calib.nvm_par_t2 = ((uint16_t)trimdata[3]<<8) | trimdata[2];
	bmp390_calib.nvm_par_t3 = trimdata[4];
	bmp390_calib.nvm_par_p1 = ((uint16_t)trimdata[6]<<8) | trimdata[5];
	bmp390_calib.nvm_par_p2 = ((uint16_t)trimdata[8]<<8) | trimdata[7];
	bmp390_calib.nvm_par_p3 = trimdata[9];
	bmp390_calib.nvm_par_p4 = ((uint16_t)trimdata[10]<<8);
	bmp390_calib.nvm_par_p5 = ((uint16_t)trimdata[12]<<8) | trimdata[11];
	bmp390_calib.nvm_par_p6 = ((uint16_t)trimdata[14]<<8) | trimdata[13];
	bmp390_calib.nvm_par_p7 = ((uint16_t)trimdata[15]<<8);
	bmp390_calib.nvm_par_p8 = ((uint16_t)trimdata[16]<<8);
	bmp390_calib.nvm_par_p9 = ((uint16_t)trimdata[18]<<8) | trimdata[17];
	bmp390_calib.nvm_par_p10 = trimdata[19];
	bmp390_calib.nvm_par_p11 = trimdata[20];

	calib_data.par_t1 = (double)bmp390_calib.nvm_par_t1 / pow(2,-8);
	calib_data.par_t2 = (double)bmp390_calib.nvm_par_t2 / pow(2,30);
	calib_data.par_t3 = (double)bmp390_calib.nvm_par_t3 / pow(2,48);
	calib_data.par_p1 = ((double)bmp390_calib.nvm_par_p1 - pow(2,14)) / pow(2,20);
	calib_data.par_p2 = ((double)bmp390_calib.nvm_par_p2 - pow(2,14)) / pow(2,29);
	calib_data.par_p3 = (double)bmp390_calib.nvm_par_p3 / pow(2,32);
	calib_data.par_p4 = (double)bmp390_calib.nvm_par_p4 / pow(2,37);
	calib_data.par_p5 = (double)bmp390_calib.nvm_par_p5 / pow(2,-3);
	calib_data.par_p6 = (double)bmp390_calib.nvm_par_p6 / pow(2,6);
	calib_data.par_p7 = (double)bmp390_calib.nvm_par_p7 / pow(2,8);
	calib_data.par_p8 = (double)bmp390_calib.nvm_par_p8 / pow(2,15);
	calib_data.par_p9 = (double)bmp390_calib.nvm_par_p9 / pow(2,49);
	calib_data.par_p10 = (double)bmp390_calib.nvm_par_p10 / pow(2,48);
	calib_data.par_p11 = (double)bmp390_calib.nvm_par_p11 / pow(2,65);
}

float BMP390_compensate_temperature(uint32_t uncomp_temp, BMP390_calib_par *calib_data) {
    float partial_data1 = (float)(uncomp_temp - calib_data->par_t1);
    float partial_data2 = (float)(partial_data1 * calib_data->par_t2);

    // Update the compensated temperature in calib structure since this is needed for pressure calculation
    calib_data->t_lin = partial_data2 + (partial_data1 * partial_data1) * calib_data->par_t3;

    // Returns compensated temperature
    return calib_data->t_lin;
}

float BMP390_compensate_pressure(uint32_t uncomp_press, BMP390_calib_par *calib_data) {
    // Variables to store temporary values used in pressure compensation
    float partial_data1;
    float partial_data2;
    float partial_data3;
    float partial_data4;
    float partial_out1;
    float partial_out2;

    // Calculate intermediate values based on calibration data and linear temperature
    partial_data1 = calib_data->par_p6 * calib_data->t_lin;
    partial_data2 = calib_data->par_p7 * (calib_data->t_lin * calib_data->t_lin);
    partial_data3 = calib_data->par_p8 * (calib_data->t_lin * calib_data->t_lin * calib_data->t_lin);
    partial_out1 = calib_data->par_p5 + partial_data1 + partial_data2 + partial_data3;

    partial_data1 = calib_data->par_p2 * calib_data->t_lin;
    partial_data2 = calib_data->par_p3 * (calib_data->t_lin * calib_data->t_lin);
    partial_data3 = calib_data->par_p4 * (calib_data->t_lin * calib_data->t_lin * calib_data->t_lin);
    partial_out2 = (float)uncomp_press * (calib_data->par_p1 + partial_data1 + partial_data2 + partial_data3);

    partial_data1 = (float)uncomp_press * (float)uncomp_press;
    partial_data2 = calib_data->par_p9 + calib_data->par_p10 * calib_data->t_lin;
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3 + ((float)uncomp_press * (float)uncomp_press * (float)uncomp_press) * calib_data->par_p11;

    // Calculate and return the compensated pressure
    float comp_press = partial_out1 + partial_out2 + partial_data4;
    return comp_press;
}

void bmp390_init()
{
    uint8_t data;
    data = BMP390_OSR_PRESSURE_x8 | BMP390_OSR_TEMPERATURE_x1;
    reg_write(I2C_PORT, BMP390_I2C_ADDR, BMP390_REG_OSR, &data, 2);

    data = BMP390_FILTER_COEFF_3;
    reg_write(I2C_PORT, BMP390_I2C_ADDR, BMP390_REG_CONFIG, &data, 2);

    data = BMP390_ODR_50;
    reg_write(I2C_PORT, BMP390_I2C_ADDR, BMP390_REG_ODR, &data, 2);

    data = BMP390_MODE_NORMAL | BMP390_TEMP_ON | BMP390_PRESSURE_ON;
    reg_write(I2C_PORT, BMP390_I2C_ADDR, BMP390_REG_PWR_CTRL, &data, 2);
}

int bmp390_read_raw()
{
	uint8_t RawData[6]={0};
    uint8_t reg = BMP390_REG_PRESS_MSB;
    // Spi::reg_read(spicore->spi_inst, spicore->csn_bmp390_pin, reg, RawData, 6);
    reg_read(I2C_PORT, BMP390_I2C_ADDR, reg, RawData, 6);
    /* Calculate the Raw data for the parameters
        * Here the Pressure and Temperature are in 20 bit format and humidity in 16 bit format
        */
    bmp390_pRaw = (RawData[2]<<16)|(RawData[1]<<8)|(RawData[0]);
    bmp390_tRaw = (RawData[5]<<16)|(RawData[4]<<8)|(RawData[3]);

    // printf("bmp390_pRaw: %x\n\r", bmp390_pRaw);
    // printf("bmp390_tRaw: %x\n\r", bmp390_tRaw);

    return 0;
}

/* measure the temp, pressure and humidity
 * the values will be stored in the parameters passed to the function
 */
void bmp390_measure()
{
	if (bmp390_read_raw() == 0)
	{
		  if (bmp390_tRaw == 0x800000) bmp390_Temperature_C = 0; // value in case temp measurement was disabled
		  else
		  {
			  bmp390_Temperature_C = (BMP390_compensate_temperature (bmp390_tRaw, &calib_data));  // as per datasheet, the temp is x100
			  printf("Temperature_C: %f C, ", bmp390_Temperature_C);
		  }

		  if (bmp390_pRaw == 0x800000) bmp390_Pressure_Pa = 0; // value in case temp measurement was disabled
		  else
		  {
			  bmp390_Pressure_Pa = (BMP390_compensate_pressure (bmp390_pRaw, &calib_data));  // as per datasheet, the pressure is x256
			  printf("Pressure_Pa: %f Pa, ", bmp390_Pressure_Pa);

			  bmp390_Pressure_hPa = bmp390_Pressure_Pa/100.0f;
			  printf("Pressure_hPa: %f hPa\n\r", bmp390_Pressure_hPa);

		  }
	}


	// if the device is detached
	else
	{
		bmp390_Temperature_C = bmp390_Pressure_Pa = 0;
		printf("BME280 detached %f\n\r", bmp390_Temperature_C);
	}

}

int main()
{
    // Initialize stdio (optional, for debugging over USB)
    stdio_init_all();

    i2c_core_init(I2C_PORT, I2C_SDA, I2C_SCL, I2C_CLK_FREQ);
    bmp390_trim_param();
    bmp390_init();

    while (true)
    {
        bmp390_measure();
        sleep_ms(500);
    }

    return 0;
}