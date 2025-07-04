#include "stm8s.h"
#include "bmp280.h"
#include "bmp280_defs.h"

#define OVERSAMPLING_SETTINGS    UINT8_C(0x07)

#define FILTER_STANDBY_SETTINGS  UINT8_C(0x18)

#undef BME280_DOUBLE_ENABLE

extern int8_t I2C_Write(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, uint32_t len);
extern int8_t I2C_Read(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, uint32_t len);

void Delay_us(uint32_t us) {

    while (us--) {
        __asm("nop"); 
        __asm("nop");
        __asm("nop");
    }
}

#define BME280_I2C_ADDR (0x76 << 1) 

// static int8_t null_ptr_check(const struct bme280_dev *dev);
// static int8_t put_device_to_sleep(struct bme280_dev *dev);
// static int8_t write_power_mode(uint8_t sensor_mode, struct bme280_dev *dev);
// static void interleave_reg_addr(const uint8_t *reg_addr, uint8_t *temp_buff, const uint8_t *reg_data, uint32_t len);
// static int8_t get_calib_data(struct bme280_dev *dev);
// static void parse_sensor_data(const uint8_t *reg_data, struct bme280_uncomp_data *uncomp_data);

// static int8_t set_osr_humidity_settings(const struct bme280_settings *settings, struct bme280_dev *dev);
// static int8_t set_osr_settings(uint8_t desired_settings, const struct bme280_settings *settings, struct bme280_dev *dev);
// static void fill_osr_press_settings(uint8_t *reg_data, const struct bme280_settings *settings);
// static void fill_osr_temp_settings(uint8_t *reg_data, const struct bme280_settings *settings);
// static int8_t set_filter_standby_settings(uint8_t desired_settings,
//                                           const struct bme280_settings *settings,
//                                           struct bme280_dev *dev);
// static void fill_filter_settings(uint8_t *reg_data, const struct bme280_settings *settings);
// static void fill_standby_settings(uint8_t *reg_data, const struct bme280_settings *settings);
// static double compensate_temperature(const struct bme280_uncomp_data *uncomp_data,
//                                      struct bme280_calib_data *calib_data);

// static void parse_device_settings(const uint8_t *reg_data, struct bme280_settings *settings)
// {
//     settings->osr_h = BME280_GET_BITS_POS_0(reg_data[0], BME280_CTRL_HUM);
//     settings->osr_p = BME280_GET_BITS(reg_data[2], BME280_CTRL_PRESS);
//     settings->osr_t = BME280_GET_BITS(reg_data[2], BME280_CTRL_TEMP);
//     settings->filter = BME280_GET_BITS(reg_data[3], BME280_FILTER);
//     settings->standby_time = BME280_GET_BITS(reg_data[3], BME280_STANDBY);
// }

// static int8_t reload_device_settings(const struct bme280_settings *settings, struct bme280_dev *dev)
// {
//     int8_t rslt;

//     rslt = set_osr_settings(BME280_SEL_ALL_SETTINGS, settings, dev);

//     if (rslt == BME280_OK)
//     {
//         rslt = set_filter_standby_settings(BME280_SEL_ALL_SETTINGS, settings, dev);
//     }

//     return rslt;
// }

static int8_t bme280_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t *)intf_ptr; 
    int8_t rslt = I2C_Read(dev_addr, reg_addr, reg_data, len);

    if (rslt != 0) {
        return BME280_E_COMM_FAIL; 
    }
    return BME280_OK;
}

static int8_t bme280_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t *)intf_ptr; 
    int8_t rslt = I2C_Write(dev_addr, reg_addr, (uint8_t *)reg_data, len);

    if (rslt != 0) {
        return BME280_E_COMM_FAIL; 
    }
    return BME280_OK;
}

static void bme280_delay_us(uint32_t period, void *intf_ptr)
{
    Delay_us(period); 
}

int8_t bme280_init(struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t chip_id = 0;

    rslt = null_ptr_check(dev);
    if (rslt != BME280_OK) {
        return rslt;
    }

    dev->intf = BME280_I2C_INTF;
    dev->read = bme280_i2c_read;
    dev->write = bme280_i2c_write;
    dev->delay_us = bme280_delay_us;

    rslt = bme280_get_regs(BME280_REG_CHIP_ID, &chip_id, 1, dev);

    if (rslt == BME280_OK) {
        if (chip_id == BME280_CHIP_ID) {
            dev->chip_id = chip_id;

            rslt = bme280_soft_reset(dev);

            if (rslt == BME280_OK) {

                rslt = get_calib_data(dev);
            }
        } else {
            rslt = BME280_E_DEV_NOT_FOUND;
        }
    }

    return rslt;
}

int8_t bme280_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, struct bme280_dev *dev)
{
    int8_t rslt;

    rslt = null_ptr_check(dev);
    if ((rslt == BME280_OK) && (reg_data != NULL)) {

        dev->intf_rslt = dev->read(reg_addr, reg_data, len, dev->intf_ptr);

        if (dev->intf_rslt != BME280_INTF_RET_SUCCESS) {
            rslt = BME280_E_COMM_FAIL;
        }
    } else {
        rslt = BME280_E_NULL_PTR;
    }

    return rslt;
}

int8_t bme280_set_regs(uint8_t *reg_addr, const uint8_t *reg_data, uint32_t len, struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t temp_buff[BME280_MAX_LEN]; 
    uint32_t temp_len;
    uint32_t reg_addr_cnt;

    if (len > BME280_MAX_LEN) {
        len = BME280_MAX_LEN;
    }

    rslt = null_ptr_check(dev);
    if ((rslt == BME280_OK) && (reg_addr != NULL) && (reg_data != NULL)) {
        if (len != 0) {
            temp_buff[0] = reg_data[0];

            if (len > 1) {
                interleave_reg_addr(reg_addr, temp_buff, reg_data, len);
                temp_len = ((len * 2) - 1);
            } else {
                temp_len = len;
            }

            dev->intf_rslt = dev->write(reg_addr[0], temp_buff, temp_len, dev->intf_ptr);

            if (dev->intf_rslt != BME280_INTF_RET_SUCCESS) {
                rslt = BME280_E_COMM_FAIL;
            }
        } else {
            rslt = BME280_E_INVALID_LEN;
        }
    } else {
        rslt = BME280_E_NULL_PTR;
    }

    return rslt;
}

static uint32_t compensate_pressure(const struct bme280_uncomp_data *uncomp_data,
                                    const struct bme280_calib_data *calib_data)
{
    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    uint32_t var5;
    uint32_t pressure;
    uint32_t pressure_min = 30000;
    uint32_t pressure_max = 110000;

    var1 = (((int32_t)calib_data->t_fine) / 2) - (int32_t)64000;
    var2 = (((var1 / 4) * (var1 / 4)) / 2048) * ((int32_t)calib_data->dig_p6);
    var2 = var2 + ((var1 * ((int32_t)calib_data->dig_p5)) * 2);
    var2 = (var2 / 4) + (((int32_t)calib_data->dig_p4) * 65536);
    var3 = (calib_data->dig_p3 * (((var1 / 4) * (var1 / 4)) / 8192)) / 8;
    var4 = (((int32_t)calib_data->dig_p2) * var1) / 2;
    var1 = (var3 + var4) / 262144;
    var1 = (((32768 + var1)) * ((int32_t)calib_data->dig_p1)) / 32768;

    if (var1)
    {
        var5 = (uint32_t)((uint32_t)1048576) - uncomp_data->pressure;
        pressure = ((uint32_t)(var5 - (uint32_t)(var2 / 4096))) * 3125;

        if (pressure < 0x80000000)
        {
            pressure = (pressure << 1) / ((uint32_t)var1);
        }
        else
        {
            pressure = (pressure / (uint32_t)var1) * 2;
        }

        var1 = (((int32_t)calib_data->dig_p9) * ((int32_t)(((pressure / 8) * (pressure / 8)) / 8192))) / 4096;
        var2 = (((int32_t)(pressure / 4)) * ((int32_t)calib_data->dig_p8)) / 8192;
        pressure = (uint32_t)((int32_t)pressure + ((var1 + var2 + calib_data->dig_p7) / 16));

        if (pressure < pressure_min)
        {
            pressure = pressure_min;
        }
        else if (pressure > pressure_max)
        {
            pressure = pressure_max;
        }
    }
    else
    {
        pressure = pressure_min;
    }

    return pressure;
}

static uint32_t compensate_humidity(const struct bme280_uncomp_data *uncomp_data,
                                    const struct bme280_calib_data *calib_data)
{
    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    int32_t var5;
    uint32_t humidity;
    uint32_t humidity_max = 102400;

    var1 = calib_data->t_fine - ((int32_t)76800);
    var2 = (int32_t)(uncomp_data->humidity * 16384);
    var3 = (int32_t)(((int32_t)calib_data->dig_h4) * 1048576);
    var4 = ((int32_t)calib_data->dig_h5) * var1;
    var5 = (((var2 - var3) - var4) + (int32_t)16384) / 32768;
    var2 = (var1 * ((int32_t)calib_data->dig_h6)) / 1024;
    var3 = (var1 * ((int32_t)calib_data->dig_h3)) / 2048;
    var4 = ((var2 * (var3 + (int32_t)32768)) / 1024) + (int32_t)2097152;
    var2 = ((var4 * ((int32_t)calib_data->dig_h2)) + 8192) / 16384;
    var3 = var5 * var2;
    var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
    var5 = var3 - ((var4 * ((int32_t)calib_data->dig_h1)) / 16);
    var5 = (var5 < 0 ? 0 : var5);
    var5 = (var5 > 419430400 ? 419430400 : var5);
    humidity = (uint32_t)(var5 / 4096);

    if (humidity > humidity_max)
    {
        humidity = humidity_max;
    }

    return humidity;
}

static void parse_sensor_data(const uint8_t *reg_data, struct bme280_uncomp_data *uncomp_data)
{

    uint32_t data_xlsb;
    uint32_t data_lsb;
    uint32_t data_msb;

    data_msb = (uint32_t)reg_data[0] << BME280_12_BIT_SHIFT;
    data_lsb = (uint32_t)reg_data[1] << BME280_4_BIT_SHIFT;
    data_xlsb = (uint32_t)reg_data[2] >> BME280_4_BIT_SHIFT;
    uncomp_data->pressure = data_msb | data_lsb | data_xlsb;

    data_msb = (uint32_t)reg_data[3] << BME280_12_BIT_SHIFT;
    data_lsb = (uint32_t)reg_data[4] << BME280_4_BIT_SHIFT;
    data_xlsb = (uint32_t)reg_data[5] >> BME280_4_BIT_SHIFT;
    uncomp_data->temperature = data_msb | data_lsb | data_xlsb;

    data_msb = (uint32_t)reg_data[6] << BME280_8_BIT_SHIFT;
    data_lsb = (uint32_t)reg_data[7];
    uncomp_data->humidity = data_msb | data_lsb;
}

static void parse_temp_press_calib_data(const uint8_t *reg_data, struct bme280_dev *dev)
{
    struct bme280_calib_data *calib_data = &dev->calib_data;

    calib_data->dig_t1 = BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
    calib_data->dig_t2 = (int16_t)BME280_CONCAT_BYTES(reg_data[3], reg_data[2]);
    calib_data->dig_t3 = (int16_t)BME280_CONCAT_BYTES(reg_data[5], reg_data[4]);
    calib_data->dig_p1 = BME280_CONCAT_BYTES(reg_data[7], reg_data[6]);
    calib_data->dig_p2 = (int16_t)BME280_CONCAT_BYTES(reg_data[9], reg_data[8]);
    calib_data->dig_p3 = (int16_t)BME280_CONCAT_BYTES(reg_data[11], reg_data[10]);
    calib_data->dig_p4 = (int16_t)BME280_CONCAT_BYTES(reg_data[13], reg_data[12]);
    calib_data->dig_p5 = (int16_t)BME280_CONCAT_BYTES(reg_data[15], reg_data[14]);
    calib_data->dig_p6 = (int16_t)BME280_CONCAT_BYTES(reg_data[17], reg_data[16]);
    calib_data->dig_p7 = (int16_t)BME280_CONCAT_BYTES(reg_data[19], reg_data[18]);
    calib_data->dig_p8 = (int16_t)BME280_CONCAT_BYTES(reg_data[21], reg_data[20]);
    calib_data->dig_p9 = (int16_t)BME280_CONCAT_BYTES(reg_data[23], reg_data[22]);
    calib_data->dig_h1 = reg_data[25];
}

static void parse_humidity_calib_data(const uint8_t *reg_data, struct bme280_dev *dev)
{
    struct bme280_calib_data *calib_data = &dev->calib_data;
    int16_t dig_h4_lsb;
    int16_t dig_h4_msb;
    int16_t dig_h5_lsb;
    int16_t dig_h5_msb;

    calib_data->dig_h2 = (int16_t)BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
    calib_data->dig_h3 = reg_data[2];
    dig_h4_msb = (int16_t)(int8_t)reg_data[3] * 16;
    dig_h4_lsb = (int16_t)(reg_data[4] & 0x0F);
    calib_data->dig_h4 = dig_h4_msb | dig_h4_lsb;
    dig_h5_msb = (int16_t)(int8_t)reg_data[5] * 16;
    dig_h5_lsb = (int16_t)(reg_data[4] >> 4);
    calib_data->dig_h5 = dig_h5_msb | dig_h5_lsb;
    calib_data->dig_h6 = (int8_t)reg_data[6];
}

static uint8_t are_settings_changed(uint8_t sub_settings, uint8_t desired_settings)
{
    uint8_t settings_changed = FALSE;

    if (sub_settings & desired_settings)
    {

        settings_changed = TRUE;
    }
    else
    {

        settings_changed = FALSE;
    }

    return settings_changed;
}

static int8_t set_osr_press_temp_settings(uint8_t desired_settings,
                                          const struct bme280_settings *settings,
                                          struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t reg_addr = BME280_REG_CTRL_MEAS;
    uint8_t reg_data;

    rslt = bme280_get_regs(reg_addr, &reg_data, 1, dev);

    if (rslt == BME280_OK)
    {
        if (desired_settings & BME280_SEL_OSR_PRESS)
        {
            fill_osr_press_settings(&reg_data, settings);
        }

        if (desired_settings & BME280_SEL_OSR_TEMP)
        {
            fill_osr_temp_settings(&reg_data, settings);
        }

        rslt = bme280_set_regs(&reg_addr, &reg_data, 1, dev);
    }

    return rslt;
}

static int8_t set_osr_settings(uint8_t desired_settings, const struct bme280_settings *settings, struct bme280_dev *dev)
{
    int8_t rslt = BME280_W_INVALID_OSR_MACRO;

    if (desired_settings & BME280_SEL_OSR_HUM)
    {
        rslt = set_osr_humidity_settings(settings, dev);
    }

    if (desired_settings & (BME280_SEL_OSR_PRESS | BME280_SEL_OSR_TEMP))
    {
        rslt = set_osr_press_temp_settings(desired_settings, settings, dev);
    }

    return rslt;
}

static int8_t set_osr_humidity_settings(const struct bme280_settings *settings, struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t ctrl_hum;
    uint8_t ctrl_meas;
    uint8_t reg_addr = BME280_REG_CTRL_HUM;

    ctrl_hum = settings->osr_h & BME280_CTRL_HUM_MSK;

    rslt = bme280_set_regs(&reg_addr, &ctrl_hum, 1, dev);

    if (rslt == BME280_OK)
    {
        reg_addr = BME280_REG_CTRL_MEAS;
        rslt = bme280_get_regs(reg_addr, &ctrl_meas, 1, dev);

        if (rslt == BME280_OK)
        {
            rslt = bme280_set_regs(&reg_addr, &ctrl_meas, 1, dev);
        }
    }

    return rslt;
}

static int8_t set_filter_standby_settings(uint8_t desired_settings,
                                          const struct bme280_settings *settings,
                                          struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t reg_addr = BME280_REG_CONFIG;
    uint8_t reg_data;

    rslt = bme280_get_regs(reg_addr, &reg_data, 1, dev);

    if (rslt == BME280_OK)
    {
        if (desired_settings & BME280_SEL_FILTER)
        {
            fill_filter_settings(&reg_data, settings);
        }

        if (desired_settings & BME280_SEL_STANDBY)
        {
            fill_standby_settings(&reg_data, settings);
        }

        rslt = bme280_set_regs(&reg_addr, &reg_data, 1, dev);
    }

    return rslt;
}

static void fill_filter_settings(uint8_t *reg_data, const struct bme280_settings *settings)
{
    *reg_data = BME280_SET_BITS(*reg_data, BME280_FILTER, settings->filter);
}

static void fill_standby_settings(uint8_t *reg_data, const struct bme280_settings *settings)
{
    *reg_data = BME280_SET_BITS(*reg_data, BME280_STANDBY, settings->standby_time);
}

static void fill_osr_press_settings(uint8_t *reg_data, const struct bme280_settings *settings)
{
    *reg_data = BME280_SET_BITS(*reg_data, BME280_CTRL_PRESS, settings->osr_p);
}

static void fill_osr_temp_settings(uint8_t *reg_data, const struct bme280_settings *settings)
{
    *reg_data = BME280_SET_BITS(*reg_data, BME280_CTRL_TEMP, settings->osr_t);
}

static int8_t get_calib_data(struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t reg_addr = BME280_REG_TEMP_PRESS_CALIB_DATA;

    uint8_t calib_data[BME280_LEN_TEMP_PRESS_CALIB_DATA] = { 0 };

    rslt = bme280_get_regs(reg_addr, calib_data, BME280_LEN_TEMP_PRESS_CALIB_DATA, dev);

    if (rslt == BME280_OK)
    {

        parse_temp_press_calib_data(calib_data, dev);
        reg_addr = BME280_REG_HUMIDITY_CALIB_DATA;

        rslt = bme280_get_regs(reg_addr, calib_data, BME280_LEN_HUMIDITY_CALIB_DATA, dev);

        if (rslt == BME280_OK)
        {

            parse_humidity_calib_data(calib_data, dev);
        }
    }

    return rslt;
}

static void interleave_reg_addr(const uint8_t *reg_addr, uint8_t *temp_buff, const uint8_t *reg_data, uint32_t len)
{
    uint32_t index;

    for (index = 1; index < len; index++)
    {
        temp_buff[(index * 2) - 1] = reg_addr[index];
        temp_buff[index * 2] = reg_data[index];
    }
}

int8_t bme280_soft_reset(struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t reg_addr = BME280_REG_RESET;
    uint8_t status_reg = 0;
    uint8_t try_run = 5;

    uint8_t soft_rst_cmd = BME280_SOFT_RESET_COMMAND;

    rslt = bme280_set_regs(&reg_addr, &soft_rst_cmd, 1, dev);

    if (rslt == BME280_OK)
    {

        do
        {

            dev->delay_us(BME280_STARTUP_DELAY, dev->intf_ptr);
            rslt = bme280_get_regs(BME280_REG_STATUS, &status_reg, 1, dev);

        } while ((rslt == BME280_OK) && (try_run--) && (status_reg & BME280_STATUS_IM_UPDATE));

        if (status_reg & BME280_STATUS_IM_UPDATE)
        {
            rslt = BME280_E_NVM_COPY_FAILED;
        }
    }

    return rslt;
}

int8_t bme280_set_sensor_settings(uint8_t desired_settings,
                                  const struct bme280_settings *settings,
                                  struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t sensor_mode;

    if (settings != NULL)
    {
        rslt = bme280_get_sensor_mode(&sensor_mode, dev);

        if ((rslt == BME280_OK) && (sensor_mode != BME280_POWERMODE_SLEEP))
        {
            rslt = put_device_to_sleep(dev);
        }

        if (rslt == BME280_OK)
        {

            if (are_settings_changed(OVERSAMPLING_SETTINGS, desired_settings))
            {
                rslt = set_osr_settings(desired_settings, settings, dev);
            }

            if ((rslt == BME280_OK) && are_settings_changed(FILTER_STANDBY_SETTINGS, desired_settings))
            {
                rslt = set_filter_standby_settings(desired_settings, settings, dev);
            }
        }
    }
    else
    {
        rslt = BME280_E_NULL_PTR;
    }

    return rslt;
}

int8_t bme280_get_sensor_settings(struct bme280_settings *settings, struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t reg_data[4];

    if (settings != NULL)
    {
        rslt = bme280_get_regs(BME280_REG_CTRL_HUM, reg_data, 4, dev);

        if (rslt == BME280_OK)
        {
            parse_device_settings(reg_data, settings);
        }
    }
    else
    {
        rslt = BME280_E_NULL_PTR;
    }

    return rslt;
}

int8_t bme280_set_sensor_mode(uint8_t sensor_mode, struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t last_set_mode;

    rslt = bme280_get_sensor_mode(&last_set_mode, dev);

    if ((rslt == BME280_OK) && (last_set_mode != BME280_POWERMODE_SLEEP))
    {
        rslt = put_device_to_sleep(dev);
    }

    if (rslt == BME280_OK)
    {
        rslt = write_power_mode(sensor_mode, dev);
    }

    return rslt;
}

int8_t bme280_get_sensor_mode(uint8_t *sensor_mode, struct bme280_dev *dev)
{
    int8_t rslt;

    if (sensor_mode != NULL)
    {

        rslt = bme280_get_regs(BME280_REG_PWR_CTRL, sensor_mode, 1, dev);

        *sensor_mode = BME280_GET_BITS_POS_0(*sensor_mode, BME280_SENSOR_MODE);
    }
    else
    {
        rslt = BME280_E_NULL_PTR;
    }

    return rslt;
}

static int8_t write_power_mode(uint8_t sensor_mode, struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t reg_addr = BME280_REG_PWR_CTRL;

    uint8_t sensor_mode_reg_val;

    rslt = bme280_get_regs(reg_addr, &sensor_mode_reg_val, 1, dev);

    if (rslt == BME280_OK)
    {
        sensor_mode_reg_val = BME280_SET_BITS_POS_0(sensor_mode_reg_val, BME280_SENSOR_MODE, sensor_mode);

        rslt = bme280_set_regs(&reg_addr, &sensor_mode_reg_val, 1, dev);
    }

    return rslt;
}

static int8_t put_device_to_sleep(struct bme280_dev *dev)
{
    int8_t rslt;
    uint8_t reg_data[4];
    struct bme280_settings settings;

    rslt = bme280_get_regs(BME280_REG_CTRL_HUM, reg_data, 4, dev);

    if (rslt == BME280_OK)
    {
        parse_device_settings(reg_data, &settings);
        rslt = bme280_soft_reset(dev);

        if (rslt == BME280_OK)
        {
            rslt = reload_device_settings(&settings, dev);
        }
    }

    return rslt;
}

int8_t bme280_get_sensor_data(uint8_t sensor_comp, struct bme280_data *comp_data, struct bme280_dev *dev)
{
    int8_t rslt;

    uint8_t reg_data[BME280_LEN_P_T_H_DATA] = { 0 };
    struct bme280_uncomp_data uncomp_data = { 0 };

    if (comp_data != NULL)
    {

        rslt = bme280_get_regs(BME280_REG_DATA, reg_data, BME280_LEN_P_T_H_DATA, dev);

        if (rslt == BME280_OK)
        {

            parse_sensor_data(reg_data, &uncomp_data);

            rslt = bme280_compensate_data(sensor_comp, &uncomp_data, comp_data, &dev->calib_data);
        }
    }
    else
    {
        rslt = BME280_E_NULL_PTR;
    }

    return rslt;
}

int8_t bme280_compensate_data(uint8_t sensor_comp,
                              const struct bme280_uncomp_data *uncomp_data,
                              struct bme280_data *comp_data,
                              struct bme280_calib_data *calib_data)
{
    int8_t rslt = BME280_OK;

    if ((uncomp_data != NULL) && (comp_data != NULL) && (calib_data != NULL))
    {

        comp_data->temperature = 0;
        comp_data->pressure = 0;
        comp_data->humidity = 0;

        if (sensor_comp & (BME280_PRESS | BME280_TEMP | BME280_HUM))
        {

            comp_data->temperature = compensate_temperature(uncomp_data, calib_data);
        }

        if (sensor_comp & BME280_PRESS)
        {

            comp_data->pressure = compensate_pressure(uncomp_data, calib_data);
        }

        if (sensor_comp & BME280_HUM)
        {

            comp_data->humidity = compensate_humidity(uncomp_data, calib_data);
        }
    }
    else
    {
        rslt = BME280_E_NULL_PTR;
    }

    return rslt;
}

static double compensate_temperature(const struct bme280_uncomp_data *uncomp_data, struct bme280_calib_data *calib_data)
{
    double var1;
    double var2;
    double temperature;
    double temperature_min = -40;
    double temperature_max = 85;

    var1 = (((double)uncomp_data->temperature) / 16384.0 - ((double)calib_data->dig_t1) / 1024.0);
    var1 = var1 * ((double)calib_data->dig_t2);
    var2 = (((double)uncomp_data->temperature) / 131072.0 - ((double)calib_data->dig_t1) / 8192.0);
    var2 = (var2 * var2) * ((double)calib_data->dig_t3);
    calib_data->t_fine = (int32_t)(var1 + var2);
    temperature = (var1 + var2) / 5120.0;

    if (temperature < temperature_min)
    {
        temperature = temperature_min;
    }
    else if (temperature > temperature_max)
    {
        temperature = temperature_max;
    }

    return temperature;
}

static int8_t null_ptr_check(const struct bme280_dev *dev)
{
    int8_t rslt;

    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_us == NULL))
    {

        rslt = BME280_E_NULL_PTR;
    }
    else
    {

        rslt = BME280_OK;
    }

    return rslt;
}

