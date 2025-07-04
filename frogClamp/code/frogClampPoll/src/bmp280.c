#include "stm8s.h"
#include "bmp280.h"

/* Read registers from BME280 */
static int8_t get_regs(uint8_t reg_addr, uint8_t *data, uint8_t len, struct bme280_dev *dev) {
    dev->intf_rslt = dev->read(reg_addr, data, len, dev->intf_ptr);
    return dev->intf_rslt == 0 ? BME280_OK : BME280_E_COMM_FAIL;
}

/* Write registers to BME280 */
static int8_t set_regs(uint8_t reg_addr, const uint8_t *data, uint8_t len, struct bme280_dev *dev) {
    dev->intf_rslt = dev->write(reg_addr, data, len, dev->intf_ptr);
    return dev->intf_rslt == 0 ? BME280_OK : BME280_E_COMM_FAIL;
}

/* Parse calibration data (temperature and pressure only) */
static void parse_calib_data(const uint8_t *reg_data, struct bme280_dev *dev) {
    struct bme280_calib_data *cal = &dev->calib_data;
    cal->dig_t1 = (reg_data[1] << 8) | reg_data[0];
    cal->dig_t2 = (int16_t)((reg_data[3] << 8) | reg_data[2]);
    cal->dig_t3 = (int16_t)((reg_data[5] << 8) | reg_data[4]);
    cal->dig_p1 = (reg_data[7] << 8) | reg_data[6];
    cal->dig_p2 = (int16_t)((reg_data[9] << 8) | reg_data[8]);
    cal->dig_p3 = (int16_t)((reg_data[11] << 8) | reg_data[10]);
    cal->dig_p4 = (int16_t)((reg_data[13] << 8) | reg_data[12]);
    cal->dig_p5 = (int16_t)((reg_data[15] << 8) | reg_data[14]);
    cal->dig_p6 = (int16_t)((reg_data[17] << 8) | reg_data[16]);
    cal->dig_p7 = (int16_t)((reg_data[19] << 8) | reg_data[18]);
    cal->dig_p8 = (int16_t)((reg_data[21] << 8) | reg_data[20]);
    cal->dig_p9 = (int16_t)((reg_data[23] << 8) | reg_data[22]);
}

/* Get calibration data */
static int8_t get_calib_data(struct bme280_dev *dev) {
    uint8_t calib_data[26];
    int8_t rslt = get_regs(BME280_REG_TEMP_PRESS_CALIB_DATA, calib_data, 26, dev);
    if (rslt == BME280_OK) {
        parse_calib_data(calib_data, dev);
    }
    return rslt;
}

/* Compensate temperature (integer, minimal) */
static int32_t compensate_temperature(const struct bme280_uncomp_data *uncomp_data, struct bme280_calib_data *cal) {
    int32_t var1, var2, temp = uncomp_data->temperature;
    var1 = ((temp >> 3) - ((int32_t)cal->dig_t1 << 1)) * cal->dig_t2 >> 11;
    var2 = (((temp >> 4) - cal->dig_t1) * ((temp >> 4) - cal->dig_t1) >> 12) * cal->dig_t3 >> 14;
    cal->t_fine = var1 + var2;
    temp = (cal->t_fine * 5 + 128) >> 8;
    return temp < -4000 ? -4000 : temp > 8500 ? 8500 : temp;
}

/* Compensate pressure (integer, 32-bit) */
static uint32_t compensate_pressure(const struct bme280_uncomp_data *uncomp_data, const struct bme280_calib_data *cal) {
    int32_t var1, var2, var3, var4;
    uint32_t press = uncomp_data->pressure;
    var1 = (cal->t_fine >> 1) - 64000;
    var2 = (var1 >> 2) * (var1 >> 2) * cal->dig_p6 >> 11;
    var2 = var2 + (var1 * cal->dig_p5 << 1);
    var2 = (var2 >> 2) + (cal->dig_p4 << 16);
    var3 = cal->dig_p3 * (var1 >> 2) * (var1 >> 2) >> 13;
    var4 = cal->dig_p2 * var1 >> 1;
    var1 = ((var3 + var4) >> 10) + 32768;
    var1 = (var1 * (int32_t)cal->dig_p1) >> 15;
    if (var1 == 0) return 30000; /* Avoid division by zero */
    press = ((1048576 - press) - (var2 >> 12)) * 3125;
    press = press < 0x80000000 ? (press << 1) / var1 : (press / var1) * 2;
    var1 = cal->dig_p9 * (int32_t)((press >> 3) * (press >> 3)) >> 12;
    var2 = (int32_t)(press >> 2) * cal->dig_p8 >> 13;
    press = (uint32_t)(press + ((var1 + var2 + cal->dig_p7) >> 4));
    return press < 30000 ? 30000 : press > 110000 ? 110000 : press;
}

/* Initialize BME280 */
int8_t bme280_init(struct bme280_dev *dev) {
    uint8_t chip_id;
    int8_t rslt = get_regs(BME280_REG_CHIP_ID, &chip_id, 1, dev);
    if (rslt == BME280_OK && chip_id == BME280_CHIP_ID) {
        uint8_t cmd = BME280_SOFT_RESET_COMMAND;
        dev->chip_id = chip_id;
        rslt = set_regs(BME280_REG_RESET, &cmd, 1, dev);
        if (rslt == BME280_OK) {
            dev->delay_us(2000); /* 2 ms startup */
            rslt = get_calib_data(dev);
            if (rslt == BME280_OK) {
                /* Hardcoded settings: osr_p=1x, osr_t=1x, forced mode */
                uint8_t reg_data = (1 << 5) | (1 << 2) | BME280_POWERMODE_FORCED;
                rslt = set_regs(BME280_REG_CTRL_MEAS, &reg_data, 1, dev);
            }
        }
    } else {
        rslt = BME280_E_DEV_NOT_FOUND;
    }
    return rslt;
}

/* Read pressure data */
int8_t bme280_get_sensor_data(struct bme280_data *comp_data, struct bme280_dev *dev) {
    uint8_t reg_data[6];
    struct bme280_uncomp_data uncomp_data;
    int8_t rslt = get_regs(BME280_REG_DATA, reg_data, 6, dev);
    if (rslt == BME280_OK) {
        uncomp_data.pressure = (uint32_t)reg_data[0] << 12 | (uint32_t)reg_data[1] << 4 | reg_data[2] >> 4;
        uncomp_data.temperature = (uint32_t)reg_data[3] << 12 | (uint32_t)reg_data[4] << 4 | reg_data[5] >> 4;
        comp_data->temperature = compensate_temperature(&uncomp_data, &dev->calib_data);
        comp_data->pressure = compensate_pressure(&uncomp_data, &dev->calib_data);
    }
    return rslt;
}