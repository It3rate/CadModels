#ifndef BME280_H
#define BME280_H

#include <stdint.h>

#define BME280_I2C_ADDR_PRIM                      UINT8_C(0x76)
#define BME280_I2C_ADDR_SEC                       UINT8_C(0x77)

#define BME280_OK 0
#define BME280_E_COMM_FAIL -1
#define BME280_E_DEV_NOT_FOUND -2
#define BME280_INTF_RET_SUCCESS 0
#define BME280_CHIP_ID 0x60
#define BME280_REG_CHIP_ID 0xD0
#define BME280_REG_RESET 0xE0
#define BME280_SOFT_RESET_COMMAND 0xB6
#define BME280_REG_CTRL_MEAS 0xF4
#define BME280_REG_DATA 0xF7
#define BME280_REG_TEMP_PRESS_CALIB_DATA 0x88
#define BME280_POWERMODE_FORCED 0x01

struct bme280_calib_data {
    uint16_t dig_t1, dig_p1;
    int16_t dig_t2, dig_t3, dig_p2, dig_p3, dig_p4, dig_p5, dig_p6, dig_p7, dig_p8, dig_p9;
    int32_t t_fine;
};

struct bme280_uncomp_data {
    uint32_t pressure, temperature;
};

struct bme280_data {
    uint32_t pressure;
    int32_t temperature;
};

struct bme280_dev {
    uint8_t chip_id, intf_rslt;
    void *intf_ptr;
    struct bme280_calib_data calib_data;
    int8_t (*read)(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr);
    int8_t (*write)(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr);
    void (*delay_us)(uint32_t period);
};

int8_t bme280_init(struct bme280_dev *dev);
int8_t bme280_get_sensor_data(struct bme280_data *comp_data, struct bme280_dev *dev);

#endif