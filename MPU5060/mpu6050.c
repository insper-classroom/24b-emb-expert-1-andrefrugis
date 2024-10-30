#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "mpu6050.h" 
// Configura o struct de configuração do MPU6050
void mpu6050_set_config(imu_c *config, i2c_inst_t *i2c, int pin_sda, int pin_scl, int acc_scale) {
    config->i2c = i2c;
    config->pin_sda = pin_sda;
    config->pin_scl = pin_scl;
    config->acc_scale = acc_scale;
}

// Configura pinos e periféricos I2C
int mpu6050_init(imu_c config) {
    // Inicializa os pinos I2C
    i2c_init(config.i2c, 100 * 1000); // 100 kHz
    gpio_set_function(config.pin_sda, GPIO_FUNC_I2C);
    gpio_set_function(config.pin_scl, GPIO_FUNC_I2C);
    gpio_pull_up(config.pin_sda);
    gpio_pull_up(config.pin_scl);

    // Aguarda um curto período para garantir que o dispositivo esteja pronto
    sleep_ms(100);

    // Acorda o MPU6050 escrevendo 0 no registrador PWR_MGMT_1
    uint8_t buf[2];
    buf[0] = MPU6050_PWR_MGMT_1;
    buf[1] = 0x00; // Define sleep = 0
    int ret = i2c_write_blocking(config.i2c, MPU6050_I2C_ADDRESS, buf, 2, false);
    if (ret != 2) {
        return 0; // Falha
    }

    // Configura a escala do acelerômetro
    buf[0] = MPU6050_ACCEL_CONFIG;
    switch (config.acc_scale) {
        case 2:
            buf[1] = 0x00; // +/- 2g
            break;
        case 4:
            buf[1] = 0x08; // +/- 4g
            break;
        case 8:
            buf[1] = 0x10; // +/- 8g
            break;
        case 16:
            buf[1] = 0x18; // +/- 16g
            break;
        default:
            buf[1] = 0x00; // Padrão para +/- 2g
            break;
    }
    ret = i2c_write_blocking(config.i2c, MPU6050_I2C_ADDRESS, buf, 2, false);
    if (ret != 2) {
        return 0; // Falha
    }

    return 1; // Sucesso
}

// Reinicia o dispositivo para o estado original
int mpu6050_reset(imu_c config) {
    uint8_t buf[2];
    buf[0] = MPU6050_PWR_MGMT_1;
    buf[1] = 0x80; // Define o bit de reset
    int ret = i2c_write_blocking(config.i2c, MPU6050_I2C_ADDRESS, buf, 2, false);
    if (ret != 2) {
        return 0; // Falha
    }

    // Aguarda um curto período para permitir que o reset seja concluído
    sleep_ms(100);

    return 1; // Sucesso
}

// Lê os valores do acelerômetro
int mpu6050_read_acc(imu_c config, int16_t accel[3]) {
    uint8_t buffer[6];
    uint8_t reg = MPU6050_ACCEL_XOUT_H;
    int ret = i2c_write_blocking(config.i2c, MPU6050_I2C_ADDRESS, &reg, 1, true); // Envia o endereço do registrador com start repetido
    if (ret != 1) {
        return 0; // Falha
    }
    ret = i2c_read_blocking(config.i2c, MPU6050_I2C_ADDRESS, buffer, 6, false); // Lê 6 bytes
    if (ret != 6) {
        return 0; // Falha
    }

    // Combina os bytes altos e baixos
    accel[0] = (buffer[0] << 8) | buffer[1]; // Eixo X
    accel[1] = (buffer[2] << 8) | buffer[3]; // Eixo Y
    accel[2] = (buffer[4] << 8) | buffer[5]; // Eixo Z

    return 1; // Sucesso
}

// Lê os valores do giroscópio
int mpu6050_read_gyro(imu_c config, int16_t gyro[3]) {
    uint8_t buffer[6];
    uint8_t reg = MPU6050_GYRO_XOUT_H;
    int ret = i2c_write_blocking(config.i2c, MPU6050_I2C_ADDRESS, &reg, 1, true); // Envia o endereço do registrador com start repetido
    if (ret != 1) {
        return 0; // Falha
    }
    ret = i2c_read_blocking(config.i2c, MPU6050_I2C_ADDRESS, buffer, 6, false); // Lê 6 bytes
    if (ret != 6) {
        return 0; // Falha
    }

    // Combina os bytes altos e baixos
    gyro[0] = (buffer[0] << 8) | buffer[1]; // Eixo X
    gyro[1] = (buffer[2] << 8) | buffer[3]; // Eixo Y
    gyro[2] = (buffer[4] << 8) | buffer[5]; // Eixo Z

    return 1; // Sucesso
}

// Lê o valor da temperatura
int mpu6050_read_temp(imu_c config, int16_t *temp) {
    uint8_t buffer[2];
    uint8_t reg = MPU6050_TEMP_OUT_H;
    int ret = i2c_write_blocking(config.i2c, MPU6050_I2C_ADDRESS, &reg, 1, true); // Envia o endereço do registrador com start repetido
    if (ret != 1) {
        return 0; // Falha
    }
    ret = i2c_read_blocking(config.i2c, MPU6050_I2C_ADDRESS, buffer, 2, false); // Lê 2 bytes
    if (ret != 2) {
        return 0; // Falha
    }

    // Combina os bytes alto e baixo
    *temp = (buffer[0] << 8) | buffer[1];

    return 1; // Sucesso
}
