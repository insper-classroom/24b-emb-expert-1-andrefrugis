/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "mpu6050.h" 
// Inclui as funções do driver do MPU6050

// Define o pino GPIO conectado ao pino INT do MPU6050
#define MPU6050_INT_PIN 16

volatile int f_irq_mpu = 0;

// Função de callback para interrupções GPIO
void gpio_callback(uint gpio, uint32_t events) {
    if (gpio == MPU6050_INT_PIN) {
        f_irq_mpu = 1;
    }
}

int main() {
    stdio_init_all();

    // Configura o pino GPIO para a interrupção
    gpio_init(MPU6050_INT_PIN);
    gpio_set_dir(MPU6050_INT_PIN, GPIO_IN);
    gpio_pull_up(MPU6050_INT_PIN);
    gpio_set_irq_enabled_with_callback(MPU6050_INT_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    // Configuração do MPU6050
    imu_c imu_config;
    // Configura o MPU6050: usa i2c0, SDA em GPIO12, SCL em GPIO13, escala de 2g
    mpu6050_set_config(&imu_config, i2c0, 12, 13, 2);
    printf("ligando \n");
    // Inicializa o MPU6050
    if (!mpu6050_init(imu_config)) {
        printf("Erro ao inicializar o MPU6050\n");
        return 1;
    } else {
        printf("MPU6050 inicializado com sucesso\n");
    }

    // Configura o MPU6050 para gerar interrupção quando novos dados estiverem prontos
    // uint8_t buf[2];
    // buf[0] = 0x38; // Registrador INT_ENABLE
    // buf[1] = 0x01; // Ativa o bit DATA_RDY_EN
    // int ret = i2c_write_blocking(imu_config.i2c, MPU6050_I2C_ADDRESS, buf, 2, false);
    // if (ret != 2) {
    //     printf("Falha ao configurar interrupções no MPU6050\n");
    //     return 1;
    // }

    // Loop principal
    while (true) {
        int16_t accel[3];
        int16_t gyro[3];
        int16_t temp_raw;
        float temperature;

       // if (f_irq_mpu) {
            f_irq_mpu = 0; // Reseta a flag da interrupção

            // Leitura do acelerômetro
            if (mpu6050_read_acc(imu_config, accel)) {
                printf("Acelerômetro - X: %d, Y: %d, Z: %d\n", accel[0], accel[1], accel[2]);
            } else {
                printf("Falha ao ler o acelerômetro\n");
            }

            // Leitura do giroscópio
            if (mpu6050_read_gyro(imu_config, gyro)) {
                printf("Giroscópio - X: %d, Y: %d, Z: %d\n", gyro[0], gyro[1], gyro[2]);
            } else {
                printf("Falha ao ler o giroscópio\n");
            }

            // Leitura da temperatura
            if (mpu6050_read_temp(imu_config, &temp_raw)) {
                 float temperature = (temp_raw / 340.0) + 36.53;
                printf("Temperatura: %.2f°C\n", temperature);
            } else {
                printf("Falha ao ler a temperatura\n");
            }

            printf("-----------------------------\n");
       // }

        // Pode realizar outras tarefas aqui
        sleep_ms(10); // Pequena pausa para evitar sobrecarga da CPU
    }

    return 0;
}
