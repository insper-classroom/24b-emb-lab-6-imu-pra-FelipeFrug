#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "pico/stdlib.h"
#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "mpu6050.h"

#include <Fusion.h>

#define UART_ID uart0

const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 4;
const int I2C_SCL_GPIO = 5;

typedef struct {
    int8_t x;
    int8_t y;
} mouse_data_t;

QueueHandle_t xQueueMouse;

static void mpu6050_reset() {
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[6];
    uint8_t val = 0x3B;

    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    val = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    val = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 2, false);

    *temp = buffer[0] << 8 | buffer[1];
}

void mpu6050_task(void *p) {
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    mpu6050_reset();

    int16_t accel[3], gyro[3], temp;
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);


    while(1) {
        mpu6050_read_raw(accel, gyro, &temp);

        FusionVector gyroscope;
        gyroscope.axis.x = gyro[0] / 131.0f;
        gyroscope.axis.y = gyro[1] / 131.0f;
        gyroscope.axis.z = gyro[2] / 131.0f;

        FusionVector accelerometer;
        accelerometer.axis.x = accel[0] / 16384.0f;
        accelerometer.axis.y = accel[1] / 16384.0f;
        accelerometer.axis.z = accel[2] / 16384.0f;

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, 0.01f);

        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        uart_putc_raw(UART_ID, 0);
        uart_putc_raw(UART_ID, (int) euler.angle.pitch & 0xFF);
        uart_putc_raw(UART_ID, ((int) euler.angle.pitch >> 8) & 0xFF);
        uart_putc_raw(UART_ID, -1);

        uart_putc_raw(UART_ID, 1);
        uart_putc_raw(UART_ID, (int) euler.angle.roll & 0xFF);
        uart_putc_raw(UART_ID, ((int) euler.angle.roll >> 8) & 0xFF);
        uart_putc_raw(UART_ID, -1);

        // printf("Acc. X = %d, Y = %d, Z = %d - ", accel[0], accel[1], accel[2]);
        // printf("Gyro. X = %d, Y = %d, Z = %d - ", gyro[0], gyro[1], gyro[2]);
        // printf("Temp. = %f\n\n\n", (temp / 340.0) + 36.53);

        int mod = abs(accel[1]);
        
        if (mod > 17000){
            uart_putc_raw(UART_ID, 2);
            uart_putc_raw(UART_ID, 0);
            uart_putc_raw(UART_ID, ((mod >> 8) & 0xFF));
            uart_putc_raw(UART_ID, -1);
        }

        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}

void main() {
    stdio_init_all();

    xQueueMouse = xQueueCreate(10, sizeof(mouse_data_t));

    xTaskCreate(mpu6050_task, "mpu6050_task", 4096, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true);
}