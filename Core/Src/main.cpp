#include "main.h"
#include "gpio.h"
#include "i2c.h"
#include "i2c_device.hpp"
#include "mpu6050.hpp"
#include "mpu6050_dmp.hpp"
#include "system_clock.h"
#include "usart.h"
#include <cstdio>
#include <utility>

static auto timer_elapsed{false};

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == MPU_INT_Pin) {
        timer_elapsed = true;
    }
}

int main()
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_I2C1_Init();

    using namespace Utility;
    using MPU6050 = MPU6050::MPU6050;

    I2CDevice i2c_device{&hi2c1, std::to_underlying(MPU6050::DevAddress::AD0_LOW)};

    MPU6050 mpu6050{std::move(i2c_device),
                    200U,
                    MPU6050::GyroRange::GYRO_FS_250,
                    MPU6050::AccelRange::ACCEL_FS_2,
                    MPU6050::DLPF::BW_256,
                    MPU6050::DHPF::DHPF_RESET};

    using namespace MPU6050;
    DMP mpu6050_dmp{std::move(mpu6050)};

    while (true) {
        if (timer_elapsed) {
            auto const& [roll, pitch, yaw]{mpu6050_dmp.get_roll_pitch_yaw()};
            printf("roll: %f, pitch %f, yaw: %f\n\r", roll, pitch, yaw);
            timer_elapsed = false;
        }
    }

    return 0;
}
