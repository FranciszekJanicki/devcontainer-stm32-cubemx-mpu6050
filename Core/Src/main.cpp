#include "main.h"
#include "gpio.h"
#include "i2c.h"
#include "i2c_device.hpp"
#include "mpu6050.hpp"
#include "mpu6050_config.hpp"
#include "mpu6050_register.hpp"
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

    using namespace MPU6050;

    auto i2c_device = I2CDevice{&hi2c1, std::to_underlying(DevAddress::AD0_LOW)};

    auto config =
        CONFIG{.ext_sync_set = std::to_underlying(ExtSync::DISABLED), .dlpf_cfg = std::to_underlying(DLPF::BW_256)};

    auto accel_config = ACCEL_CONFIG{.xa_st = false,
                                     .ya_st = false,
                                     .za_st = false,
                                     .afs_sel = accel_range_to_scale(AccelRange::ACCEL_FS_2),
                                     .accel_hpf = std::to_underlying(DHPF::DHPF_5)};

    auto gyro_config = GYRO_CONFIG{.xg_st = false,
                                   .yg_st = false,
                                   .zg_st = false,
                                   .fs_sel = gyro_range_to_scale(GyroRange::GYRO_FS_250)};

    auto smplrt_div = SMPLRT_DIV{.smplrt_div = smplrt_to_divider(8000U, DLPF::BW_5)};

    auto int_pin_cfg = INT_PIN_CFG{.int_level = std::to_underlying(IntMode::ACTIVELOW),
                                   .int_open = std::to_underlying(IntDrive::PUSHPULL),
                                   .latch_int_en = std::to_underlying(IntLatch::PULSE50US),
                                   .int_rd_clear = std::to_underlying(IntClear::ANYREAD),
                                   .fsync_int_level = false,
                                   .fsync_int_en = false,
                                   .i2c_bypass_en = false,
                                   .clock_output = false};

    auto int_enable = INT_ENABLE{.ff_en = false,
                                 .mot_en = false,
                                 .zmot_en = false,
                                 .fifo_oflow_en = false,
                                 .i2c_mst_en = false,
                                 .pll_rdy_int_en = false,
                                 .dmp_int_en = false,
                                 .raw_rdy_int_en = true};

    auto mpu6050 =
        MPU6050::MPU6050{std::move(i2c_device), config, accel_config, gyro_config, smplrt_div, int_pin_cfg, int_enable};

    while (true) {
        if (timer_elapsed) {
            auto const& [roll, pitch, yaw]{mpu6050.get_roll_pitch_yaw().value()};
            printf("roll: %f, pitch %f, yaw: %f\n\r", roll, pitch, yaw);
            timer_elapsed = false;
        }
    }

    return 0;
}
