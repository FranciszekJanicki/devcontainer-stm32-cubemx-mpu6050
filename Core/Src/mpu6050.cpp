#include "mpu6050.hpp"
#include "i2c.h"
#include "i2c_device.hpp"
#include "main.h"
#include "mpu6050_register.hpp"
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <utility>

namespace MPU6050 {

    MPU6050::MPU6050(I2CDevice&& i2c_device,
                     CONFIG const config,
                     ACCEL_CONFIG const accel_config,
                     GYRO_CONFIG const gyro_config,
                     SMPLRT_DIV const smplrt_div,
                     INT_PIN_CFG const int_pin_cfg,
                     INT_ENABLE const int_enable) noexcept :
        i2c_device_{std::forward<I2CDevice>(i2c_device)},
        accel_scale_{accel_range_to_scale((AccelRange)accel_config.afs_sel)},
        gyro_scale_{gyro_range_to_scale((GyroRange)gyro_config.fs_sel)}
    {
        this->initialize(config, accel_config, gyro_config, smplrt_div, int_pin_cfg, int_enable);
    }

    MPU6050::~MPU6050() noexcept
    {
        this->deinitialize();
    }

    std::optional<Vec3D<float>> MPU6050::get_acceleration_scaled() const noexcept
    {
        return this->get_acceleration_raw().transform(
            [this](Vec3D<std::int16_t> const& raw) { return static_cast<Vec3D<float>>(raw) / this->accel_scale_; });
    }

    std::optional<float> MPU6050::get_acceleration_x_scaled() const noexcept
    {
        return this->get_acceleration_x_raw().transform(
            [this](std::int16_t const raw) { return static_cast<float>(raw) / this->accel_scale_; });
    }

    std::optional<float> MPU6050::get_acceleration_y_scaled() const noexcept
    {
        return this->get_acceleration_y_raw().transform(
            [this](std::int16_t const raw) { return static_cast<float>(raw) / this->accel_scale_; });
    }

    std::optional<float> MPU6050::get_acceleration_z_scaled() const noexcept
    {
        return this->get_acceleration_z_raw().transform(
            [this](std::int16_t const raw) { return static_cast<float>(raw) / this->accel_scale_; });
    }

    std::optional<float> MPU6050::get_temperature_celsius() const noexcept
    {
        return this->get_temperature_raw().transform(
            [this](std::int16_t const raw) { return static_cast<float>(raw) / 340.0F + 36.53F; });
    }

    std::optional<Vec3D<float>> MPU6050::get_rotation_scaled() const noexcept
    {
        return this->get_rotation_raw().transform(
            [this](Vec3D<std::int16_t> const& raw) { return static_cast<Vec3D<float>>(raw) / this->gyro_scale_; });
    }

    std::optional<float> MPU6050::get_rotation_x_scaled() const noexcept
    {
        return this->get_rotation_x_raw().transform(
            [this](std::int16_t const raw) { return static_cast<float>(raw) / this->gyro_scale_; });
    }

    std::optional<float> MPU6050::get_rotation_y_scaled() const noexcept
    {
        return this->get_rotation_y_raw().transform(
            [this](std::int16_t const raw) { return static_cast<float>(raw) / this->gyro_scale_; });
    }

    std::optional<float> MPU6050::get_rotation_z_scaled() const noexcept
    {
        return this->get_rotation_z_raw().transform(
            [this](std::int16_t const raw) { return static_cast<float>(raw) / this->gyro_scale_; });
    }

    std::optional<Vec3D<float>> MPU6050::get_roll_pitch_yaw() const noexcept
    {
        return this->get_acceleration_scaled().transform([](Vec3D<float> const& accel) {
            return Vec3D<float>{std::atan2(accel.y, accel.z),
                                -std::atan2(accel.x, std::sqrt(accel.y * accel.y + accel.z * accel.z)),
                                0.0F};
        });
    }

    std::optional<float> MPU6050::get_roll() const noexcept
    {
        return this->get_acceleration_scaled().transform(
            [](Vec3D<float> const& accel) { return std::atan2(accel.y, accel.z); });
    }

    std::optional<float> MPU6050::get_pitch() const noexcept
    {
        return this->get_acceleration_scaled().transform([](Vec3D<float> const& accel) {
            return -std::atan2(accel.x, std::sqrt(accel.y * accel.y + accel.z * accel.z));
        });
    }

    std::optional<float> MPU6050::get_yaw() const noexcept
    {
        return this->get_acceleration_scaled().transform([](Vec3D<float> const& accel) { return 0.0F; });
    }

    void MPU6050::device_reset() const noexcept
    {
        this->set_pwr_mgmt_1_register(std::bit_cast<PWR_MGMT_1>(static_cast<std::uint8_t>(0b10000000)));
    }

    void MPU6050::device_wake_up() const noexcept
    {
        this->set_pwr_mgmt_1_register(std::bit_cast<PWR_MGMT_1>(static_cast<std::uint8_t>(0b00000000)));
        HAL_Delay(200);
    }

    bool MPU6050::is_valid_device_id() const noexcept
    {
        return this->get_device_id() == this->i2c_device_.dev_address();
    }

    std::uint8_t MPU6050::get_device_id() const noexcept
    {
        return std::bit_cast<std::uint8_t>(this->get_who_am_i_register());
    }

    void MPU6050::initialize(CONFIG const config,
                             ACCEL_CONFIG const accel_config,
                             GYRO_CONFIG const gyro_config,
                             SMPLRT_DIV const smplrt_div,
                             INT_PIN_CFG const int_pin_cfg,
                             INT_ENABLE const int_enable) noexcept
    {
        if (this->is_valid_device_id()) {
            this->device_reset();
            this->device_wake_up();
            this->set_config_register(config);
            this->set_accel_config_register(accel_config);
            this->set_gyro_config_register(gyro_config);
            this->set_smplrt_div_register(smplrt_div);
            this->set_int_pin_cfg_register(int_pin_cfg);
            this->set_int_enable_register(int_enable);
            this->initialized_ = true;
        }
    }

    void MPU6050::deinitialize() noexcept
    {
        if (this->is_valid_device_id()) {
            this->device_reset();
            this->initialized_ = false;
        }
    }

    std::optional<Vec3D<std::int16_t>> MPU6050::get_acceleration_raw() const noexcept
    {
        return this->initialized_ ? std::optional<Vec3D<std::int16_t>>{std::bit_cast<Vec3D<std::int16_t>>(
                                        this->get_accel_out_registers())}
                                  : std::optional<Vec3D<std::int16_t>>{std::nullopt};
    }

    std::optional<std::int16_t> MPU6050::get_acceleration_x_raw() const noexcept
    {
        return this->initialized_
                   ? std::optional<std::int16_t>{std::bit_cast<std::int16_t>(this->get_accel_xout_registers())}
                   : std::optional<std::int16_t>{std::nullopt};
    }

    std::optional<std::int16_t> MPU6050::get_acceleration_y_raw() const noexcept
    {
        return this->initialized_
                   ? std::optional<std::int16_t>{std::bit_cast<std::int16_t>(this->get_accel_yout_registers())}
                   : std::optional<std::int16_t>{std::nullopt};
    }

    std::optional<std::int16_t> MPU6050::get_acceleration_z_raw() const noexcept
    {
        return this->initialized_
                   ? std::optional<std::int16_t>{std::bit_cast<std::int16_t>(this->get_accel_zout_registers())}
                   : std::optional<std::int16_t>{std::nullopt};
    }

    std::optional<std::int16_t> MPU6050::get_temperature_raw() const noexcept
    {
        return this->initialized_
                   ? std::optional<std::int16_t>{std::bit_cast<std::int16_t>(this->get_temp_out_registers())}
                   : std::optional<std::int16_t>{std::nullopt};
    }

    std::optional<Vec3D<std::int16_t>> MPU6050::get_rotation_raw() const noexcept
    {
        return this->initialized_ ? std::optional<Vec3D<std::int16_t>>{std::bit_cast<Vec3D<std::int16_t>>(
                                        this->get_gyro_out_registers())}
                                  : std::optional<Vec3D<std::int16_t>>{std::nullopt};
    }

    std::optional<std::int16_t> MPU6050::get_rotation_x_raw() const noexcept
    {
        return this->initialized_
                   ? std::optional<std::int16_t>{std::bit_cast<std::int16_t>(this->get_gyro_xout_registers())}
                   : std::optional<std::int16_t>{std::nullopt};
    }

    std::optional<std::int16_t> MPU6050::get_rotation_y_raw() const noexcept
    {
        return this->initialized_
                   ? std::optional<std::int16_t>{std::bit_cast<std::int16_t>(this->get_gyro_yout_registers())}
                   : std::optional<std::int16_t>{std::nullopt};
    }

    std::optional<std::int16_t> MPU6050::get_rotation_z_raw() const noexcept
    {
        return this->initialized_
                   ? std::optional<std::int16_t>{std::bit_cast<std::int16_t>(this->get_gyro_zout_registers())}
                   : std::optional<std::int16_t>{std::nullopt};
    }

    void MPU6050::set_xg_offs_tc_register(XG_OFFS_TC const xg_offs_tc) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::XG_OFFS_TC), std::bit_cast<std::uint8_t>(xg_offs_tc));
    }

    XG_OFFS_TC MPU6050::get_xg_offs_tc_register() const noexcept
    {
        return std::bit_cast<XG_OFFS_TC>(this->i2c_device_.read_byte(std::to_underlying(RA::XG_OFFS_TC)));
    }

    void MPU6050::set_yg_offs_tc_register(YG_OFFS_TC const yg_offs_tc) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::YG_OFFS_TC), std::bit_cast<std::uint8_t>(yg_offs_tc));
    }

    YG_OFFS_TC MPU6050::get_yg_offs_tc_register() const noexcept
    {
        return std::bit_cast<YG_OFFS_TC>(this->i2c_device_.read_byte(std::to_underlying(RA::YG_OFFS_TC)));
    }

    void MPU6050::set_zg_offs_tc_register(ZG_OFFS_TC const zg_offs_tc) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::ZG_OFFS_TC), std::bit_cast<std::uint8_t>(zg_offs_tc));
    }

    ZG_OFFS_TC MPU6050::get_zg_offs_tc_register() const noexcept
    {
        return std::bit_cast<ZG_OFFS_TC>(this->i2c_device_.read_byte(std::to_underlying(RA::ZG_OFFS_TC)));
    }

    void MPU6050::set_x_fine_gain_register(X_FINE_GAIN const x_fine_gain) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::X_FINE_GAIN), std::bit_cast<std::uint8_t>(x_fine_gain));
    }

    X_FINE_GAIN MPU6050::get_x_fine_gain_register() const noexcept
    {
        return std::bit_cast<X_FINE_GAIN>(this->i2c_device_.read_byte(std::to_underlying(RA::X_FINE_GAIN)));
    }

    void MPU6050::set_y_fine_gain_register(Y_FINE_GAIN const y_fine_gain) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::Y_FINE_GAIN), std::bit_cast<std::uint8_t>(y_fine_gain));
    }

    Y_FINE_GAIN MPU6050::get_y_fine_gain_register() const noexcept
    {
        return std::bit_cast<Y_FINE_GAIN>(this->i2c_device_.read_byte(std::to_underlying(RA::Y_FINE_GAIN)));
    }

    void MPU6050::set_z_fine_gain_register(Z_FINE_GAIN const z_fine_gain) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::Z_FINE_GAIN), std::bit_cast<std::uint8_t>(z_fine_gain));
    }

    Z_FINE_GAIN MPU6050::get_z_fine_gain_register() const noexcept
    {
        return std::bit_cast<Z_FINE_GAIN>(this->i2c_device_.read_byte(std::to_underlying(RA::Z_FINE_GAIN)));
    }

    void MPU6050::set_xa_offs_registers(XA_OFFS const xa_offs) const noexcept
    {
        this->i2c_device_.write_word(std::to_underlying(RA::XA_OFFS_H), std::bit_cast<std::uint16_t>(xa_offs));
    }

    XA_OFFS MPU6050::get_xa_offs_registers() const noexcept
    {
        return std::bit_cast<XA_OFFS>(this->i2c_device_.read_word(std::to_underlying(RA::XA_OFFS_H)));
    }

    void MPU6050::set_ya_offs_registers(YA_OFFS const ya_offs) const noexcept
    {
        this->i2c_device_.write_word(std::to_underlying(RA::YA_OFFS_H), std::bit_cast<std::uint16_t>(ya_offs));
    }

    YA_OFFS MPU6050::get_ya_offs_registers() const noexcept
    {
        return std::bit_cast<YA_OFFS>(this->i2c_device_.read_word(std::to_underlying(RA::YA_OFFS_H)));
    }

    void MPU6050::set_za_offs_registers(ZA_OFFS const za_offs) const noexcept
    {
        this->i2c_device_.write_word(std::to_underlying(RA::ZA_OFFS_H), std::bit_cast<std::uint16_t>(za_offs));
    }

    ZA_OFFS MPU6050::get_za_offs_registers() const noexcept
    {
        return std::bit_cast<ZA_OFFS>(this->i2c_device_.read_word(std::to_underlying(RA::ZA_OFFS_H)));
    }

    void MPU6050::set_xg_offs_usr_registers(XG_OFFS_USR const xg_offs_usr) const noexcept
    {
        this->i2c_device_.write_word(std::to_underlying(RA::XG_OFFS_USRH), std::bit_cast<std::uint16_t>(xg_offs_usr));
    }

    XG_OFFS_USR MPU6050::get_xg_offs_usr_registers() const noexcept
    {
        return std::bit_cast<XG_OFFS_USR>(this->i2c_device_.read_word(std::to_underlying(RA::XG_OFFS_USRH)));
    }

    void MPU6050::set_yg_offs_usr_registers(YG_OFFS_USR const yg_offs_usr) const noexcept
    {
        this->i2c_device_.write_word(std::to_underlying(RA::YG_OFFS_USRH), std::bit_cast<std::uint16_t>(yg_offs_usr));
    }

    YG_OFFS_USR MPU6050::get_yg_offs_usr_registers() const noexcept
    {
        return std::bit_cast<YG_OFFS_USR>(this->i2c_device_.read_word(std::to_underlying(RA::YG_OFFS_USRH)));
    }

    void MPU6050::set_zg_offs_usr_registers(ZG_OFFS_USR const zg_offs_usr) const noexcept
    {
        this->i2c_device_.write_word(std::to_underlying(RA::ZG_OFFS_USRH), std::bit_cast<std::uint16_t>(zg_offs_usr));
    }

    ZG_OFFS_USR MPU6050::get_zg_offs_usr_register() const noexcept
    {
        return std::bit_cast<ZG_OFFS_USR>(this->i2c_device_.read_word(std::to_underlying(RA::ZG_OFFS_USRH)));
    }

    void MPU6050::set_self_test_x_register(SELF_TEST_X const self_test_x) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::SELF_TEST_X), std::bit_cast<std::uint8_t>(self_test_x));
    }

    SELF_TEST_X MPU6050::get_self_test_x_register() const noexcept
    {
        return std::bit_cast<SELF_TEST_X>(this->i2c_device_.read_byte(std::to_underlying(RA::SELF_TEST_X)));
    }

    void MPU6050::set_self_test_y_register(SELF_TEST_Y const self_test_y) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::SELF_TEST_Y), std::bit_cast<std::uint8_t>(self_test_y));
    }

    SELF_TEST_Y MPU6050::get_self_test_y_register() const noexcept
    {
        return std::bit_cast<SELF_TEST_Y>(this->i2c_device_.read_byte(std::to_underlying(RA::SELF_TEST_Y)));
    }

    void MPU6050::set_self_test_z_register(SELF_TEST_Z const self_test_z) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::SELF_TEST_Z), std::bit_cast<std::uint8_t>(self_test_z));
    }

    SELF_TEST_Z MPU6050::get_self_test_z_register() const noexcept
    {
        return std::bit_cast<SELF_TEST_Z>(this->i2c_device_.read_byte(std::to_underlying(RA::SELF_TEST_Z)));
    }

    void MPU6050::set_self_test_a_register(SELF_TEST_A const self_test_a) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::SELF_TEST_A), std::bit_cast<std::uint8_t>(self_test_a));
    }

    SELF_TEST_A MPU6050::get_self_test_a_register() const noexcept
    {
        return std::bit_cast<SELF_TEST_A>(this->i2c_device_.read_byte(std::to_underlying(RA::SELF_TEST_A)));
    }

    void MPU6050::set_smplrt_div_register(SMPLRT_DIV const smplrt_div) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::SMPLRT_DIV), std::bit_cast<std::uint8_t>(smplrt_div));
    }

    SMPLRT_DIV MPU6050::get_smplrt_div_register() const noexcept
    {
        return std::bit_cast<SMPLRT_DIV>(this->i2c_device_.read_byte(std::to_underlying(RA::SMPLRT_DIV)));
    }

    void MPU6050::set_config_register(CONFIG const config) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::CONFIG), std::bit_cast<std::uint8_t>(config));
    }

    CONFIG MPU6050::get_config_register() const noexcept
    {
        return std::bit_cast<CONFIG>(this->i2c_device_.read_byte(std::to_underlying(RA::CONFIG)));
    }

    void MPU6050::set_gyro_config_register(GYRO_CONFIG const gyro_config) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::GYRO_CONFIG), std::bit_cast<std::uint8_t>(gyro_config));
    }

    GYRO_CONFIG MPU6050::get_gyro_config_register() const noexcept
    {
        return std::bit_cast<GYRO_CONFIG>(this->i2c_device_.read_byte(std::to_underlying(RA::GYRO_CONFIG)));
    }

    void MPU6050::set_accel_config_register(ACCEL_CONFIG const accel_config) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::ACCEL_CONFIG), std::bit_cast<std::uint8_t>(accel_config));
    }

    ACCEL_CONFIG MPU6050::get_accel_config_register() const noexcept
    {
        return std::bit_cast<ACCEL_CONFIG>(this->i2c_device_.read_byte(std::to_underlying(RA::ACCEL_CONFIG)));
    }

    void MPU6050::set_ff_thr_register(FF_THR const ff_thr) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::FF_THR), std::bit_cast<std::uint8_t>(ff_thr));
    }

    FF_THR MPU6050::get_ff_thr_register() const noexcept
    {
        return std::bit_cast<FF_THR>(this->i2c_device_.read_byte(std::to_underlying(RA::FF_THR)));
    }

    void MPU6050::set_ff_dur_register(FF_DUR const ff_dur) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::FF_DUR), std::bit_cast<std::uint8_t>(ff_dur));
    }

    FF_DUR MPU6050::get_ff_dur_register() const noexcept
    {
        return std::bit_cast<FF_DUR>(this->i2c_device_.read_byte(std::to_underlying(RA::FF_DUR)));
    }

    void MPU6050::set_mot_thr_register(MOT_THR const mot_thr) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::MOT_THR), std::bit_cast<std::uint8_t>(mot_thr));
    }

    MOT_THR MPU6050::get_mot_thr_register() const noexcept
    {
        return std::bit_cast<MOT_THR>(this->i2c_device_.read_byte(std::to_underlying(RA::MOT_THR)));
    }

    void MPU6050::set_mot_dur_register(MOT_DUR const mot_dur) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::MOT_DUR), std::bit_cast<std::uint8_t>(mot_dur));
    }

    MOT_DUR MPU6050::get_mot_dur_register() const noexcept
    {
        return std::bit_cast<MOT_DUR>(this->i2c_device_.read_byte(std::to_underlying(RA::MOT_DUR)));
    }

    ZRMOT_THR MPU6050::get_zrmot_thr_register() const noexcept
    {
        return std::bit_cast<ZRMOT_THR>(this->i2c_device_.read_byte(std::to_underlying(RA::ZRMOT_THR)));
    }

    void MPU6050::set_zrmot_thr_register(ZRMOT_THR const zrmot_thr) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::ZRMOT_THR), std::bit_cast<std::uint8_t>(zrmot_thr));
    }

    void MPU6050::set_zrmot_dur_register(ZRMOT_DUR const zrmot_dur) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::ZRMOT_DUR), std::bit_cast<std::uint8_t>(zrmot_dur));
    }

    ZRMOT_DUR MPU6050::get_zrmot_dur_register() const noexcept
    {
        return std::bit_cast<ZRMOT_DUR>(this->i2c_device_.read_byte(std::to_underlying(RA::ZRMOT_DUR)));
    }

    void MPU6050::set_fifo_en_register(FIFO_EN const fifo_en) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::FIFO_EN), std::bit_cast<std::uint8_t>(fifo_en));
    }

    FIFO_EN MPU6050::get_fifo_en_register() const noexcept
    {
        return std::bit_cast<FIFO_EN>(this->i2c_device_.read_byte(std::to_underlying(RA::FIFO_EN)));
    }

    void MPU6050::set_i2c_mst_ctrl_register(I2C_MST_CTRL const i2c_mst_ctrl) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::I2C_MST_CTRL), std::bit_cast<std::uint8_t>(i2c_mst_ctrl));
    }

    I2C_MST_CTRL MPU6050::get_i2c_mst_ctrl_register() const noexcept
    {
        return std::bit_cast<I2C_MST_CTRL>(this->i2c_device_.read_byte(std::to_underlying(RA::I2C_MST_CTRL)));
    }

    void MPU6050::set_i2c_slv_addr_register(SlaveNum const slave_num, I2C_SLV_ADDR const i2c_slv_addr) const noexcept
    {
        this->i2c_device_.write_byte(static_cast<uint8_t>(std::to_underlying(RA::I2C_SLV0_ADDR) +
                                                          static_cast<std::uint8_t>(3) * std::to_underlying(slave_num)),
                                     std::bit_cast<std::uint8_t>(i2c_slv_addr));
    }

    void MPU6050::set_i2c_slv_reg_register(SlaveNum const slave_num, I2C_SLV_REG const i2c_slv_reg) const noexcept
    {
        this->i2c_device_.write_byte(static_cast<uint8_t>(std::to_underlying(RA::I2C_SLV0_REG) +
                                                          static_cast<std::uint8_t>(3) * std::to_underlying(slave_num)),
                                     std::bit_cast<std::uint8_t>(i2c_slv_reg));
    }

    void MPU6050::set_i2c_slv_ctrl_register(SlaveNum const slave_num, I2C_SLV_CTRL const i2c_slv_ctrl) const noexcept
    {
        this->i2c_device_.write_byte(static_cast<uint8_t>(std::to_underlying(RA::I2C_SLV0_CTRL) +
                                                          static_cast<std::uint8_t>(3) * std::to_underlying(slave_num)),
                                     std::bit_cast<std::uint8_t>(i2c_slv_ctrl));
    }

    void MPU6050::set_i2c_slv4_addr_register(I2C_SLV4_ADDR const i2c_slv4_addr) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::I2C_SLV4_ADDR), std::bit_cast<std::uint8_t>(i2c_slv4_addr));
    }

    void MPU6050::set_i2c_slv4_reg_register(I2C_SLV4_REG const i2c_slv4_reg) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::I2C_SLV4_REG), std::bit_cast<std::uint8_t>(i2c_slv4_reg));
    }

    void MPU6050::set_i2c_slv4_ctrl_register(I2C_SLV4_CTRL const i2c_slv4_ctrl) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::I2C_SLV4_CTRL), std::bit_cast<std::uint8_t>(i2c_slv4_ctrl));
    }

    void MPU6050::set_i2c_slv4_do_register(I2C_SLV4_DO const i2c_slv4_do) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::I2C_SLV4_DO), std::bit_cast<std::uint8_t>(i2c_slv4_do));
    }

    I2C_SLV4_DI MPU6050::get_i2c_slv4_di_register() const noexcept
    {
        return std::bit_cast<I2C_SLV4_DI>(this->i2c_device_.read_byte(std::to_underlying(RA::I2C_SLV4_DI)));
    }

    I2C_MST_STATUS MPU6050::get_i2c_mst_status_register() const noexcept
    {
        return std::bit_cast<I2C_MST_STATUS>(this->i2c_device_.read_byte(std::to_underlying(RA::I2C_MST_STATUS)));
    }

    void MPU6050::set_int_pin_cfg_register(INT_PIN_CFG const int_pin_cfg) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::INT_PIN_CFG), std::bit_cast<std::uint8_t>(int_pin_cfg));
    }

    INT_PIN_CFG MPU6050::get_int_pin_cfg_register() const noexcept
    {
        return std::bit_cast<INT_PIN_CFG>(this->i2c_device_.read_byte(std::to_underlying(RA::INT_PIN_CFG)));
    }

    void MPU6050::set_int_enable_register(INT_ENABLE const int_enable) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::INT_ENABLE), std::bit_cast<std::uint8_t>(int_enable));
    }

    INT_ENABLE MPU6050::get_int_enable_register() const noexcept
    {
        return std::bit_cast<INT_ENABLE>(this->i2c_device_.read_byte(std::to_underlying(RA::INT_ENABLE)));
    }

    DMP_INT_STATUS MPU6050::get_dmp_int_status_register() const noexcept
    {
        return std::bit_cast<DMP_INT_STATUS>(this->i2c_device_.read_byte(std::to_underlying(RA::DMP_INT_STATUS)));
    }

    TC MPU6050::get_tc_register() const noexcept
    {
        return std::bit_cast<TC>(static_cast<std::uint8_t>(0));
    }

    INT_STATUS MPU6050::get_int_status_register() const noexcept
    {
        return std::bit_cast<INT_STATUS>(this->i2c_device_.read_byte(std::to_underlying(RA::INT_STATUS)));
    }

    ACCEL_OUT MPU6050::get_accel_out_registers() const noexcept
    {
        return std::bit_cast<ACCEL_OUT>(this->i2c_device_.read_words<3UL>(std::to_underlying(RA::ACCEL_XOUT_H)));
    }

    ACCEL_XOUT MPU6050::get_accel_xout_registers() const noexcept
    {
        return std::bit_cast<ACCEL_XOUT>(this->i2c_device_.read_word(std::to_underlying(RA::ACCEL_XOUT_H)));
    }

    ACCEL_YOUT MPU6050::get_accel_yout_registers() const noexcept
    {
        return std::bit_cast<ACCEL_YOUT>(this->i2c_device_.read_word(std::to_underlying(RA::ACCEL_YOUT_H)));
    }

    ACCEL_ZOUT MPU6050::get_accel_zout_registers() const noexcept
    {
        return std::bit_cast<ACCEL_ZOUT>(this->i2c_device_.read_word(std::to_underlying(RA::ACCEL_ZOUT_H)));
    }

    TEMP_OUT MPU6050::get_temp_out_registers() const noexcept
    {
        return std::bit_cast<TEMP_OUT>(this->i2c_device_.read_word(std::to_underlying(RA::TEMP_OUT_H)));
    }

    GYRO_OUT MPU6050::get_gyro_out_registers() const noexcept
    {
        return std::bit_cast<GYRO_OUT>(this->i2c_device_.read_words<3UL>(std::to_underlying(RA::GYRO_XOUT_H)));
    }

    GYRO_XOUT MPU6050::get_gyro_xout_registers() const noexcept
    {
        return std::bit_cast<GYRO_XOUT>(this->i2c_device_.read_word(std::to_underlying(RA::GYRO_XOUT_H)));
    }

    GYRO_YOUT MPU6050::get_gyro_yout_registers() const noexcept
    {
        return std::bit_cast<GYRO_YOUT>(this->i2c_device_.read_word(std::to_underlying(RA::GYRO_YOUT_H)));
    }

    GYRO_ZOUT MPU6050::get_gyro_zout_registers() const noexcept
    {
        return std::bit_cast<GYRO_ZOUT>(this->i2c_device_.read_word(std::to_underlying(RA::GYRO_ZOUT_H)));
    }

    EXT_SENS_DATA MPU6050::get_ext_sens_data_register(std::uint8_t const position) const noexcept
    {
        return std::bit_cast<EXT_SENS_DATA>(
            this->i2c_device_.read_byte(std::to_underlying(RA::EXT_SENS_DATA_00) + position));
    }

    MOT_DETECT_STATUS MPU6050::get_mot_detect_status_registet() const noexcept
    {
        return std::bit_cast<MOT_DETECT_STATUS>(this->i2c_device_.read_byte(std::to_underlying(RA::MOT_DETECT_STATUS)));
    }

    void MPU6050::set_i2c_slv_do_register(SlaveNum const slave_num, I2C_SLV_DO const i2c_slv_do) const noexcept
    {
        this->i2c_device_.write_byte(
            static_cast<std::uint8_t>(std::to_underlying(RA::I2C_SLV0_DO) +
                                      static_cast<std::uint8_t>(3) * std::to_underlying(slave_num)),
            std::bit_cast<std::uint8_t>(i2c_slv_do));
    }

    void MPU6050::set_i2c_mst_delay_ctrl_register(I2C_MST_DELAY_CTRL const i2c_mst_delay_ctrl) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::I2C_MST_DELAY_CTRL),
                                     std::bit_cast<std::uint8_t>(i2c_mst_delay_ctrl));
    }

    I2C_MST_DELAY_CTRL MPU6050::get_i2c_mst_delay_ctrl_register() const noexcept
    {
        return std::bit_cast<I2C_MST_DELAY_CTRL>(
            this->i2c_device_.read_byte(std::to_underlying(RA::I2C_MST_DELAY_CTRL)));
    }

    void MPU6050::set_signal_path_reset_register(SIGNAL_PATH_RESET const signal_path_reset) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::SIGNAL_PATH_RESET),
                                     std::bit_cast<std::uint8_t>(signal_path_reset));
    }

    SIGNAL_PATH_RESET MPU6050::get_signal_path_reset_register() const noexcept
    {
        return std::bit_cast<SIGNAL_PATH_RESET>(this->i2c_device_.read_byte(std::to_underlying(RA::SIGNAL_PATH_RESET)));
    }

    void MPU6050::set_mot_detect_ctrl_register(MOT_DETECT_CTRL const mot_detect_ctrl) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::MOT_DETECT_CTRL),
                                     std::bit_cast<std::uint8_t>(mot_detect_ctrl));
    }

    MOT_DETECT_CTRL MPU6050::get_mot_detect_ctrl_register() const noexcept
    {
        return std::bit_cast<MOT_DETECT_CTRL>(this->i2c_device_.read_byte(std::to_underlying(RA::MOT_DETECT_CTRL)));
    }

    void MPU6050::set_user_ctrl_register(USER_CTRL const user_ctrl) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::USER_CTRL), std::bit_cast<std::uint8_t>(user_ctrl));
    }

    USER_CTRL MPU6050::get_user_ctrl_register() const noexcept
    {
        return std::bit_cast<USER_CTRL>(this->i2c_device_.read_byte(std::to_underlying(RA::USER_CTRL)));
    }

    void MPU6050::set_pwr_mgmt_1_register(PWR_MGMT_1 const pwr_mgmt_1) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::PWR_MGMT_1), std::bit_cast<std::uint8_t>(pwr_mgmt_1));
    }

    PWR_MGMT_1 MPU6050::get_pwr_mgmt_1_register() const noexcept
    {
        return std::bit_cast<PWR_MGMT_1>(this->i2c_device_.read_byte(std::to_underlying(RA::PWR_MGMT_1)));
    }

    void MPU6050::set_pwr_mgmt_2_register(PWR_MGMT_2 const pwr_mgmt_2) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::PWR_MGMT_2), std::bit_cast<std::uint8_t>(pwr_mgmt_2));
    }

    PWR_MGMT_2 MPU6050::set_pwr_mgmt_2_register() const noexcept
    {
        return std::bit_cast<PWR_MGMT_2>(this->i2c_device_.read_byte(std::to_underlying(RA::PWR_MGMT_2)));
    }

    void MPU6050::set_bank_sel_register(BANK_SEL const bank_sel) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::BANK_SEL), std::bit_cast<std::uint8_t>(bank_sel));
    }

    BANK_SEL MPU6050::get_bank_sel_register() const noexcept
    {
        return std::bit_cast<BANK_SEL>(this->i2c_device_.read_byte(std::to_underlying(RA::BANK_SEL)));
    }

    void MPU6050::set_mem_start_addr_register(MEM_START_ADDR const mem_start_addr) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::MEM_START_ADDR),
                                     std::bit_cast<std::uint8_t>(mem_start_addr));
    }

    MEM_START_ADDR MPU6050::get_mem_start_addr_register() const noexcept
    {
        return std::bit_cast<MEM_START_ADDR>(this->i2c_device_.read_byte(std::to_underlying(RA::MEM_START_ADDR)));
    }

    void MPU6050::set_mem_r_w_register(MEM_R_W const mem_r_w) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::MEM_R_W), std::bit_cast<std::uint8_t>(mem_r_w));
    }

    MEM_R_W MPU6050::get_mem_r_w_register() const noexcept
    {
        return std::bit_cast<MEM_R_W>(this->i2c_device_.read_byte(std::to_underlying(RA::MEM_R_W)));
    }

    void MPU6050::set_dmp_cfg_1_register(DMP_CFG_1 const dmp_cfg_1) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::DMP_CFG_1), std::bit_cast<std::uint8_t>(dmp_cfg_1));
    }

    DMP_CFG_1 MPU6050::get_dmp_cfg_1_register() const noexcept
    {
        return std::bit_cast<DMP_CFG_1>(this->i2c_device_.read_byte(std::to_underlying(RA::DMP_CFG_1)));
    }

    void MPU6050::set_dmp_cfg_2_register(DMP_CFG_2 const dmp_cfg_2) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::DMP_CFG_2), std::bit_cast<std::uint8_t>(dmp_cfg_2));
    }

    DMP_CFG_2 MPU6050::get_dmp_cfg_2_register() const noexcept
    {
        return std::bit_cast<DMP_CFG_2>(this->i2c_device_.read_byte(std::to_underlying(RA::DMP_CFG_2)));
    }

    FIFO_COUNT MPU6050::get_fifo_count_registers() const noexcept
    {
        return std::bit_cast<FIFO_COUNT>(this->i2c_device_.read_word(std::to_underlying(RA::FIFO_COUNTH)));
    }

    void MPU6050::set_fifo_r_w_register(FIFO_R_W const fifo_r_w) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::FIFO_R_W), std::bit_cast<std::uint8_t>(fifo_r_w));
    }

    FIFO_R_W MPU6050::get_fifo_r_w_register() const noexcept
    {
        return std::bit_cast<FIFO_R_W>(this->i2c_device_.read_byte(std::to_underlying(RA::FIFO_R_W)));
    }

    WHO_AM_I MPU6050::get_who_am_i_register() const noexcept
    {
        return std::bit_cast<WHO_AM_I>(this->i2c_device_.read_byte(std::to_underlying(RA::WHO_AM_I)));
    }

}; // namespace MPU6050