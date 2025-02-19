#ifndef MPU6050_HPP
#define MPU6050_HPP

#include "i2c_device.hpp"
#include "mpu6050_config.hpp"
#include "mpu6050_register.hpp"
#include "stm32l4xx_hal.h"
#include "vector3d.hpp"
#include <cstddef>
#include <cstdint>
#include <optional>

namespace MPU6050 {

    struct MPU6050 {
    public:
        MPU6050() noexcept = default;
        MPU6050(I2CDevice&& i2c_device,
                CONFIG const config,
                ACCEL_CONFIG const accel_config,
                GYRO_CONFIG const gyro_config,
                SMPLRT_DIV const smplrt_div,
                INT_PIN_CFG const int_pin_cfg,
                INT_ENABLE const int_enable) noexcept;

        MPU6050(MPU6050 const& other) noexcept = delete;
        MPU6050(MPU6050&& other) noexcept = default;

        MPU6050& operator=(MPU6050 const& other) noexcept = delete;
        MPU6050& operator=(MPU6050&& other) noexcept = default;

        ~MPU6050() noexcept;

        /* meters per square second */
        [[nodiscard]] std::optional<Vec3D<float>> get_acceleration_scaled() const noexcept;
        [[nodiscard]] std::optional<float> get_acceleration_x_scaled() const noexcept;
        [[nodiscard]] std::optional<float> get_acceleration_y_scaled() const noexcept;
        [[nodiscard]] std::optional<float> get_acceleration_z_scaled() const noexcept;

        /* celsius */
        [[nodiscard]] std::optional<float> get_temperature_celsius() const noexcept;

        /* radians */
        [[nodiscard]] std::optional<Vec3D<float>> get_rotation_scaled() const noexcept;
        [[nodiscard]] std::optional<float> get_rotation_x_scaled() const noexcept;
        [[nodiscard]] std::optional<float> get_rotation_y_scaled() const noexcept;
        [[nodiscard]] std::optional<float> get_rotation_z_scaled() const noexcept;

        /* radians */
        [[nodiscard]] std::optional<Vec3D<float>> get_roll_pitch_yaw() const noexcept;
        [[nodiscard]] std::optional<float> get_roll() const noexcept;
        [[nodiscard]] std::optional<float> get_pitch() const noexcept;
        [[nodiscard]] std::optional<float> get_yaw() const noexcept;

    private:
        void device_reset() const noexcept;
        void device_wake_up() const noexcept;
        bool is_valid_device_id() const noexcept;
        std::uint8_t get_device_id() const noexcept;

        std::optional<Vec3D<std::int16_t>> get_acceleration_raw() const noexcept;
        std::optional<std::int16_t> get_acceleration_x_raw() const noexcept;
        std::optional<std::int16_t> get_acceleration_y_raw() const noexcept;
        std::optional<std::int16_t> get_acceleration_z_raw() const noexcept;

        std::optional<std::int16_t> get_temperature_raw() const noexcept;

        std::optional<Vec3D<std::int16_t>> get_rotation_raw() const noexcept;
        std::optional<std::int16_t> get_rotation_x_raw() const noexcept;
        std::optional<std::int16_t> get_rotation_y_raw() const noexcept;
        std::optional<std::int16_t> get_rotation_z_raw() const noexcept;

        void initialize(CONFIG const config,
                        ACCEL_CONFIG const accel_config,
                        GYRO_CONFIG const gyro_config,
                        SMPLRT_DIV const smplrt_div,
                        INT_PIN_CFG const int_pin_cfg,
                        INT_ENABLE const int_enable) noexcept;

        void deinitialize() noexcept;

        void set_xg_offs_tc_register(XG_OFFS_TC const xg_offs_tc) const noexcept;
        XG_OFFS_TC get_xg_offs_tc_register() const noexcept;

        void set_yg_offs_tc_register(YG_OFFS_TC const yg_offs_tc) const noexcept;
        YG_OFFS_TC get_yg_offs_tc_register() const noexcept;

        void set_zg_offs_tc_register(ZG_OFFS_TC const zg_offs_tc) const noexcept;
        ZG_OFFS_TC get_zg_offs_tc_register() const noexcept;

        void set_x_fine_gain_register(X_FINE_GAIN const x_fine_gain) const noexcept;
        X_FINE_GAIN get_x_fine_gain_register() const noexcept;

        void set_y_fine_gain_register(Y_FINE_GAIN const y_fine_gain) const noexcept;
        Y_FINE_GAIN get_y_fine_gain_register() const noexcept;

        void set_z_fine_gain_register(Z_FINE_GAIN const z_fine_gain) const noexcept;
        Z_FINE_GAIN get_z_fine_gain_register() const noexcept;

        void set_xa_offs_registers(XA_OFFS const xa_offs) const noexcept;
        XA_OFFS get_xa_offs_registers() const noexcept;

        void set_ya_offs_registers(YA_OFFS const ya_offs) const noexcept;
        YA_OFFS get_ya_offs_registers() const noexcept;

        void set_za_offs_registers(ZA_OFFS const za_offs) const noexcept;
        ZA_OFFS get_za_offs_registers() const noexcept;

        void set_xg_offs_usr_registers(XG_OFFS_USR const xg_offs_usr) const noexcept;
        XG_OFFS_USR get_xg_offs_usr_registers() const noexcept;

        void set_yg_offs_usr_registers(YG_OFFS_USR const yg_offs_usr) const noexcept;
        YG_OFFS_USR get_yg_offs_usr_registers() const noexcept;

        void set_zg_offs_usr_registers(ZG_OFFS_USR const zg_offs_usr) const noexcept;
        ZG_OFFS_USR get_zg_offs_usr_register() const noexcept;

        void set_self_test_x_register(SELF_TEST_X const self_test_x) const noexcept;
        SELF_TEST_X get_self_test_x_register() const noexcept;

        void set_self_test_y_register(SELF_TEST_Y const self_test_y) const noexcept;
        SELF_TEST_Y get_self_test_y_register() const noexcept;

        void set_self_test_z_register(SELF_TEST_Z const self_test_z) const noexcept;
        SELF_TEST_Z get_self_test_z_register() const noexcept;

        void set_self_test_a_register(SELF_TEST_A const self_test_a) const noexcept;
        SELF_TEST_A get_self_test_a_register() const noexcept;

        void set_smplrt_div_register(SMPLRT_DIV const smplrt_div) const noexcept;
        SMPLRT_DIV get_smplrt_div_register() const noexcept;

        void set_config_register(CONFIG const config) const noexcept;
        CONFIG get_config_register() const noexcept;

        void set_gyro_config_register(GYRO_CONFIG const gyro_config) const noexcept;
        GYRO_CONFIG get_gyro_config_register() const noexcept;

        void set_accel_config_register(ACCEL_CONFIG const accel_config) const noexcept;
        ACCEL_CONFIG get_accel_config_register() const noexcept;

        void set_ff_thr_register(FF_THR const ff_thr) const noexcept;
        FF_THR get_ff_thr_register() const noexcept;

        void set_ff_dur_register(FF_DUR const ff_dur) const noexcept;
        FF_DUR get_ff_dur_register() const noexcept;

        void set_mot_thr_register(MOT_THR const mot_thr) const noexcept;
        MOT_THR get_mot_thr_register() const noexcept;

        void set_mot_dur_register(MOT_DUR const mot_dur) const noexcept;
        MOT_DUR get_mot_dur_register() const noexcept;

        void set_zrmot_thr_register(ZRMOT_THR const zrmot_thr) const noexcept;
        ZRMOT_THR get_zrmot_thr_register() const noexcept;

        void set_zrmot_dur_register(ZRMOT_DUR const zrmot_dur) const noexcept;
        ZRMOT_DUR get_zrmot_dur_register() const noexcept;

        void set_fifo_en_register(FIFO_EN const fifo_en) const noexcept;
        FIFO_EN get_fifo_en_register() const noexcept;

        void set_i2c_mst_ctrl_register(I2C_MST_CTRL const i2c_mst_ctrl) const noexcept;
        I2C_MST_CTRL get_i2c_mst_ctrl_register() const noexcept;

        void set_i2c_slv_addr_register(SlaveNum const slave_num, I2C_SLV_ADDR const i2c_slv_addr) const noexcept;
        void set_i2c_slv_reg_register(SlaveNum const slave_num, I2C_SLV_REG const i2c_slv_reg) const noexcept;
        void set_i2c_slv_ctrl_register(SlaveNum const slave_num, I2C_SLV_CTRL const i2c_slv_ctrl) const noexcept;

        void set_i2c_slv4_addr_register(I2C_SLV4_ADDR const i2c_slv4_addr) const noexcept;
        void set_i2c_slv4_reg_register(I2C_SLV4_REG const i2c_slv4_reg) const noexcept;
        void set_i2c_slv4_ctrl_register(I2C_SLV4_CTRL const i2c_slv4_ctrl) const noexcept;
        void set_i2c_slv4_do_register(I2C_SLV4_DO const i2c_slv4_do) const noexcept;
        I2C_SLV4_DI get_i2c_slv4_di_register() const noexcept;

        I2C_MST_STATUS get_i2c_mst_status_register() const noexcept;

        void set_int_pin_cfg_register(INT_PIN_CFG const int_pin_cfg) const noexcept;
        INT_PIN_CFG get_int_pin_cfg_register() const noexcept;

        void set_int_enable_register(INT_ENABLE const int_enable) const noexcept;
        INT_ENABLE get_int_enable_register() const noexcept;

        DMP_INT_STATUS get_dmp_int_status_register() const noexcept;

        TC get_tc_register() const noexcept;

        INT_STATUS get_int_status_register() const noexcept;

        ACCEL_OUT get_accel_out_registers() const noexcept;
        ACCEL_XOUT get_accel_xout_registers() const noexcept;
        ACCEL_YOUT get_accel_yout_registers() const noexcept;
        ACCEL_ZOUT get_accel_zout_registers() const noexcept;

        TEMP_OUT get_temp_out_registers() const noexcept;

        GYRO_OUT get_gyro_out_registers() const noexcept;
        GYRO_XOUT get_gyro_xout_registers() const noexcept;
        GYRO_YOUT get_gyro_yout_registers() const noexcept;
        GYRO_ZOUT get_gyro_zout_registers() const noexcept;

        EXT_SENS_DATA get_ext_sens_data_register(std::uint8_t const position) const noexcept;

        MOT_DETECT_STATUS get_mot_detect_status_registet() const noexcept;

        void set_i2c_slv_do_register(SlaveNum const slave_num, I2C_SLV_DO const i2c_slv_do) const noexcept;

        void set_i2c_mst_delay_ctrl_register(I2C_MST_DELAY_CTRL const i2c_mst_delay_ctrl) const noexcept;
        I2C_MST_DELAY_CTRL get_i2c_mst_delay_ctrl_register() const noexcept;

        void set_signal_path_reset_register(SIGNAL_PATH_RESET const signal_path_reset) const noexcept;
        SIGNAL_PATH_RESET get_signal_path_reset_register() const noexcept;

        void set_mot_detect_ctrl_register(MOT_DETECT_CTRL const mot_detect_ctrl) const noexcept;
        MOT_DETECT_CTRL get_mot_detect_ctrl_register() const noexcept;

        void set_user_ctrl_register(USER_CTRL const user_ctrl) const noexcept;
        USER_CTRL get_user_ctrl_register() const noexcept;

        void set_pwr_mgmt_1_register(PWR_MGMT_1 const pwr_mgmt_1) const noexcept;
        PWR_MGMT_1 get_pwr_mgmt_1_register() const noexcept;

        void set_pwr_mgmt_2_register(PWR_MGMT_2 const pwr_mgmt_2) const noexcept;
        PWR_MGMT_2 set_pwr_mgmt_2_register() const noexcept;

        void set_bank_sel_register(BANK_SEL const bank_sel) const noexcept;
        BANK_SEL get_bank_sel_register() const noexcept;

        void set_mem_start_addr_register(MEM_START_ADDR const mem_start_addr) const noexcept;
        MEM_START_ADDR get_mem_start_addr_register() const noexcept;

        void set_mem_r_w_register(MEM_R_W const mem_r_w) const noexcept;
        MEM_R_W get_mem_r_w_register() const noexcept;

        void set_dmp_cfg_1_register(DMP_CFG_1 const dmp_cfg_1) const noexcept;
        DMP_CFG_1 get_dmp_cfg_1_register() const noexcept;

        void set_dmp_cfg_2_register(DMP_CFG_2 const dmp_cfg_2) const noexcept;
        DMP_CFG_2 get_dmp_cfg_2_register() const noexcept;

        FIFO_COUNT get_fifo_count_registers() const noexcept;

        void set_fifo_r_w_register(FIFO_R_W const fifo_r_w) const noexcept;
        FIFO_R_W get_fifo_r_w_register() const noexcept;

        WHO_AM_I get_who_am_i_register() const noexcept;

        I2CDevice i2c_device_{};

        float accel_scale_{};
        float gyro_scale_{};

        bool initialized_{false};
    };

}; // namespace MPU6050

#endif // MPU6050_HPP